"""
Safety Shield (mini) — validate a 4-D action against the policy, repair if
possible, brake only as last resort.

Flow per tick (mirrors the grant's Safety Shield node, scaled down):

    check (trend-aware)                "does this action lead somewhere illegal?"
        -> repair operators, in order   kinematic clamp -> altitude fix -> geofence escape/slide
        -> re-check the repaired action P0 escape guard
        -> still illegal? BRAKE         fail-safe

Design rule learned the hard way: when the vehicle is ALREADY in violation
(below the altitude floor, inside a zone), the right output is a RECOVERY
action, not a freeze — braking would lock the violation in place forever.
So checks are trend-aware: "in violation but actively correcting" passes.

Guarantee: the action this module RETURNS never makes things worse — it is
either predicted-clean, actively recovering, or a full stop.
"""
from __future__ import annotations

from pydantic import BaseModel

from .geometry import fence_polygon, point_in_fence, push_out_direction
from .models import (
    Action4D,
    AltitudeEnvelope,
    KinematicEnvelope,
    Policy,
    PolygonFence,
    State,
)

BRAKE = Action4D(vx=0.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)


class Violation(BaseModel):
    rule_id: str
    category: str            # "kinematic" | "altitude" | "geofence"
    detail: str
    predicted_at_s: float    # how far into the lookahead it happens (0 = now)


class Repair(BaseModel):
    operator: str
    detail: str


class ShieldDecision(BaseModel):
    raw: Action4D
    emitted: Action4D
    violations: list[Violation] = []
    repairs: list[Repair] = []
    braked: bool = False

    @property
    def touched(self) -> bool:
        return bool(self.violations)


class Shield:
    def __init__(self, policy: Policy, lookahead_s: float = 3.0, dt: float = 0.5):
        self.policy = policy
        self.lookahead_s = lookahead_s
        self.dt = dt
        # Pre-build shapely polygons once (grant: pre-compute at ingest,
        # never rebuild per tick).
        self._fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
        self._alts = policy.by_type(AltitudeEnvelope)
        self._kins = policy.by_type(KinematicEnvelope)

    # ---------------- violation checking ---------------- #

    def _predict(self, state: State, action: Action4D):
        """Constant-velocity forecast (the grant Shield's 50-pose forecast,
        shortened)."""
        t = 0.0
        while t <= self.lookahead_s + 1e-9:
            yield t, State(
                x=state.x + action.vx * t,
                y=state.y + action.vy * t,
                up=state.up + action.vz_up * t,
                yaw_deg=state.yaw_deg + action.yaw_rate * t,
            )
            t += self.dt

    def _check(self, state: State, action: Action4D) -> list[Violation]:
        found: list[Violation] = []

        # -- kinematic: property of the action itself
        for k in self._kins:
            h_speed = (action.vx ** 2 + action.vy ** 2) ** 0.5
            if h_speed > k.speed_max_mps + 1e-9:
                found.append(Violation(
                    rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                    detail=f"h-speed {h_speed:.2f} > max {k.speed_max_mps}"))
            if abs(action.vz_up) > k.climb_rate_max_mps + 1e-9:
                found.append(Violation(
                    rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                    detail=f"|vz| {abs(action.vz_up):.2f} > max {k.climb_rate_max_mps}"))
            if abs(action.yaw_rate) > k.yaw_rate_max_dps + 1e-9:
                found.append(Violation(
                    rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                    detail=f"|yaw_rate| {abs(action.yaw_rate):.1f} > max {k.yaw_rate_max_dps}"))

        # -- altitude: trend-aware. Outside the band but moving back in = OK.
        for env in self._alts:
            for t, p in self._predict(state, action):
                below = p.up < env.alt_min_m - 1e-9
                above = p.up > env.alt_max_m + 1e-9
                if below and action.vz_up <= 1e-9:
                    found.append(Violation(
                        rule_id=env.id, category="altitude", predicted_at_s=t,
                        detail=f"alt {p.up:.1f}m < floor {env.alt_min_m}m, not climbing"))
                    break
                if above and action.vz_up >= -1e-9:
                    found.append(Violation(
                        rule_id=env.id, category="altitude", predicted_at_s=t,
                        detail=f"alt {p.up:.1f}m > ceiling {env.alt_max_m}m, not descending"))
                    break

        # -- geofence: trend-aware for the "already inside" case.
        for f, poly in self._fences:
            if point_in_fence(state.x, state.y, state.up, f, poly):
                ox, oy = push_out_direction(state.x, state.y, poly)
                escaping = (action.vx * ox + action.vy * oy) > 0.1
                if not escaping:
                    found.append(Violation(
                        rule_id=f.id, category="geofence", predicted_at_s=0.0,
                        detail="currently INSIDE zone and not escaping"))
                continue   # while inside, predictive entry checks are moot
            for t, p in self._predict(state, action):
                if point_in_fence(p.x, p.y, p.up, f, poly):
                    found.append(Violation(
                        rule_id=f.id, category="geofence", predicted_at_s=t,
                        detail=f"predicted pos ({p.x:.1f},{p.y:.1f}) inside NFZ at t+{t:.1f}s"))
                    break

        # dedupe by (rule, category), keep earliest
        seen: dict[tuple, Violation] = {}
        for v in found:
            key = (v.rule_id, v.category)
            if key not in seen or v.predicted_at_s < seen[key].predicted_at_s:
                seen[key] = v
        return list(seen.values())

    # ---------------- repair operators ---------------- #

    def _repair_kinematic(self, a: Action4D, repairs: list[Repair]) -> Action4D:
        vx, vy, vz, yr = a.vx, a.vy, a.vz_up, a.yaw_rate
        for k in self._kins:
            h = (vx ** 2 + vy ** 2) ** 0.5
            if h > k.speed_max_mps:
                s = k.speed_max_mps / h
                vx, vy = vx * s, vy * s
                repairs.append(Repair(operator="SpeedClamp",
                                      detail=f"h-speed {h:.2f} -> {k.speed_max_mps}"))
            if abs(vz) > k.climb_rate_max_mps:
                new = k.climb_rate_max_mps * (1 if vz > 0 else -1)
                repairs.append(Repair(operator="ClimbClamp", detail=f"vz {vz:.2f} -> {new:.2f}"))
                vz = new
            if abs(yr) > k.yaw_rate_max_dps:
                new = k.yaw_rate_max_dps * (1 if yr > 0 else -1)
                repairs.append(Repair(operator="YawClamp", detail=f"yaw {yr:.1f} -> {new:.1f}"))
                yr = new
        return Action4D(vx=vx, vy=vy, vz_up=vz, yaw_rate=yr)

    def _repair_altitude(self, state: State, a: Action4D, repairs: list[Repair]) -> Action4D:
        """Project vz so the lookahead endpoint lands inside the band.
        Handles both overshoot (flying out of the band) and recovery
        (already outside: climb/descend back at a sane rate)."""
        vz = a.vz_up
        for env in self._alts:
            end_up = state.up + vz * self.lookahead_s
            if end_up > env.alt_max_m:
                new = (env.alt_max_m - state.up) / self.lookahead_s
                repairs.append(Repair(operator="AltitudeFix",
                                      detail=f"vz {vz:.2f} -> {new:.2f} (ceiling {env.alt_max_m}m)"))
                vz = new
            elif end_up < env.alt_min_m:
                new = (env.alt_min_m - state.up) / self.lookahead_s
                repairs.append(Repair(operator="AltitudeFix",
                                      detail=f"vz {vz:.2f} -> {new:.2f} (floor {env.alt_min_m}m)"))
                vz = new
        return Action4D(vx=a.vx, vy=a.vy, vz_up=vz, yaw_rate=a.yaw_rate)

    def _repair_geofence(self, state: State, a: Action4D, repairs: list[Repair]) -> Action4D:
        """Two modes:
        - INSIDE a zone  -> GeofenceEscape: fly straight out (recovery).
        - heading INTO a zone -> GeofenceSlide: cancel the into-zone velocity
          component, keep/add a tangent one so the mission keeps moving along
          the edge instead of stalling (anti-stall bias)."""
        vx, vy = a.vx, a.vy
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)

        for f, poly in self._fences:
            if point_in_fence(state.x, state.y, state.up, f, poly):
                ox, oy = push_out_direction(state.x, state.y, poly)
                spd = min(2.0, cap)
                vx, vy = ox * spd, oy * spd
                repairs.append(Repair(operator="GeofenceEscape",
                                      detail=f"{f.id}: inside -> exit at {spd:.1f} m/s"))
                continue

            probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
            hit = any(point_in_fence(p.x, p.y, p.up, f, poly)
                      for _, p in self._predict(state, probe))
            if not hit:
                continue

            ox, oy = push_out_direction(state.x, state.y, poly)   # unit "away from zone"
            into = -(vx * ox + vy * oy)                           # speed INTO the zone
            if into > 0:
                vx += ox * into                                   # cancel it
                vy += oy * into
                # anti-stall: if nearly nothing is left (head-on approach),
                # push along the zone edge instead of stopping dead.
                h = (vx ** 2 + vy ** 2) ** 0.5
                if h < 0.5:
                    tx, ty = -oy, ox                              # tangent to the edge
                    if a.vx * tx + a.vy * ty < 0:                 # keep the raw action's turn side
                        tx, ty = -tx, -ty
                    spd = min(max(into, 1.0), cap)
                    vx, vy = tx * spd, ty * spd
                repairs.append(Repair(operator="GeofenceSlide",
                                      detail=f"{f.id}: removed {into:.2f} m/s into-zone component"))
        return Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)

    # ---------------- mid-flight policy update ---------------- #

    def hot_apply(self, fence: PolygonFence) -> None:
        """Inject a dynamic NFZ mid-flight (grant: dynamic_nfz hot-apply).
        Bumps the policy generation so every artefact after this instant
        carries a different policy_hash — the audit trail shows exactly
        which rules were active when."""
        self.policy.constraints.append(fence)
        self.policy.generation += 1
        self._fences.append((fence, fence_polygon(fence)))

    # ---------------- the public entry point ---------------- #

    def filter(self, state: State, raw: Action4D) -> ShieldDecision:
        violations = self._check(state, raw)
        if not violations:
            return ShieldDecision(raw=raw, emitted=raw)   # untouched passthrough

        repairs: list[Repair] = []
        fixed = self._repair_kinematic(raw, repairs)
        fixed = self._repair_altitude(state, fixed, repairs)
        fixed = self._repair_geofence(state, fixed, repairs)

        # P0 escape guard: repaired action must re-check clean. If not -> BRAKE.
        if self._check(state, fixed):
            repairs.append(Repair(operator="Brake", detail="repair not converged -> stop"))
            return ShieldDecision(raw=raw, emitted=BRAKE, violations=violations,
                                  repairs=repairs, braked=True)

        return ShieldDecision(raw=raw, emitted=fixed, violations=violations,
                              repairs=repairs)
