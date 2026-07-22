"""Repair operators + the bounded repair stack (safety-shield.md).

Operators are tried in priority order; the first that yields a feasible action
with magnitude <= threshold wins. A bounded loop (default 3 iterations) re-checks
after each applied repair and guards against operators interacting destructively
under simultaneous violations. If nothing converges within the magnitude cap, the
stack reports failure and the caller escalates to the fail-safe FSM.

Phase-1 slice ships two operators: ``AltitudeClamp`` (cheap, envelope-only) and
``LateralProjection`` (the operator the mid-term demo fires). ``PathRepair`` is
Phase 2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal, Protocol

from policy_dsl import PolicyIR
from vlaguard_common import Action4D

from safety_shield.checker import Violation, check
from safety_shield.kinematics import VehicleState, body_to_enu, enu_to_body, predict

RepairResult = Literal["ok", "skip", "fail"]


@dataclass(frozen=True)
class RepairAttempt:
    operator: str
    result: RepairResult
    reason: str = ""
    magnitude_m: float = 0.0
    iterations: int = 0


@dataclass
class RepairConfig:
    max_iterations: int = 3
    lateral_threshold_m: float = 2.0  # theta (lateral) from the FSM defaults
    vertical_threshold_m: float = 0.5  # theta (vertical)
    outward_margin_m: float = 1.0
    escape_speed_mps: float = 2.0  # recovery speed for GeofenceEscape


@dataclass
class RepairOutcome:
    action: Action4D
    attempts: list[RepairAttempt] = field(default_factory=list)
    converged: bool = False


class RepairOperator(Protocol):
    name: str

    def repair(
        self,
        action: Action4D,
        violation: Violation,
        ir: PolicyIR,
        state: VehicleState,
        cfg: RepairConfig,
    ) -> tuple[Action4D, RepairAttempt]: ...


class AltitudeClamp:
    """Clamp ``vz`` so the predicted altitude stays inside the envelope band.

    ``magnitude_m`` is the altitude error being corrected (a *position* metric, in
    metres) — comparable to the vertical threshold theta in :class:`RepairConfig`.
    """

    name = "AltitudeClamp"

    def repair(self, action, violation, ir, state, cfg):  # type: ignore[no-untyped-def]
        if violation.category != "envelope":
            return action, RepairAttempt(self.name, "skip", "not an altitude violation")
        env = next((e for e in ir.envelopes if e.id == violation.rule_id), None)
        if env is None:
            return action, RepairAttempt(self.name, "skip", "rule not an envelope")
        t = max(violation.first_hit_s, 1e-3)
        if violation.hit_alt_m > env.altitude_max_m:
            error = violation.hit_alt_m - env.altitude_max_m
            vz_max = (env.altitude_max_m - state.alt_agl_m) / t
            vz = min(action.vz, vz_max)
        else:
            error = env.altitude_min_m - violation.hit_alt_m
            vz_min = (env.altitude_min_m - state.alt_agl_m) / t
            vz = max(action.vz, vz_min)
        new = Action4D(vx=action.vx, vy=action.vy, vz=vz, yaw_rate=action.yaw_rate)
        return new, RepairAttempt(self.name, "ok", magnitude_m=abs(error))


class LateralProjection:
    """Project the predicted trajectory off the fence and steer along the boundary.

    Finds the deepest predicted penetration into the polygon, takes the outward
    normal there, removes the inward component of the horizontal velocity, and
    adds an outward bias sized to clear the penetration within the horizon (while
    preserving the original ground speed). ``magnitude_m`` is the penetration
    depth (metres) — the *position* correction — comparable to the lateral
    threshold theta. A deep head-on dive therefore reports a large magnitude and
    is escalated rather than projected; a shallow clip is projected.
    """

    name = "LateralProjection"

    def repair(self, action, violation, ir, state, cfg):  # type: ignore[no-untyped-def]
        if violation.category != "geometric":
            return action, RepairAttempt(self.name, "skip", "not a geometric violation")
        rec = next((p for p in ir.polygons if p.id == violation.rule_id), None)
        if rec is None:
            return action, RepairAttempt(self.name, "skip", "rule not a polygon")

        # LateralProjection owns *approaching* trajectories. A vehicle that is
        # already inside the polygon is handed to GeofenceEscape instead — this
        # operator's depth-based magnitude would always exceed the cap there and
        # trigger an escalation/brake that locks the violation in place.
        if ir.signed_distance(rec, state.lat, state.lon) < 0:
            return action, RepairAttempt(
                self.name, "skip", "vehicle inside; recovery operator owns this"
            )

        # deepest predicted penetration into this polygon over the horizon
        origin_xy = ir.projection.to_xy(state.lat, state.lon)
        depth, deep_lat, deep_lon = 0.0, violation.hit_lat, violation.hit_lon
        for pose in predict(state, action, origin_xy):
            lat, lon = ir.projection.to_latlon(pose.east_m, pose.north_m)
            sd = ir.signed_distance(rec, lat, lon)
            if sd < 0 and -sd > depth:
                depth, deep_lat, deep_lon = -sd, lat, lon

        dx, dy = ir.projection.to_xy(deep_lat, deep_lon)
        ext_lat, ext_lon = ir.nearest_exterior_latlon(
            rec, deep_lat, deep_lon, margin_m=cfg.outward_margin_m
        )
        ex, ey = ir.projection.to_xy(ext_lat, ext_lon)
        ox, oy = ex - dx, ey - dy
        norm = math.hypot(ox, oy)
        if norm < 1e-6:
            return action, RepairAttempt(self.name, "fail", "degenerate outward normal")
        ox, oy = ox / norm, oy / norm  # unit outward normal

        ve, vn, vu = body_to_enu(action, state.yaw_rad)
        speed = math.hypot(ve, vn)
        inward = -(ve * ox + vn * oy)
        if inward > 0:  # slide: remove the component heading into the fence
            ve += inward * ox
            vn += inward * oy
        # outward bias sized to clear the penetration (+margin) within the horizon
        clear_speed = (depth + cfg.outward_margin_m) / 5.0
        ve += clear_speed * ox
        vn += clear_speed * oy
        # preserve original ground speed so the manoeuvre stays in-envelope
        new_norm = math.hypot(ve, vn)
        if new_norm > 1e-6 and speed > 1e-6:
            ve, vn = ve / new_norm * speed, vn / new_norm * speed

        new_body = enu_to_body(ve, vn, vu, state.yaw_rad)
        new = Action4D(vx=new_body.vx, vy=new_body.vy, vz=action.vz, yaw_rate=action.yaw_rate)
        return new, RepairAttempt(self.name, "ok", magnitude_m=depth)


class GeofenceEscape:
    """Drive a vehicle already *inside* a polygon straight out to the nearest exit.

    Where ``LateralProjection`` slides an *approaching* trajectory off the fence
    (a correction sized under the lateral threshold), this operator handles the
    complementary case the threshold was never meant to cover: a vehicle that is
    already deep inside a zone. Its penetration depth is large by construction,
    so the magnitude cap that protects ``LateralProjection`` from over-correcting
    would reject it and the Shield would brake — and braking while inside locks
    the violation forever. We therefore flag this as a *recovery* operator
    (``recovery = True``): the repair loop exempts recovery operators from the
    magnitude cap, because their large magnitude is the point.

    The emitted action flies toward the nearest boundary point at a moderate,
    envelope-safe speed (capped against the kinematic budget, defaulting low).
    ``magnitude_m`` is the current penetration depth (the position error being
    recovered), reported for the audit log even though it is not capped.
    """

    name = "GeofenceEscape"
    recovery = True
    _escape_speed_mps = 2.0

    def repair(self, action, violation, ir, state, cfg):  # type: ignore[no-untyped-def]
        if violation.category != "geometric":
            return action, RepairAttempt(self.name, "skip", "not a geometric violation")
        rec = next((p for p in ir.polygons if p.id == violation.rule_id), None)
        if rec is None:
            return action, RepairAttempt(self.name, "skip", "rule not a polygon")
        # only applies when the vehicle is actually inside; otherwise LateralProjection owns it
        if ir.signed_distance(rec, state.lat, state.lon) >= 0:
            return action, RepairAttempt(self.name, "skip", "vehicle not inside the polygon")

        ox, oy = ir.outward_normal_enu(rec, state.lat, state.lon)  # toward nearest exit
        speed = min(self._escape_speed_mps, cfg.escape_speed_mps)
        ve, vn = ox * speed, oy * speed
        new_body = enu_to_body(ve, vn, action.vz, state.yaw_rad)
        depth = abs(ir.signed_distance(rec, state.lat, state.lon))
        new = Action4D(vx=new_body.vx, vy=new_body.vy, vz=action.vz, yaw_rate=action.yaw_rate)
        return new, RepairAttempt(self.name, "ok", magnitude_m=depth)


# Priority order — cheapest / most-local first; recovery last among geometric
# handlers so projection still gets first dibs on approaching trajectories.
DEFAULT_STACK: list[RepairOperator] = [AltitudeClamp(), LateralProjection(), GeofenceEscape()]


def repair_action(
    ir: PolicyIR,
    state: VehicleState,
    action: Action4D,
    violations: list[Violation],
    cfg: RepairConfig | None = None,
    stack: list[RepairOperator] | None = None,
) -> RepairOutcome:
    """Run the bounded repair loop. Returns the repaired action + per-attempt log.

    ``converged`` is True only if, after the loop, the predicted trajectory has no
    violations AND every applied repair stayed within the magnitude thresholds.
    """
    cfg = cfg or RepairConfig()
    stack = stack or DEFAULT_STACK
    current = action
    attempts: list[RepairAttempt] = []
    active = violations

    for it in range(1, cfg.max_iterations + 1):
        if not active:
            return RepairOutcome(current, attempts, converged=True)
        target = active[0]  # earliest-hit violation
        applied = False
        for op in stack:
            new, att = op.repair(current, target, ir, state, cfg)
            att = RepairAttempt(att.operator, att.result, att.reason, att.magnitude_m, it)
            if att.result == "ok":
                # Recovery operators (e.g. GeofenceEscape for an inside vehicle)
                # are exempt from the magnitude cap: their large magnitude is the
                # penetration depth being recovered, which projection thresholds
                # were never meant to govern. Capping them would re-introduce the
                # "brake while inside = deadlock" failure.
                is_recovery = getattr(op, "recovery", False)
                if not is_recovery:
                    cap = (
                        cfg.vertical_threshold_m
                        if target.category == "envelope"
                        else cfg.lateral_threshold_m
                    )
                    if att.magnitude_m > cap:
                        attempts.append(
                            RepairAttempt(
                                att.operator, "fail", "magnitude over cap", att.magnitude_m, it
                            )
                        )
                        return RepairOutcome(current, attempts, converged=False)
                attempts.append(att)
                current = new
                applied = True
                break
            attempts.append(att)
        if not applied:
            return RepairOutcome(current, attempts, converged=False)
        active = check(ir, state, current)

    # exhausted iterations
    return RepairOutcome(current, attempts, converged=not check(ir, state, current))
