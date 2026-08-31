"""
Safety Shield (mini) — validate a 4-D action against the policy, repair if
possible, brake only as last resort.

Flow per tick (mirrors the grant's Safety Shield node, scaled down):

    check (trend-aware)                "does this action lead somewhere illegal?"
        -> repair operators             kinematic clamp -> altitude fix, then
                                        obstacle clearance <-> geofence
                                        escape/slide ITERATED to a fixed point
        -> re-check the repaired action P0 escape guard
        -> still illegal? recovery      a heading that re-checks clean...
        -> nothing left? BRAKE          ...and a stop only where a stop is legal

Design rule learned the hard way: when the vehicle is ALREADY in violation
(below the altitude floor, inside a zone, inside the clearance ring), the right
output is a RECOVERY action, not a freeze — braking would lock the violation in
place forever. So checks are trend-aware: "in violation but actively correcting"
passes, and BRAKE is itself re-checked before it is ever emitted (a standstill
inside the clearance ring is not a fail-safe, it IS the deadlock).

The repair operators are COUPLED, so the chain is a fixed-point iteration, not a
pipeline: the geofence anti-stall tangent can leave the clearance ring violated,
and the clearance push can aim into a fence. Running each once leaves whatever
the LAST operator produced un-repaired — which the P0 guard then brakes on.

Guarantee: the action this module RETURNS never makes things worse — it is
either predicted-clean, actively recovering, or a full stop.
"""
from __future__ import annotations

import math

import numpy as np
from pydantic import BaseModel

from .geometry import fence_polygon, point_in_fence, push_out_direction
from .models import (
    Action4D,
    AltitudeEnvelope,
    KinematicEnvelope,
    ObstacleClearance,
    Policy,
    PolygonFence,
    State,
    SubjectStandoff,
)

BRAKE = Action4D(vx=0.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)


def _sanitise(a: Action4D) -> tuple[Action4D, list[str]]:
    """Force every channel finite, returning the names of the ones that were not.

    NaN fails EVERY comparison, so an action carrying one sails through _check()
    with zero violations and out through the untouched-passthrough branch of
    filter() — the guardrail fails OPEN, which is the one direction it must never
    fail. Infinity is no better: the speed clamp scales it by cap/hypot, and
    inf * 0.0 is NaN, so a bounded repair operator manufactures the poison.

    This is reachable from a real pilot, not just from a fuzzer. servo() sizes
    forward speed from the detector's box width, so a zero-width box is one
    division away from NaN, and a detector that fails mid-flight is a normal
    event rather than an exotic one.

    Fail-safe reading: a non-finite command is not a command. The channel goes to
    zero, and the caller sees a violation, so it is never silent.
    """
    bad = [n for n, v in (("vx", a.vx), ("vy", a.vy),
                          ("vz_up", a.vz_up), ("yaw_rate", a.yaw_rate))
           if not math.isfinite(v)]
    if not bad:
        return a, bad
    return Action4D(
        vx=a.vx if math.isfinite(a.vx) else 0.0,
        vy=a.vy if math.isfinite(a.vy) else 0.0,
        vz_up=a.vz_up if math.isfinite(a.vz_up) else 0.0,
        yaw_rate=a.yaw_rate if math.isfinite(a.yaw_rate) else 0.0,
    ), bad

# Chamfer(1, sqrt2) OVER-estimates true Euclidean distance by at most 8.24%
# (worst case at atan(sqrt2-1) = 22.5 deg). Over-estimating clearance is the
# UNSAFE direction — it would report "further from the wall than we are" — so
# every distance is scaled by 1/1.0824. The field is therefore a conservative
# lower bound on the true distance, never an optimistic one.
_CHAMFER_CORR = 1.0 / 1.08239

# Floor on the clearance speed taper: the repair SLOWS, it never stops.
# Stopping is the brake's job (and the brake is a deliberate, audited event).
_CLEAR_MIN_SCALE = 0.25

# Nominal horizontal deceleration (m/s^2) used to SIZE the clearance forecast.
# The clearance question is "can I still stop before the ring?", NOT "where does
# a 3 s straight line end up". Scanning the full geofence horizon at constant
# velocity turns min_clearance_m into an effective standoff of
# min_clearance_m + v * lookahead_s (5 m -> 17 m at 4 m/s), so the Shield ends up
# vetoing every planned route whose corridor is narrower than that — including
# routes its own planner produced.
_CLEAR_DECEL_MPS2 = 3.0

# How often the repair chain is re-run before giving up. The operators are
# COUPLED (the geofence anti-stall tangent can break clearance; the clearance
# push can aim into a fence), so the chain is iterated to a fixed point instead
# of being treated as a one-way pipeline whose last stage is never re-repaired.
_REPAIR_PASSES = 3


def _dedupe(repairs: list["Repair"]) -> list["Repair"]:
    """One audit line per distinct fix — the repair chain is iterated, so the
    same operator can legitimately log the same detail more than once."""
    seen, out = set(), []
    for r in repairs:
        key = (r.operator, r.detail)
        if key not in seen:
            seen.add(key)
            out.append(r)
    return out


def _build_distance_field(occ, res: float):
    """Metres from each free cell to the nearest BLOCKED cell centre.

    Pure-numpy two-pass chamfer distance transform (no scipy). Each pass walks
    the rows once, propagating from the previous row (orthogonal cost `res`,
    diagonal cost `res*sqrt(2)`) and then running the in-row propagation as a
    vectorised min-scan, using the identity

        min_{k<=j} d[k] + a*(j-k)  ==  cummin(d - a*j) + a*j

    so the whole thing is O(N) numpy calls instead of O(N^2) python.

    Caveat by construction: distance is to the nearest occupied cell CENTRE,
    so a building face sits up to res/2 closer than the number says. Choose
    min_clearance_m with that quantisation in mind (the maps are res = 2 m).
    """
    occ = np.asarray(occ)
    if occ.ndim != 2:
        raise ValueError(f"obstacle_map['occ'] must be 2-D, got shape {occ.shape}")
    n, m = occ.shape
    a = float(res)                 # orthogonal step cost
    b = a * math.sqrt(2.0)         # diagonal step cost
    big = (n + m + 2) * a          # finite "unreachable" sentinel (no inf math)

    d = np.where(occ > 0, 0.0, big).astype(np.float64)
    idx = np.arange(m, dtype=np.float64)
    a_idx = a * idx

    def _hscan(row):
        """In-row propagation, both directions (vectorised min-scan)."""
        row = np.minimum(row, np.minimum.accumulate(row - a_idx) + a_idx)
        rev = (row + a_idx)[::-1]
        return np.minimum(row, np.minimum.accumulate(rev)[::-1] - a_idx)

    def _diag(prev):
        """min(prev[j-1], prev[j+1]) with `big` outside the grid."""
        left = np.empty_like(prev)
        left[0] = big
        left[1:] = prev[:-1]
        right = np.empty_like(prev)
        right[-1] = big
        right[:-1] = prev[1:]
        return np.minimum(left, right)

    # forward pass: rows top -> bottom
    d[0] = _hscan(d[0])
    for i in range(1, n):
        prev = d[i - 1]
        d[i] = np.minimum(d[i], np.minimum(prev + a, _diag(prev) + b))
        d[i] = _hscan(d[i])
    # backward pass: rows bottom -> top
    for i in range(n - 2, -1, -1):
        nxt = d[i + 1]
        d[i] = np.minimum(d[i], np.minimum(nxt + a, _diag(nxt) + b))
        d[i] = _hscan(d[i])

    return d * _CHAMFER_CORR


def _build_signed_distance_field(occ, res: float):
    """SIGNED distance: positive metres to the nearest obstacle out in free
    space, NEGATIVE metres to the nearest free cell when the point is INSIDE an
    obstacle footprint.

    An unsigned field is exactly 0 across a whole building footprint, so its
    gradient there is flat and "the way out" is undefined. That is not a corner
    case: the 2-D grid treats buildings as infinitely tall columns, so a drone
    flying OVER a building (the demo's ESCAPE mode climbs on purpose) sits on
    those dead cells at any altitude, gets "no direction" back, and freezes —
    the exact deadlock the module docstring forbids. Signing the field gives
    every point inside a building a well-defined shortest way out, so the repair
    always has something to steer by.

    The inside half is deliberately NOT chamfer-corrected: over-estimating how
    deep inside we are is the conservative direction.
    """
    occ = (np.asarray(occ) > 0).astype(np.uint8)
    outside = _build_distance_field(occ, res)
    inside = _build_distance_field(1 - occ, res) / _CHAMFER_CORR
    return outside - inside


class Violation(BaseModel):
    rule_id: str
    category: str            # "kinematic" | "altitude" | "geofence" | "clearance"
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
    # What is STILL wrong with the action that was actually flown.
    #
    # `violations` is the check on `raw`, so on its own it cannot answer the
    # grant's hard KPI - "a P0 that was seen and then flown anyway". The escape
    # rate was inferred instead, from whether the Shield had done *something*
    # (see guardrail/kpi.py), and since every branch that produces a violation
    # also appends a Repair, that inference could not return a non-zero number
    # for any log this Shield can generate.
    #
    # Recording the re-check makes the KPI a measured fact rather than an
    # inference, and makes it auditable offline from the artefact alone. Empty
    # is the good case and the overwhelmingly common one: re-checked against
    # every delivered flight, the emitted action violated a P0 rule on zero
    # ticks.
    emitted_violations: list[Violation] = []

    @property
    def touched(self) -> bool:
        return bool(self.violations)

    @property
    def escaped(self) -> bool:
        """A P0 rule was violated by the action that was flown."""
        return bool(self.emitted_violations)


class Shield:
    def __init__(self, policy: Policy, lookahead_s: float = 3.0, dt: float = 0.5,
                 obstacle_map: dict | None = None):
        """obstacle_map: None, or {"occ": uint8 NxN (1 = blocked), "res": float,
        "ox": float, "oy": float} — the same occupancy grid the global planner
        uses. A policy YAML cannot carry an 80x80 grid, so obstacle_clearance
        rules get their world here. With obstacle_map=None those rules are
        inert and the Shield behaves exactly as before."""
        self.policy = policy
        self.lookahead_s = lookahead_s
        self.dt = dt
        # Pre-build shapely polygons once (grant: pre-compute at ingest,
        # never rebuild per tick).
        self._fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
        self._alts = policy.by_type(AltitudeEnvelope)
        self._kins = policy.by_type(KinematicEnvelope)
        self._clear = policy.by_type(ObstacleClearance)
        self._standoffs = policy.by_type(SubjectStandoff)
        # Where the thing we are following is, in world coordinates, plus what
        # kind of thing it is. Supplied by the perception stack once per tick via
        # set_subject(); None means "not currently tracking anything", and every
        # SubjectStandoff rule is inert until it is set.
        self._subject: tuple[float, float] | None = None
        self._subject_class: str | None = None

        # Distance field: built ONCE here, never per tick (same rule as the
        # fence polygons). Skipped entirely when nothing needs it.
        self._dist = None
        self._res = self._ox = self._oy = 0.0
        if obstacle_map is not None and self._clear:
            self._res = float(obstacle_map["res"])
            self._ox = float(obstacle_map["ox"])
            self._oy = float(obstacle_map["oy"])
            self._dist = _build_signed_distance_field(obstacle_map["occ"], self._res)

    # ---------------- obstacle distance field ---------------- #

    def _distance_at(self, x: float, y: float) -> float:
        """Approx. SIGNED metres from world point (x, y) to the nearest mapped
        obstacle — negative inside a building footprint. inf when there is no
        map. Bilinear inside the grid; outside it, a safe lower bound built from
        the distance to the map edge (every obstacle lives inside the map, so
        being far off-map IS clear)."""
        d = self._dist
        if d is None:
            return float("inf")
        n, m = d.shape
        fi = (x - self._ox) / self._res
        fj = (y - self._oy) / self._res
        ci = min(max(fi, 0.0), n - 1.0)
        cj = min(max(fj, 0.0), m - 1.0)
        off = math.hypot((fi - ci) * self._res, (fj - cj) * self._res)

        i0 = int(math.floor(ci)); i1 = min(i0 + 1, n - 1); ti = ci - i0
        j0 = int(math.floor(cj)); j1 = min(j0 + 1, m - 1); tj = cj - j0
        v = (float(d[i0, j0]) * (1 - ti) * (1 - tj)
             + float(d[i1, j0]) * ti * (1 - tj)
             + float(d[i0, j1]) * (1 - ti) * tj
             + float(d[i1, j1]) * ti * tj)
        if off > 0.0:
            # |d(q) - d(edge)| <= off (1-Lipschitz), and d(q) >= off because
            # every obstacle is inside the map -> conservative bound. The second
            # half only holds where the clamped edge value is POSITIVE; a signed
            # field can be negative if the map border itself is built on.
            lo = v - off
            return max(lo, off) if v > 0.0 else lo
        return v

    def _away_dir(self, x: float, y: float, probe_r: float) -> tuple[float, float]:
        """Unit vector along the local gradient of the distance field, i.e.
        'the way AWAY from the nearest building'. (0, 0) when even the probe
        finds nothing to steer by — the caller then leaves the action alone and
        the P0 re-check decides."""
        if self._dist is None:
            return 0.0, 0.0
        h = max(self._res * 0.5, 1e-3)
        gx = (self._distance_at(x + h, y) - self._distance_at(x - h, y)) / (2 * h)
        gy = (self._distance_at(x, y + h) - self._distance_at(x, y - h)) / (2 * h)
        norm = math.hypot(gx, gy)
        if norm > 1e-6:
            return gx / norm, gy / norm
        # Flat field (a symmetric ridge between two buildings, the exact centre
        # of a footprint): probe 8 compass directions and take the one that
        # gains the most distance, WIDENING the radius until something gains.
        # "No direction" must never fall through to a brake — a stationary
        # vehicle never recedes, so the violation would repeat forever.
        here = self._distance_at(x, y)
        r0 = max(probe_r, self._res)
        for mult in (1.0, 2.0, 4.0):
            r = r0 * mult
            best, best_d = (0.0, 0.0), here
            for k in range(8):
                ang = k * math.pi / 4.0
                ux, uy = math.cos(ang), math.sin(ang)
                dv = self._distance_at(x + ux * r, y + uy * r)
                if dv > best_d + 1e-9:
                    best, best_d = (ux, uy), dv
            if best != (0.0, 0.0):
                return best
        return 0.0, 0.0

    # ---------------- violation checking ---------------- #

    def _predict(self, state: State, action: Action4D,
                 horizon_s: float | None = None):
        """Constant-velocity forecast (the grant Shield's 50-pose forecast,
        shortened). `horizon_s` overrides the default lookahead — the clearance
        rule uses a much shorter, braking-distance-sized one (see
        `_clear_horizon_s`). The final sample always lands exactly ON the
        horizon, so a horizon that is not a whole number of `dt` still gets its
        endpoint checked."""
        horizon = self.lookahead_s if horizon_s is None else horizon_s
        t = 0.0
        while True:
            yield t, State(
                x=state.x + action.vx * t,
                y=state.y + action.vy * t,
                up=state.up + action.vz_up * t,
                yaw_deg=state.yaw_deg + action.yaw_rate * t,
            )
            if t >= horizon - 1e-9:
                return
            t = min(t + self.dt, horizon)

    def _clear_horizon_s(self, action: Action4D) -> float:
        """How far ahead the CLEARANCE rule looks: one control step of reaction
        plus the distance still needed to stop, expressed as a time at the
        current speed (which is what `_predict` advances at).

        Using the full geofence lookahead here is what made the rule unflyable:
        a constant-velocity 3 s forecast at 4 m/s puts the effective standoff at
        min_clearance_m + 12 m, so any planned corridor narrower than that trips
        the rule every time the path curves — even though the vehicle would
        follow the curve and never get close."""
        v = math.hypot(action.vx, action.vy)
        if v <= 1e-9:
            return 0.0
        return min(self.lookahead_s, self.dt + v / (2.0 * _CLEAR_DECEL_MPS2))

    def _clear_min_dist(self, state: State, a: Action4D) -> float:
        """Smallest mapped-obstacle distance `a`'s FORECAST reaches. This is the
        score every clearance repair is judged by: a repair must raise it, never
        lower it.

        The t = 0 sample is excluded on purpose — every candidate action shares
        the present position, so scoring it in makes the number blind exactly
        when the vehicle is already inside the ring and the choice matters most.
        (A standstill has no forecast, so it scores its own position.)"""
        if self._dist is None:
            return float("inf")
        steps = list(self._predict(state, a, self._clear_horizon_s(a)))
        return min(self._distance_at(p.x, p.y) for _, p in (steps[1:] or steps))

    def _clear_dir_candidates(self, a: Action4D, cap: float):
        """Recovery headings: 12 bearings x 2 speeds, inheriting `a`'s vertical
        and yaw components (those were already repaired by the altitude operator,
        so they must not be re-invented here) — re-clamped so a candidate can
        never be rejected for a kinematic reason it inherited."""
        vz, yr = a.vz_up, a.yaw_rate
        for k in self._kins:
            vz = max(-k.climb_rate_max_mps, min(k.climb_rate_max_mps, vz))
            # UNITS. The policy states the cap in DEGREES per second; the
            # Action4D contract carries yaw_rate in RADIANS per second.
            # These were compared raw, so a 45 dps cap sat at 45 rad/s
            # (2578 dps) and this P1 rule could never fire on any real
            # action. Convert at the boundary, here and at the two other
            # sites below.
            ymax = math.radians(k.yaw_rate_max_dps)
            yr = max(-ymax, min(ymax, yr))
        for k in range(12):
            ang = k * math.pi / 6.0
            ux, uy = math.cos(ang), math.sin(ang)
            for frac in (1.0, 0.5):
                spd = cap * frac
                yield Action4D(vx=ux * spd, vy=uy * spd, vz_up=vz, yaw_rate=yr)

    def _best_clear_dir(self, state: State, a: Action4D, cap: float):
        """The candidate heading whose clearance forecast keeps the most room,
        ties broken toward the commanded heading."""
        if self._dist is None:
            return None
        best, best_key = None, None
        for cand in self._clear_dir_candidates(a, cap):
            key = (self._clear_min_dist(state, cand),
                   cand.vx * a.vx + cand.vy * a.vy)
            if best_key is None or key > best_key:
                best, best_key = cand, key
        return best

    def _rescue(self, state: State, raw: Action4D, a: Action4D, cap: float):
        """Best available recovery when the repair chain did not converge.

        Returns (clean, best): `clean` re-checks with ZERO violations, `best`
        merely keeps the most clearance. Stopping is not an answer on its own —
        a stationary vehicle inside the clearance ring (or inside a fence) is
        still in violation on the next tick and every tick after it, so a brake
        there is a permanent freeze, not a fail-safe."""
        clean = clean_key = best = best_key = None
        for cand in self._clear_dir_candidates(a, cap):
            key = (self._clear_min_dist(state, cand) if self._dist is not None else 0.0,
                   cand.vx * raw.vx + cand.vy * raw.vy)
            if best_key is None or key > best_key:
                best, best_key = cand, key
            if not self._check(state, cand) and (clean_key is None or key > clean_key):
                clean, clean_key = cand, key
        return clean, best

    # ---------------- the monitor, one predicate per rule ---------------- #
    #
    # Split out of a single `_check` on 2026-08-26 so that the REPAIRS can ask
    # the same question the monitor asks, instead of each carrying its own
    # weaker copy of the trend test. Three repair operators had drifted into
    # judging position only, and would clobber an action that was already
    # escaping - measured at six times slower on the standoff rule.
    #
    # `_check` folds these and is unchanged in behaviour, which
    # tests/test_check_contract.py pins by hashing its answers over 28 800
    # seeded samples across every policy in policies/.

    def _check_kinematic(self, k, action: Action4D) -> list[Violation]:
        """A property of the action alone - no state, no forecast."""
        out = []
        h_speed = (action.vx ** 2 + action.vy ** 2) ** 0.5
        if h_speed > k.speed_max_mps + 1e-9:
            out.append(Violation(
                rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                detail=f"h-speed {h_speed:.2f} > max {k.speed_max_mps}"))
        if abs(action.vz_up) > k.climb_rate_max_mps + 1e-9:
            out.append(Violation(
                rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                detail=f"|vz| {abs(action.vz_up):.2f} > max {k.climb_rate_max_mps}"))
        yaw_dps = math.degrees(abs(action.yaw_rate))     # contract is rad/s
        if yaw_dps > k.yaw_rate_max_dps + 1e-9:
            out.append(Violation(
                rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                detail=f"|yaw_rate| {yaw_dps:.1f} dps > max {k.yaw_rate_max_dps}"))
        return out

    def _check_altitude(self, env, state: State, action: Action4D) -> list[Violation]:
        """Trend-aware. Outside the band but moving back in = OK."""
        for t, p in self._predict(state, action):
            below = p.up < env.alt_min_m - 1e-9
            above = p.up > env.alt_max_m + 1e-9
            if below and action.vz_up <= 1e-9:
                return [Violation(
                    rule_id=env.id, category="altitude", predicted_at_s=t,
                    detail=f"alt {p.up:.1f}m < floor {env.alt_min_m}m, not climbing")]
            if above and action.vz_up >= -1e-9:
                return [Violation(
                    rule_id=env.id, category="altitude", predicted_at_s=t,
                    detail=f"alt {p.up:.1f}m > ceiling {env.alt_max_m}m, not descending")]
        return []

    def _check_standoff(self, so, state: State, action: Action4D) -> list[Violation]:
        """Trend-aware, same shape as the fence rule.

        Inert with no subject set, which is the honest reading: a standoff rule
        with nothing to stand off from has no opinion. "Already too close but
        opening the range" passes, because the alternative is to raise a
        violation on the very action that is fixing it - and BRAKE inside the
        ring would freeze the aircraft at the distance it must not hold.
        """
        if self._subject is None or not so.binds(self._subject_class):
            return []
        sx, sy = self._subject
        d_now = math.hypot(state.x - sx, state.y - sy)
        what = self._subject_class or "subject"

        if d_now < so.min_range_m - 1e-9:
            # ALREADY inside. Judge the trend from the radial velocity, not from
            # predicted positions: _predict yields the current pose as its first
            # sample, where the range has not changed yet, so a comparison
            # against it can never see an opening move and the rule fires on the
            # very action that is recovering. Same shape as the geofence
            # "escaping" test.
            ux, uy = ((sx - state.x) / d_now, (sy - state.y) / d_now) \
                if d_now > 1e-6 else (0.0, 0.0)
            opening = -(action.vx * ux + action.vy * uy)
            if opening <= 0.1:
                return [Violation(
                    rule_id=so.id, category="standoff", predicted_at_s=0.0,
                    detail=(f"range {d_now:.1f}m to {what} < min "
                            f"{so.min_range_m}m and not opening"))]
            return []        # while inside, predictive checks are moot

        for t, p in self._predict(state, action):
            d = math.hypot(p.x - sx, p.y - sy)
            if d < so.min_range_m - 1e-9:
                return [Violation(
                    rule_id=so.id, category="standoff", predicted_at_s=t,
                    detail=(f"range closes to {d:.1f}m from {what} in "
                            f"{t:.1f}s, below min {so.min_range_m}m"))]
        return []

    def _check_fence(self, f, poly, state: State, action: Action4D) -> list[Violation]:
        """Trend-aware for the already-inside case."""
        if point_in_fence(state.x, state.y, state.up, f, poly):
            ox, oy = push_out_direction(state.x, state.y, poly)
            escaping = (action.vx * ox + action.vy * oy) > 0.1
            if not escaping:
                return [Violation(
                    rule_id=f.id, category="geofence", predicted_at_s=0.0,
                    detail="currently INSIDE zone and not escaping")]
            return []      # while inside, predictive entry checks are moot
        for t, p in self._predict(state, action):
            if point_in_fence(p.x, p.y, p.up, f, poly):
                return [Violation(
                    rule_id=f.id, category="geofence", predicted_at_s=t,
                    detail=f"predicted pos ({p.x:.1f},{p.y:.1f}) inside NFZ at t+{t:.1f}s")]
        return []

    def _clear_forecast(self, state: State, action: Action4D):
        """(steps, dists) over the clearance horizon.

        Shared so a repair that has already paid for the forecast does not pay
        again, once per rule per repair pass.
        """
        steps = list(self._predict(state, action, self._clear_horizon_s(action)))
        return steps, [self._distance_at(p.x, p.y) for _, p in steps]

    def _check_clearance(self, c, state: State, action: Action4D,
                         steps=None, dists=None) -> list[Violation]:
        """Trend-aware, same shape as the geofence rule."""
        if self._dist is None:
            return []
        if steps is None or dists is None:
            steps, dists = self._clear_forecast(state, action)
        d_now = dists[0]
        # "moving away" = distance grows over the FIRST forecast step
        receding = len(dists) > 1 and dists[1] > d_now + 1e-6
        # Being inside the ring is forgiven only while the forecast KEEPS
        # improving. The old code dropped the ENTIRE horizon on the strength of
        # one 0.5 s step, so a drone anywhere inside the ring got a free pass on
        # every obstacle on the map - including forecasts that ended up inside a
        # building.
        forgiving = d_now < c.min_clearance_m and receding
        prev = None
        for (t, p), d in zip(steps, dists):
            if d >= c.min_clearance_m:
                forgiving = False         # out of the ring: normal rules again
                prev = d
                continue
            if forgiving and (prev is None or d > prev + 1e-6):
                prev = d
                continue                  # still actively escaping -> never freeze
            return [Violation(
                rule_id=c.id, category="clearance", predicted_at_s=t,
                detail=(f"obstacle dist {d:.2f}m < min {c.min_clearance_m}m "
                        f"at ({p.x:.1f},{p.y:.1f}) t+{t:.1f}s"))]
        return []

    def _check(self, state: State, action: Action4D) -> list[Violation]:
        found: list[Violation] = []
        for k in self._kins:
            found += self._check_kinematic(k, action)
        for env in self._alts:
            found += self._check_altitude(env, state, action)
        if self._subject is not None:
            for so in self._standoffs:
                found += self._check_standoff(so, state, action)
        for f, poly in self._fences:
            found += self._check_fence(f, poly, state, action)
        if self._dist is not None and self._clear:
            # One forecast shared across every clearance rule, as before.
            steps, dists = self._clear_forecast(state, action)
            for c in self._clear:
                found += self._check_clearance(c, state, action, steps, dists)

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
            ymax = math.radians(k.yaw_rate_max_dps)
            if abs(yr) > ymax:
                new = ymax * (1 if yr > 0 else -1)
                repairs.append(Repair(
                    operator="YawClamp",
                    detail=f"yaw {math.degrees(yr):.1f} -> {math.degrees(new):.1f} dps"))
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

    def _repair_clearance(self, state: State, a: Action4D,
                          repairs: list[Repair]) -> Action4D:
        """Steer the HORIZONTAL velocity away from the building the FORECAST
        actually runs into.

        All three moves are driven by the local gradient of the distance field:
        cancel the velocity component pointing INTO the obstacle, taper what is
        left (the mission-progress part) by how deep the incursion is, then add
        an outward push sized to recover the shortfall within one lookahead.

        The gradient is read at the OFFENDING FORECAST POSE, not at the current
        one. When the nearest building right now is not the building the
        forecast hits, "outward" as measured here points straight AT the future
        obstacle: the into-component is negative so it is never cancelled, and
        the push then ACCELERATES the vehicle into the wall it is meant to
        avoid.

        The taper deliberately does NOT touch the push: deeper incursion means
        SLOWER progress but a FIRMER escape, never a limp one. soft_margin_m
        only widens the band the taper ramps over. It never zeroes the action —
        that is the brake's job."""
        if self._dist is None or not self._clear:
            return a
        vx, vy = a.vx, a.vy
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)

        for c in self._clear:
            hard_r = c.min_clearance_m
            soft_r = c.min_clearance_m + c.soft_margin_m
            probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
            steps, dists = self._clear_forecast(state, probe)

            # Ask the MONITOR, against the mutating vector, rather than
            # re-deriving a weaker condition here. The two had drifted: the
            # monitor forgives an action that keeps improving the forecast,
            # this loop fired on `d_min < hard_r` alone. So an aircraft flying
            # straight away from a wall at 5.00 m/s passed through untouched
            # while 5.01 - two tenths of a percent over the speed cap, enough
            # to drag it into the repair loop - came out at 3.57 m/s with a
            # sideways component it never asked for.
            content = not self._check_clearance(c, state, probe, steps, dists)
            d_min = min(dists)
            if content and d_min >= hard_r:
                continue                       # clear of the ring; nothing to do
            if content:
                # Inside the ring and the monitor is satisfied - but its test is
                # `d > prev + 1e-6` per forecast sample, an epsilon rather than a
                # rate, so a two-centimetre-per-second creep past a building
                # counts as escaping. Declining outright here would leave that
                # crawl in place. Raise the outward component to the recovery
                # speed and leave the tangential alone; never reshape an action
                # the monitor is happy with.
                oxc, oyc = self._away_dir(state.x, state.y, hard_r)
                if oxc == 0.0 and oyc == 0.0:
                    continue
                out_have = vx * oxc + vy * oyc
                need = min(cap, max(0.5, (hard_r - d_min) / max(self.lookahead_s, 1e-6)))
                if out_have < need:
                    vx += oxc * (need - out_have)
                    vy += oyc * (need - out_have)
                    repairs.append(Repair(
                        operator="ClearanceFix",
                        detail=(f"{c.id}: inside {hard_r}m and leaving at "
                                f"{out_have:.2f} m/s -> raised to {need:.2f}")))
                continue

            # the FIRST offending pose is the obstacle we have to steer off; at
            # t = 0 that is the current position, which is the old behaviour.
            hit = steps[next(i for i, d in enumerate(dists) if d < hard_r)][1]
            ox, oy = self._away_dir(hit.x, hit.y, hard_r)
            if ox == 0.0 and oy == 0.0:
                ox, oy = self._away_dir(state.x, state.y, hard_r)
            if ox == 0.0 and oy == 0.0:
                continue                       # nothing to steer by; rescue decides

            # `before` measured the SAME way as `after`.
            #
            # It used to be `min(dists)`, which includes t = 0, while `after`
            # comes from _clear_min_dist, which excludes it. At (38, 20.5)
            # flying away at 5 m/s that is 0.886 against 3.696 - a 4.2x gap, so
            # `after > before` held automatically and the "never leave it
            # worse" invariant below was suppressed exactly where it mattered.
            before = self._clear_min_dist(state, probe)

            out_c = vx * ox + vy * oy          # outward component already commanded
            tx, ty = vx - out_c * ox, vy - out_c * oy      # tangential = mission part
            depth = hard_r - d_min             # how far inside the ring we are
            # taper the tangential (mission) motion, floored so we still move
            scale = max(_CLEAR_MIN_SCALE, 1.0 - depth / max(soft_r, 1e-6))
            push = min(cap, max(0.5, depth / max(self.lookahead_s, 1e-6)))
            # FLOOR, not assign: never slow an escape that is already faster
            # than the recovery this repair would have sized.
            out_n = min(cap, max(out_c, push))
            # The cap eats the TANGENTIAL first - scaling the whole vector
            # would undo the push that was just computed.
            room = math.sqrt(max(cap * cap - out_n * out_n, 0.0))
            t_mag = math.hypot(tx, ty)
            t_scale = min(scale, room / t_mag) if t_mag > 1e-9 else 0.0
            cx = tx * t_scale + ox * out_n
            cy = ty * t_scale + oy * out_n

            # INVARIANT: a repair must never leave the forecast worse than it
            # found it. Where the local gradient is a poor guide (corners, two
            # buildings in play) fall back to the direction search rather than
            # shipping a "fix" that flies further in.
            cand = Action4D(vx=cx, vy=cy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
            after = self._clear_min_dist(state, cand)
            if after <= before + 1e-9:
                esc = self._best_clear_dir(state, probe, cap)
                esc_d = self._clear_min_dist(state, esc) if esc is not None else None
                if esc_d is not None and esc_d > max(after, before):
                    cx, cy, after = esc.vx, esc.vy, esc_d
                elif before >= after:
                    # Nothing on offer improves on the action we were handed, so
                    # ship THAT rather than a "fix" that scores worse. `before`
                    # is by definition its score, which makes this the monotone
                    # answer instead of a guess.
                    cx, cy, after = vx, vy, before
            vx, vy = cx, cy
            repairs.append(Repair(
                operator="ClearanceFix",
                detail=(f"{c.id}: dist {d_min:.2f}m < {hard_r}m -> push "
                        f"({ox:+.2f},{oy:+.2f}) at {out_n:.2f} m/s "
                        f"(commanded {out_c:+.2f}, recovery needs {push:.2f}), "
                        f"tangential x{t_scale:.2f}, forecast {before:.2f}->{after:.2f}m")))
        return Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)

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
                # FLOOR the exit speed, never assign it.
                #
                # `spd` stays min(2.0, cap): 2.0 is the reference
                # implementation's `escape_speed_mps` default, chosen because
                # GeofenceEscape is a recovery operator exempt from the
                # magnitude cap and should leave at a moderate, envelope-safe
                # speed rather than bolt down an unvalidated straight line.
                #
                # What changed is that it was ASSIGNED over the whole
                # horizontal vector. An aircraft 1 m inside the boundary
                # already leaving at 4.00 m/s passed through untouched, while
                # 4.01 - a quarter of a percent over the speed cap, enough to
                # drag it into the repair loop - was slowed to 2.00 and took
                # twice as long to get out of a P0 zone.
                #
                # Note this is evaluated against the MUTATING (vx, vy): with
                # two overlapping fences the second must see what the first
                # left behind, or the last one silently wins.
                spd = min(2.0, cap)
                out_now = vx * ox + vy * oy          # outward speed commanded
                probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                if not self._check_fence(f, poly, state, probe):
                    # The monitor is content, so this action IS escaping - but
                    # it forgives anything above 0.1 m/s, which would leave a
                    # crawl. Raise the outward component to the floor and leave
                    # the tangential alone rather than reshaping a legal action.
                    if out_now < spd:
                        vx += ox * (spd - out_now)
                        vy += oy * (spd - out_now)
                        repairs.append(Repair(
                            operator="GeofenceEscape",
                            detail=(f"{f.id}: inside and leaving at "
                                    f"{out_now:.2f} m/s -> raised to {spd:.1f}")))
                    continue
                keep = min(cap, max(out_now, spd))
                vx, vy = ox * keep, oy * keep
                repairs.append(Repair(operator="GeofenceEscape",
                                      detail=(f"{f.id}: inside -> exit at {keep:.1f} m/s "
                                              f"(commanded {out_now:+.2f})")))
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
                    # ...but not straight into a building. Both signs slide along
                    # the same fence edge, so when the preferred side BREAKS the
                    # clearance rule the other side is free to take. Without this
                    # the anti-stall tangent gets vetoed by the clearance check
                    # that already ran, and the P0 guard brakes. The turn side is
                    # only overridden on an actual violation — a legal tangent
                    # keeps following the raw action's intent.
                    if self._dist is not None and self._clear:
                        hard = min(c.min_clearance_m for c in self._clear)
                        pro = Action4D(vx=tx * spd, vy=ty * spd,
                                       vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                        if self._clear_min_dist(state, pro) < hard:
                            con = Action4D(vx=-tx * spd, vy=-ty * spd,
                                           vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                            if self._clear_min_dist(state, con) >= hard:
                                tx, ty = -tx, -ty
                    vx, vy = tx * spd, ty * spd
                repairs.append(Repair(operator="GeofenceSlide",
                                      detail=f"{f.id}: removed {into:.2f} m/s into-zone component"))
        return Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)

    # ---------------- mid-flight policy update ---------------- #

    def set_subject(self, x: float | None, y: float | None = None,
                    subject_class: str | None = None) -> None:
        """Tell the Shield where the tracked subject is, once per tick.

        The Shield cannot see. SubjectStandoff rules are inert until the
        perception stack supplies this, and calling `set_subject(None)` when the
        target is lost is REQUIRED rather than optional: a stale position would
        have the Shield enforcing a standoff from where the subject used to be,
        which is both wrong and unfalsifiable from the logs.

        `subject_class` is what selects between per-class rules, so a policy can
        hold 10 m from a pedestrian and 5 m from a vehicle.
        """
        if x is None or y is None:
            self._subject = None
            self._subject_class = None
            return
        self._subject = (float(x), float(y))
        self._subject_class = subject_class

    @staticmethod
    def _cap_sparing_radial(a: Action4D, ux: float, uy: float,
                            keep: float, cap: float) -> Action4D:
        """Fit under `cap` by spending the TANGENTIAL component, not the escape.

        Scaling the whole horizontal vector is the obvious way to respect the
        speed cap and the wrong one during a recovery: it shrinks the very
        component that is getting the aircraft out. Measured on the standoff
        rule, the end-of-pass SpeedClamp took a 3.00 m/s recovery down to 2.12
        while faithfully preserving 2.12 m/s of *mission* motion - it spent the
        budget on the part that was not urgent.

        `(ux, uy)` is the unit escape direction and `keep` the outward speed
        that must survive. Whatever room the cap leaves goes to the tangential
        part; if `keep` alone exceeds the cap, the escape is clipped to the cap
        and the tangential is dropped entirely.
        """
        keep = min(abs(keep), cap)
        out_v = (a.vx * ux + a.vy * uy)
        tx, ty = a.vx - out_v * ux, a.vy - out_v * uy      # tangential remainder
        t_mag = math.hypot(tx, ty)
        room = math.sqrt(max(cap * cap - keep * keep, 0.0))
        if t_mag > room > 0.0:
            tx, ty = tx * room / t_mag, ty * room / t_mag
        elif room <= 0.0:
            tx = ty = 0.0
        out_keep = max(out_v, keep) if out_v >= 0 else keep
        return Action4D(vx=tx + ux * out_keep, vy=ty + uy * out_keep,
                        vz_up=a.vz_up, yaw_rate=a.yaw_rate)

    def _repair_standoff(self, state: State, a: Action4D,
                         repairs: list["Repair"]) -> Action4D:
        """Remove the closing component of velocity along the line to the subject.

        Not a brake and not a reversal: the tangential component survives, so the
        aircraft can still circle the subject at the held range and keep it in
        frame. Killing the whole velocity would stop the mission to satisfy a
        rule that only objects to one direction of travel.
        """
        if self._subject is None:
            return a
        sx, sy = self._subject
        worst = None
        for so in self._standoffs:
            if so.binds(self._subject_class):
                worst = so if worst is None or so.min_range_m > worst.min_range_m else worst
        if worst is None:
            return a

        dx, dy = sx - state.x, sy - state.y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return a                       # directly overhead; no defined line
        ux, uy = dx / d, dy / d            # unit vector pointing AT the subject

        closing = a.vx * ux + a.vy * uy    # positive means approaching
        ring = worst.min_range_m

        # The REPAIR must trigger on the same criterion the CHECK uses, or it
        # declines to act on the very violation that was raised and the action
        # falls through to the brake - and, inside the ring, to the rescue
        # search, which was measured emitting +5 m/s straight AT the subject.
        # The check is predictive over the full lookahead, so this must be too:
        # at 4 m/s and a 3 s horizon the aircraft commits 12 m ahead of itself.
        breach_ahead = (d - closing * self.lookahead_s) < ring - 1e-9

        if d < ring - 1e-9:
            # ALREADY inside. Removing the closing component would leave the
            # range exactly where it is, which the check reads as "not opening" -
            # so the violation would persist every tick and the aircraft would be
            # frozen at a distance the policy forbids. Recovery means opening the
            # range, the same reasoning as the clearance ring's push-out.
            cap = min((k.speed_max_mps for k in self._kins), default=4.0)
            want = min(cap, max(0.5, (ring - d) / max(self.lookahead_s, 1e-3)))
            # FLOOR the opening rate, never assign it.
            #
            # `-closing` is the opening speed already commanded. Writing the
            # radial component to `want` outright made the repair a CEILING on
            # a legal escape: at 9 m from a pedestrian with a 10 m ring, an
            # action opening at 3.00 m/s passed through untouched, while 3.01 -
            # a third of a percent over the speed cap, enough to drag it into
            # the repair loop - came out at 0.500 m/s. Six times slower escape
            # from a P0 breach, and the KPI could not see it because the result
            # still re-checks clean.
            #
            # It is a floor and not a decline for the opposite reason: the
            # monitor forgives any opening above 0.1 m/s, so a repair that
            # simply stood aside would leave a 0.11 m/s crawl out of a ring the
            # policy forbids near a person.
            out = min(cap, max(want, -closing))
            vx = a.vx - (closing + out) * ux
            vy = a.vy - (closing + out) * uy
            repairs.append(Repair(
                operator="StandoffRecover",
                detail=(f"range {d:.1f}m inside min {ring}m: opening at "
                        f"{out:.2f} m/s (commanded {-closing:+.2f}, "
                        f"recovery needs {want:.2f}), tangential motion kept")))
            return self._cap_sparing_radial(
                Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate),
                ux=-ux, uy=-uy, keep=out, cap=cap)

        if closing <= 0.0 or not breach_ahead:
            return a                       # opening already, or no breach coming

        vx, vy = a.vx - closing * ux, a.vy - closing * uy
        repairs.append(Repair(
            operator="StandoffHold",
            detail=(f"range {d:.1f}m, closing {closing:.2f} m/s would breach "
                    f"min {ring}m within {self.lookahead_s:.0f}s: closing "
                    f"component removed, tangential motion kept")))
        return Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)

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
        # Finiteness first: every check below is a comparison, and NaN loses them
        # all, so an unsanitised action would be declared legal and passed
        # straight through. See _sanitise().
        raw, nonfinite = _sanitise(raw)
        violations = self._check(state, raw)
        if nonfinite:
            violations = violations + [Violation(
                rule_id="action-finite", category="contract",
                detail=f"non-finite channel(s) {','.join(nonfinite)} zeroed",
                predicted_at_s=0.0)]

        if not violations:
            # Nothing was wrong with it, so the re-check is empty by
            # construction and does not need running again.
            return ShieldDecision(raw=raw, emitted=raw)   # untouched passthrough

        repairs: list[Repair] = []
        if nonfinite:
            repairs.append(Repair(operator="Sanitise",
                                  detail=f"{','.join(nonfinite)} -> 0.0"))
        fixed = self._repair_kinematic(raw, repairs)
        fixed = self._repair_altitude(state, fixed, repairs)
        # Clearance and geofence are COUPLED: the anti-stall tangent can leave
        # the clearance ring violated and the clearance push can aim into a
        # fence, so the chain is iterated to a fixed point. Running it once as a
        # pipeline leaves whatever the LAST operator produced un-repaired, which
        # is exactly what the P0 guard then brakes on — every tick, forever.
        for _ in range(_REPAIR_PASSES):
            fixed = self._repair_clearance(state, fixed, repairs)
            fixed = self._repair_standoff(state, fixed, repairs)
            fixed = self._repair_geofence(state, fixed, repairs)
            # ...and the caps last: AltitudeFix sizes vz to reach the band in one
            # lookahead, which can overshoot the climb cap on a deep recovery.
            fixed = self._repair_kinematic(fixed, repairs)
            if not self._check(state, fixed):
                break

        # P0 escape guard: repaired action must re-check clean.
        blocking = self._check(state, fixed)
        if blocking:
            # BRAKE is a fail-safe only where standing still is legal. Inside
            # the clearance ring a zero action leaves d_next == d_now, so the
            # same violation is raised next tick and the vehicle is frozen into
            # the violation instead of recovering from it. Look for a heading
            # that re-checks CLEAN before considering a stop.
            clean = best = None
            if (any(v.category == "clearance" for v in blocking)
                    or self._check(state, BRAKE)):
                cap = min((k.speed_max_mps for k in self._kins), default=4.0)
                clean, best = self._rescue(state, raw, fixed, cap)
            if clean is not None:
                repairs.append(Repair(
                    operator="ClearanceEscape",
                    detail="repair not converged -> recovery heading"))
                fixed = clean
            elif not self._check(state, BRAKE):
                repairs.append(Repair(operator="Brake", detail="repair not converged -> stop"))
                return ShieldDecision(raw=raw, emitted=BRAKE, violations=violations,
                                      repairs=_dedupe(repairs), braked=True,
                                      emitted_violations=self._check(state, BRAKE))
            elif best is not None:
                repairs.append(Repair(
                    operator="ClearanceEscape",
                    detail="stopping is itself illegal here -> best-effort recovery"))
                fixed = best
            else:
                repairs.append(Repair(operator="Brake", detail="repair not converged -> stop"))
                return ShieldDecision(raw=raw, emitted=BRAKE, violations=violations,
                                      repairs=_dedupe(repairs), braked=True,
                                      emitted_violations=self._check(state, BRAKE))

        # One more _check, on the action that is actually leaving the building.
        # In the common case `fixed` already re-checked clean inside the loop
        # above and this is a repeat; in the `best is not None` branch it is the
        # only check that has ever been run against what gets flown.
        return ShieldDecision(raw=raw, emitted=fixed, violations=violations,
                              repairs=_dedupe(repairs),
                              emitted_violations=self._check(state, fixed))
