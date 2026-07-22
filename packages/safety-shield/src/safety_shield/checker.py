"""Violation checker — the single rule-evaluation path in the system.

Reuses the Policy DSL IR (safety-shield.md: "there is exactly one rule-evaluation
code path"). For the Phase-1 slice it runs two of the three categories:

* **Geometric** — does any predicted pose fall inside a polygon fence?
* **Envelope** — does any predicted altitude leave an altitude envelope?

Time-window checking is Phase 2 (needs the hot-apply classes).

Trend-awareness for the geometric category (ported from the in-house prototype's
empirical findings): a vehicle that is *already inside* a zone but actively
flying *outward* is escaping, not violating. Without this, braking the vehicle
in place locks the violation forever (a deadlock). The outward test uses the
IR's outward normal in the local ENU plane; a small threshold rejects jitter.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from policy_dsl import PolicyIR
from policy_dsl.ir import PolygonRecord
from vlaguard_common import Action4D

from safety_shield.kinematics import PredictedPose, VehicleState, body_to_enu, predict

Category = Literal["geometric", "envelope"]

# A vehicle inside a zone is "escaping" when its outward speed exceeds this
# (m/s). Below it, the repair stack must drive it out (GeofenceEscape). Small so
# near-stationary drift does not masquerade as recovery.
_ESCAPE_SPEED_MPS = 0.1


@dataclass(frozen=True)
class Violation:
    rule_id: str
    category: Category
    priority: str
    violation_action: str
    first_hit_s: float
    # offending predicted position (lat/lon for geometric, alt for envelope)
    hit_lat: float
    hit_lon: float
    hit_alt_m: float


def check(
    ir: PolicyIR, state: VehicleState, action: Action4D, horizon_s: float = 5.0
) -> list[Violation]:
    """Return all violations the predicted trajectory would incur, earliest-hit first."""
    origin_xy = ir.projection.to_xy(state.lat, state.lon)
    poses = predict(state, action, origin_xy, horizon_s=horizon_s)

    # spatial pre-filter: only polygons within reach over the horizon
    reach_m = _reach(poses, origin_xy)
    candidates = ir.polygons_near(state.lat, state.lon, radius_m=reach_m + 1.0)

    violations: list[Violation] = []

    for rec in candidates:
        # Trend-aware recovery: if the vehicle is ALREADY inside this polygon,
        # flag a violation only when it is NOT actively flying outward. An
        # escaping vehicle passes; a stalled/heading-deeper one must be repaired
        # (the GeofenceEscape operator drives it straight out). Braking in place
        # here would lock the violation forever.
        if ir.signed_distance(rec, state.lat, state.lon) < 0:
            if _escapes(ir, rec, state, action):
                continue  # heading outward -> not a violation
            # stalled inside: flag at t=0 so the repair stack drives it out
            violations.append(
                Violation(
                    rule_id=rec.id,
                    category="geometric",
                    priority=rec.priority,
                    violation_action=rec.violation_action,
                    first_hit_s=0.0,
                    hit_lat=state.lat,
                    hit_lon=state.lon,
                    hit_alt_m=state.alt_agl_m,
                )
            )
            continue
        for pose in poses:
            lat, lon = ir.projection.to_latlon(pose.east_m, pose.north_m)
            if ir.signed_distance(rec, lat, lon) < 0:
                violations.append(
                    Violation(
                        rule_id=rec.id,
                        category="geometric",
                        priority=rec.priority,
                        violation_action=rec.violation_action,
                        first_hit_s=pose.t_s,
                        hit_lat=lat,
                        hit_lon=lon,
                        hit_alt_m=pose.alt_agl_m,
                    )
                )
                break  # earliest hit for this rule is enough

    for env in ir.envelopes:
        for pose in poses:
            if pose.alt_agl_m < env.altitude_min_m or pose.alt_agl_m > env.altitude_max_m:
                lat, lon = ir.projection.to_latlon(pose.east_m, pose.north_m)
                violations.append(
                    Violation(
                        rule_id=env.id,
                        category="envelope",
                        priority=env.priority,
                        violation_action=env.violation_action,
                        first_hit_s=pose.t_s,
                        hit_lat=lat,
                        hit_lon=lon,
                        hit_alt_m=pose.alt_agl_m,
                    )
                )
                break

    violations.sort(key=lambda v: v.first_hit_s)
    return violations


def _reach(poses: list[PredictedPose], origin_xy: tuple[float, float]) -> float:
    e0, n0 = origin_xy
    if not poses:
        return 0.0
    last = poses[-1]
    return float(((last.east_m - e0) ** 2 + (last.north_m - n0) ** 2) ** 0.5)


def _escapes(ir: PolicyIR, rec: PolygonRecord, state: VehicleState, action: Action4D) -> bool:
    """True when the vehicle is inside the polygon but the signed distance is rising.

    Signed distance is negative inside; rising (toward zero, then positive) means
    the vehicle is getting closer to leaving — toward *any* exit wall. This is the
    clean, frame-correct trend signal: unlike an outward-normal dot product it has
    no centre-point ambiguity (several equidistant walls competing), and unlike
    the unsigned distance it stays correct when the probe crosses the boundary
    (outside is positive, so the delta keeps rising instead of flipping sign). We
    probe one second of motion; a small threshold rejects jitter.
    """
    ve, vn, _vu = body_to_enu(action, state.yaw_rad)
    now_sd = ir.signed_distance(rec, state.lat, state.lon)
    ex, nx = ir.projection.to_xy(state.lat, state.lon)
    probe_lat, probe_lon = ir.projection.to_latlon(ex + ve, nx + vn)
    next_sd = ir.signed_distance(rec, probe_lat, probe_lon)
    return (next_sd - now_sd) > _ESCAPE_SPEED_MPS
