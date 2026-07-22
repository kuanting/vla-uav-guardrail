"""Trend-aware recovery regression tests (the "already inside the NFZ" case).

These tests prove a real defect in the original Phase-1 Shield: a vehicle that
is *already inside* a P0 no-fly zone cannot recover, because every repair
attempt reports a penetration depth larger than the lateral threshold, so the
Shield brakes — and braking while inside locks the violation in place forever.

Two fixes, both ported from the in-house prototype's empirical findings, close
this:

* **trend-aware checking** (checker.py): a vehicle that is inside a zone but
  actively flying *outward* is not flagged as a violation;
* **GeofenceEscape** (repair.py): when a vehicle is inside and *not* trending
  outward, the repair stack emits a direct "fly straight out" action that is
  exempt from the projection magnitude cap (recovery, not projection).

Each test below is written so that it fails on the original code and passes
once the two fixes land — the classic red/green regression shape.
"""

from __future__ import annotations

import math
from pathlib import Path

from policy_dsl import ingest_text
from safety_shield import SafetyShield, VehicleState, check
from safety_shield.kinematics import body_to_enu
from vlaguard_common import Action4D

DEMO = Path(__file__).resolve().parents[3] / "bundles" / "itri-icl-2026-demo.yaml"

# NFZ school-yard spans lat 25.0421–25.0428, lon 121.5310–121.5318 (centroid ~
# 25.04245, 121.5314). We place the vehicle at the centroid (well inside) and
# have it head West (yaw=-pi/2 in NED: 0=North, CW+) toward the nearer west
# wall, i.e. flying *out* of the zone.
_INSIDE_LAT = 25.04245
_INSIDE_LON = 121.5314
# NED heading West: yaw = -pi/2 (or 3pi/2).
_YAW_WEST = -math.pi / 2
_SPEED_OUT = 5.0


def _ir():
    return ingest_text(DEMO.read_text())


def _ir():
    return ingest_text(DEMO.read_text())


def _inside() -> VehicleState:
    """Vehicle at the NFZ centroid, heading West (out toward the nearer wall)."""
    return VehicleState(lat=_INSIDE_LAT, lon=_INSIDE_LON, alt_agl_m=50.0, yaw_rad=_YAW_WEST)


def _action_out() -> Action4D:
    """Body-forward velocity at the West heading = flying straight out."""
    return Action4D(vx=_SPEED_OUT, vy=0.0, vz=0.0, yaw_rate=0.0)


# --------------------------------------------------------------------------- #
# Proof of the defect: an inside vehicle flying OUTWARD must NOT be flagged.
# --------------------------------------------------------------------------- #


def test_inside_vehicle_flying_outward_is_not_in_violation():
    """A vehicle already inside the NFZ, heading West (out), is escaping — the
    checker must not treat that as a violation. On the original code this fails
    because every predicted pose inside the polygon counts, regardless of trend.
    """
    ir = _ir()
    state = _inside()
    action = _action_out()
    violations = check(ir, state, action)
    assert not any(
        v.category == "geometric" for v in violations
    ), "escaping vehicle must not be flagged as a geometric violation"


# --------------------------------------------------------------------------- #
# Proof of the defect: the original Shield brakes an escaping vehicle dead.
# --------------------------------------------------------------------------- #


def test_shield_lets_escaping_inside_vehicle_continue():
    """The Shield's emitted action for an escaping inside-vehicle must be the
    (unbraked) escape action, not a full stop. On the original code the repair
    magnitude exceeds the cap and the Shield brakes -> deadlock.
    """
    ir = _ir()
    shield = SafetyShield(ir)
    state = _inside()
    action = _action_out()
    decision = shield.tick(state, action)
    emitted = decision.emitted_action
    assert not (emitted.vx == 0.0 and emitted.vy == 0.0 and emitted.vz == 0.0), (
        "escaping inside-vehicle must not be braked to a full stop (deadlock)"
    )


# --------------------------------------------------------------------------- #
# End-to-end recovery: a stuck-inside vehicle actually leaves the zone.
# --------------------------------------------------------------------------- #


def test_inside_vehicle_actually_exits_the_zone():
    """Fly the inside vehicle tick-by-tick with the Shield on; it must reach a
    position *outside* the P0 polygon within a bounded number of ticks. This is
    the real-world guarantee: recovery, not just a single clean tick.

    On the original code the Shield brakes every tick, so the vehicle never
    moves and never exits — the test times out.
    """
    ir = _ir()
    shield = SafetyShield(ir)
    state = _inside()  # heading West toward the nearer west wall
    p0 = [p for p in ir.polygons if p.priority == "P0"]
    assert p0, "demo policy must have a P0 polygon for this test"

    dt = 0.1
    exited = False
    for _ in range(200):  # generous bound (~20 s at 10 Hz); real exit is ~seconds
        decision = shield.tick(state, _action_out())
        e = decision.emitted_action
        # advance on the EMITTED action, in the IR's local ENU plane
        ex, nx = ir.projection.to_xy(state.lat, state.lon)
        ve, vn, vu = body_to_enu(e, state.yaw_rad)
        new_lat, new_lon = ir.projection.to_latlon(ex + ve * dt, nx + vn * dt)
        state = VehicleState(
            lat=new_lat, lon=new_lon, alt_agl_m=state.alt_agl_m + vu * dt, yaw_rad=state.yaw_rad
        )
        if all(ir.signed_distance(poly, state.lat, state.lon) >= 0 for poly in p0):
            exited = True
            break
    assert exited, "vehicle never left the NFZ — recovery failed (deadlock persists)"


# --------------------------------------------------------------------------- #
# Recovery via GeofenceEscape: a vehicle inside and heading DEEPER must still
# be driven out by the repair stack (the trend-aware check lets an escaping
# action pass, but a VLA that insists on diving deeper is repaired, not braked).
# --------------------------------------------------------------------------- #


def test_inside_vehicle_heading_deeper_is_driven_out():
    """The VLA here demands to fly East (deeper into the zone, away from the
    nearer west wall). The trend-aware check flags it; LateralProjection defers
    to the recovery operator; ``GeofenceEscape`` overrides the dive with a
    straight-out action that is exempt from the projection magnitude cap. The
    vehicle must still exit the zone within a bounded time.

    On the original code this brakes on the first tick and never recovers.
    """
    ir = _ir()
    shield = SafetyShield(ir)
    p0 = [p for p in ir.polygons if p.priority == "P0"]
    # just inside the west wall, body heading East -> toward the zone interior
    state = VehicleState(
        lat=_INSIDE_LAT, lon=121.5311, alt_agl_m=50.0, yaw_rad=math.pi / 2
    )
    dt = 0.1
    escaped = False
    for _ in range(300):  # ~30 s bound; real recovery is ~5 s
        action = Action4D(vx=5.0, vy=0.0, vz=0.0, yaw_rate=0.0)  # VLA insists on East
        decision = shield.tick(state, action)
        e = decision.emitted_action
        ex, nx = ir.projection.to_xy(state.lat, state.lon)
        ve, vn, vu = body_to_enu(e, state.yaw_rad)
        new_lat, new_lon = ir.projection.to_latlon(ex + ve * dt, nx + vn * dt)
        state = VehicleState(
            lat=new_lat, lon=new_lon, alt_agl_m=state.alt_agl_m + vu * dt, yaw_rad=state.yaw_rad
        )
        if all(ir.signed_distance(poly, state.lat, state.lon) >= 0 for poly in p0):
            escaped = True
            break
    assert escaped, (
        "deeper-heading inside vehicle never left the NFZ — GeofenceEscape failed"
    )
