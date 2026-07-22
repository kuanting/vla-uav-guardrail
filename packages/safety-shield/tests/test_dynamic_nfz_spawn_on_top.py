"""Dynamic NFZ spawn-on-top regression tests.

The design docs (policy-dsl.md, safety-shield.md) say three constraint classes
hot-apply mid-mission, ``dynamic_nfz`` among them. An obvious stress case the
Phase-1 tests never covered: a dynamic NFZ that appears **directly on top of the
vehicle** — the vehicle is instantly inside a zone it was not violating a tick
ago. With the trend-aware + GeofenceEscape fix this must recover cleanly in two
sub-cases:

* the vehicle's current action carries it *out* of the new zone -> passes (no
  violation flagged, mission continues);
* the vehicle's current action carries it *deeper* into the new zone -> the
  GeofenceEscape operator overrides it and flies the vehicle out.

Both tests build a fresh IR with the spawned polygon already active (simulating
the instant after the hot-apply) so they exercise the checker/repair path with
no REST plumbing.
"""

from __future__ import annotations

from pathlib import Path

from policy_dsl import ingest_text
from policy_dsl.ir import build_ir
from policy_dsl.models import (
    LatLon,
    PolicyDoc,
    PolygonFence,
    PolygonGeometry,
)
from safety_shield import SafetyShield, VehicleState, check
from safety_shield.kinematics import body_to_enu
from vlaguard_common import Action4D

DEMO = Path(__file__).resolve().parents[3] / "bundles" / "itri-icl-2026-demo.yaml"

# Vehicle is well clear of the static school-yard, cruising north over open
# ground. The dynamic NFZ spawns centred on it.
_VEH_LAT = 25.04320
_VEH_LON = 121.531012
_VEH_ALT = 50.0


def _base_ir():
    return ingest_text(DEMO.read_text())


def _spawned_doc() -> PolicyDoc:
    """The demo policy + a dynamic NFZ spawned centred on the vehicle."""
    base = _base_ir().canonical
    doc = PolicyDoc.model_validate(base)
    doc.constraints.append(_spawned_fence())
    return doc


def _spawned_fence() -> PolygonFence:
    # 60 m square centred on the vehicle -> vehicle is at its exact centre.
    half = 0.00027  # ~30 m in degrees at this latitude
    return PolygonFence(
        id="nfz-dynamic-on-top",
        type="polygon_fence",
        constraint_type="hard",
        scope="global",
        priority="P0",
        layer="site",
        violation_action="project_fix",
        geometry=PolygonGeometry(
            vertices=[
                LatLon(lat=_VEH_LAT - half, lon=_VEH_LON - half),
                LatLon(lat=_VEH_LAT + half, lon=_VEH_LON - half),
                LatLon(lat=_VEH_LAT + half, lon=_VEH_LON + half),
                LatLon(lat=_VEH_LAT - half, lon=_VEH_LON + half),
            ],
            altitude_floor_m=0,
            altitude_ceiling_m=200,
            altitude_ref="AGL",
        ),
    )


def _vehicle(yaw_rad: float) -> VehicleState:
    return VehicleState(lat=_VEH_LAT, lon=_VEH_LON, alt_agl_m=_VEH_ALT, yaw_rad=yaw_rad)


def _ir_with_spawn():
    """A runtime IR that already contains the spawned-on-top dynamic NFZ."""
    return build_ir(_spawned_doc())


# --------------------------------------------------------------------------- #
# Sub-case A: the vehicle's heading already takes it out of the spawned zone.
# --------------------------------------------------------------------------- #


def test_spawn_on_top_outward_heading_not_flagged():
    """A dynamic NFZ appears on the vehicle while it is cruising north; its
    northward action carries it out of the zone's north edge. The checker must
    not flag this — the vehicle is escaping from the instant the zone appears.
    """
    ir = _ir_with_spawn()
    state = _vehicle(yaw_rad=0.0)  # heading North -> out the north edge
    action = Action4D(vx=5.0, vy=0.0, vz=0.0, yaw_rate=0.0)
    violations = check(ir, state, action)
    assert not any(
        v.category == "geometric" and v.rule_id == "nfz-dynamic-on-top"
        for v in violations
    ), "vehicle escaping the spawned-on-top zone must not be flagged"


def test_spawn_on_top_outward_heading_vehicle_exits():
    """End-to-end: fly the vehicle tick-by-tick from inside the spawned zone
    heading north; it must leave the P0 polygon within a bounded time.
    """
    ir = _ir_with_spawn()
    shield = SafetyShield(ir)
    p0 = [p for p in ir.polygons if p.id == "nfz-dynamic-on-top"]
    state = _vehicle(yaw_rad=0.0)
    dt = 0.1
    exited = False
    for _ in range(200):
        decision = shield.tick(state, Action4D(vx=5.0, vy=0.0, vz=0.0, yaw_rate=0.0))
        e = decision.emitted_action
        ex, nx = ir.projection.to_xy(state.lat, state.lon)
        ve, vn, vu = body_to_enu(e, state.yaw_rad)
        nlat, nlon = ir.projection.to_latlon(ex + ve * dt, nx + vn * dt)
        state = VehicleState(
            lat=nlat, lon=nlon, alt_agl_m=state.alt_agl_m + vu * dt, yaw_rad=state.yaw_rad
        )
        if all(ir.signed_distance(poly, state.lat, state.lon) >= 0 for poly in p0):
            exited = True
            break
    assert exited, "vehicle never left the spawned-on-top zone"


# --------------------------------------------------------------------------- #
# Sub-case B: the vehicle's heading takes it DEEPER into the spawned zone.
# --------------------------------------------------------------------------- #


def test_spawn_on_top_deeper_heading_is_driven_out():
    """A dynamic NFZ appears on the vehicle while it is heading across the zone
    interior (its action would carry it deeper, not out). The Shield must repair
    via GeofenceEscape and the vehicle must still leave within a bounded time.
    """
    ir = _ir_with_spawn()
    shield = SafetyShield(ir)
    p0 = [p for p in ir.polygons if p.id == "nfz-dynamic-on-top"]
    # vehicle at the south edge, heading north -> straight into the zone interior
    south_edge_lat = _VEH_LAT - 0.00020
    state = VehicleState(lat=south_edge_lat, lon=_VEH_LON, alt_agl_m=_VEH_ALT, yaw_rad=0.0)
    dt = 0.1
    escaped = False
    for _ in range(300):
        # VLA insists on north (deeper into the zone)
        decision = shield.tick(state, Action4D(vx=5.0, vy=0.0, vz=0.0, yaw_rate=0.0))
        e = decision.emitted_action
        ex, nx = ir.projection.to_xy(state.lat, state.lon)
        ve, vn, vu = body_to_enu(e, state.yaw_rad)
        nlat, nlon = ir.projection.to_latlon(ex + ve * dt, nx + vn * dt)
        state = VehicleState(
            lat=nlat, lon=nlon, alt_agl_m=state.alt_agl_m + vu * dt, yaw_rad=state.yaw_rad
        )
        if all(ir.signed_distance(poly, state.lat, state.lon) >= 0 for poly in p0):
            escaped = True
            break
    assert escaped, "deeper-heading vehicle never left the spawned-on-top zone"
