"""Kinematic simulation harness — run the real safety_shield core, no ROS needed.

This is the functional-rail vertical slice that the Grant's ``dev`` topology
describes, minus the autopilot: a point-mass vehicle (``pos += v*dt``) driven by
the in-house VLA stub, with the *production* :class:`SafetyShield` doing the
monitoring/repair. It exists so the Shield logic can be exercised, A/B'd and
demoed end-to-end on a plain desktop, with the same audit log and KPI shape as
``demo/mid_term_demo.py`` but with plotting, dynamic NFZ injection, and the
inside-zone recovery scenario.

Three canned scenarios (``--scenario``):

* ``crossing``  — straight path clips the school-yard NFZ. Shield OFF enters
                  it; Shield ON slides along the edge and completes.
* ``dynamic``   — an extra NFZ is hot-applied mid-flight on the detour path;
                  the Shield reroutes around both zones.
* ``recovery``  — the vehicle STARTS inside the NFZ. Without the trend-aware +
                  GeofenceEscape fixes this deadlocks; with them it flies out.

Outputs per run land in ``demo/out/<scenario>-<shield>/``: ``trajectory.png``,
``report.md``, and the ``shield_audit.jsonl`` the Shield's own AuditLog writes.

Run:

    uv run python -m demo.kinematic_demo --scenario crossing --shield off
    uv run python -m demo.kinematic_demo --scenario crossing --shield on
    uv run python -m demo.kinematic_demo --scenario dynamic  --shield on
    uv run python -m demo.kinematic_demo --scenario recovery --shield on
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TypedDict

from policy_dsl import ingest_file
from policy_dsl.ir import PolicyIR
from policy_dsl.models import LatLon, PolygonFence, PolygonGeometry
from safety_shield import AuditLog, SafetyShield, VehicleState
from safety_shield.kinematics import body_to_enu
from vlaguard_common import Action4D, policy_hash

ROOT = Path(__file__).resolve().parents[1]
DEMO_SRC = ROOT / "bundles" / "itri-icl-2026-demo.yaml"

DT = 0.1          # 10 Hz, the grant's action/monitor rate
MAX_TICKS = 1200  # 120 s cap
REACH_M = 2.0

# A second NFZ hot-applied in the ``dynamic`` scenario: it sits ~60 m east of
# the school-yard so the west-edge detour around the static zone now also has to
# clear it — a second, wider reroute mid-flight.
DYNAMIC_AT_S = 8.0
DYNAMIC_LAT0, DYNAMIC_LAT1 = 25.0426, 25.0434
DYNAMIC_LON0, DYNAMIC_LON1 = 121.5326, 121.5336


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt_agl_m: float


class Scenario(TypedDict):
    start: Waypoint
    target: Waypoint
    start_inside: bool


class TrajPoint(TypedDict):
    t: float
    lat: float
    lon: float
    alt: float
    intercepted: bool


# Each scenario's start/target sit near the NTUT demo site (lat 25.0421–25.0434,
# lon 121.5310–121.5336). The school-yard NFZ spans lat 25.0421–25.0428, lon
# 121.5310–121.5318 — straight-line missions that cross that band exercise the
# Shield; the recovery scenario starts on top of it.
SCENARIOS: dict[str, Scenario] = {
    "crossing": {
        "start": Waypoint(25.04195, 121.531012, 50.0),
        "target": Waypoint(25.04320, 121.531012, 50.0),
        "start_inside": False,
    },
    "dynamic": {
        "start": Waypoint(25.04195, 121.531012, 50.0),
        "target": Waypoint(25.04340, 121.533612, 50.0),
        "start_inside": False,
    },
    "recovery": {
        # start at the school-yard centroid -> already inside the P0 zone
        "start": Waypoint(25.04245, 121.5314, 50.0),
        "target": Waypoint(25.04195, 121.531012, 50.0),
        "start_inside": True,
    },
    "spawn_on_top": {
        # cruise north over open ground; a dynamic NFZ is hot-applied centred on
        # the vehicle at t=8 s, so it is instantly inside a zone it never neared.
        "start": Waypoint(25.0420, 121.531012, 50.0),
        "target": Waypoint(25.0440, 121.531012, 50.0),
        "start_inside": False,
    },
}


def _bearing_rad(state: VehicleState, target: Waypoint) -> float:
    """NED heading (0 = North, CW positive) from vehicle to target."""
    d_north = target.lat - state.lat
    d_east = (target.lon - state.lon) * math.cos(math.radians(state.lat))
    return math.atan2(d_east, d_north)


def _stub_action(state: VehicleState, target: Waypoint, cruise_mps: float = 5.0) -> Action4D:
    """Deliberately rule-ignorant pilot: body-forward straight at the target."""
    dz = max(-3.0, min(3.0, target.alt_agl_m - state.alt_agl_m))
    return Action4D(vx=cruise_mps, vy=0.0, vz=dz, yaw_rate=0.0)


def _make_dynamic_fence() -> PolygonFence:
    return PolygonFence(
        id="nfz-dynamic-event",
        type="polygon_fence",
        constraint_type="hard",
        scope="global",
        priority="P0",
        layer="site",
        violation_action="project_fix",
        geometry=PolygonGeometry(
            vertices=[
                LatLon(lat=DYNAMIC_LAT0, lon=DYNAMIC_LON0),
                LatLon(lat=DYNAMIC_LAT1, lon=DYNAMIC_LON0),
                LatLon(lat=DYNAMIC_LAT1, lon=DYNAMIC_LON1),
                LatLon(lat=DYNAMIC_LAT0, lon=DYNAMIC_LON1),
            ],
            altitude_floor_m=0,
            altitude_ceiling_m=200,
            altitude_ref="AGL",
        ),
    )


def hot_apply_dynamic(shield: SafetyShield, fence: PolygonFence) -> None:
    """Append a polygon to the live IR and bump its generation (design-doc hot path).

    The mid-flight hot-apply is Phase 2 in the design; this is the minimal,
    correct implementation a demo needs: rebuild the IR's polygon list + spatial
    index, re-derive the policy hash, and bump generation so audit records after
    this instant carry the new bundle identity.
    """
    ir = shield.ir
    from policy_dsl.ir import PolygonRecord  # local import: avoids cycle in module top
    from shapely.geometry import Polygon

    ring = [ir.projection.to_xy(v.lat, v.lon) for v in fence.geometry.vertices]
    ir.polygons.append(
        PolygonRecord(
            id=fence.id,
            constraint_type=fence.constraint_type,
            priority=fence.priority,
            violation_action=fence.violation_action,
            altitude_floor_m=fence.geometry.altitude_floor_m,
            altitude_ceiling_m=fence.geometry.altitude_ceiling_m,
            polygon=Polygon(ring),
        )
    )
    # rebuild STRtree + manifest identity
    ir.__post_init__()
    ir.generation += 1
    ir.policy_hash = policy_hash(ir.canonical)
    if shield.audit is not None:
        shield.audit.generation = ir.generation
        shield.audit.policy_hash = ir.policy_hash


def hot_apply_centred(shield: SafetyShield, state: VehicleState) -> None:
    """Hot-apply a dynamic NFZ centred on the vehicle (the spawn-on-top case).

    The hardest placement for the Shield: the vehicle is instantly inside a zone
    it never approached. Exercises the trend-aware checker (escaping?) and the
    GeofenceEscape recovery operator. Square ~60 m, P0, project_fix.
    """
    # ~30 m in degrees at this latitude for a 60 m square centred on the vehicle.
    half = 0.00027
    fence = PolygonFence(
        id="nfz-spawn-on-top",
        type="polygon_fence",
        constraint_type="hard",
        scope="global",
        priority="P0",
        layer="site",
        violation_action="project_fix",
        geometry=PolygonGeometry(
            vertices=[
                LatLon(lat=state.lat - half, lon=state.lon - half),
                LatLon(lat=state.lat + half, lon=state.lon - half),
                LatLon(lat=state.lat + half, lon=state.lon + half),
                LatLon(lat=state.lat - half, lon=state.lon + half),
            ],
            altitude_floor_m=0,
            altitude_ceiling_m=200,
            altitude_ref="AGL",
        ),
    )
    hot_apply_dynamic(shield, fence)


def run(scenario: str, shield_on: bool, out: Path) -> int:
    cfg = SCENARIOS[scenario]
    start, target = cfg["start"], cfg["target"]
    ir = ingest_file(DEMO_SRC)

    audit_path = out / "shield_audit.jsonl"
    audit_path.unlink(missing_ok=True)
    audit = AuditLog(audit_path, ir.policy_hash, ir.generation) if shield_on else None
    shield = SafetyShield(ir, audit=audit)

    state = VehicleState(
        lat=start.lat,
        lon=start.lon,
        alt_agl_m=start.alt_agl_m,
        yaw_rad=_bearing_rad_v0(start, target),
    )
    traj: list[TrajPoint] = []
    intercepts = 0
    dynamic_applied_at = None
    reached = False

    for tick in range(1, MAX_TICKS + 1):
        now_s = tick * DT

        if scenario == "dynamic" and dynamic_applied_at is None and now_s >= DYNAMIC_AT_S:
            hot_apply_dynamic(shield, _make_dynamic_fence())
            dynamic_applied_at = now_s

        if (
            scenario == "spawn_on_top"
            and dynamic_applied_at is None
            and now_s >= DYNAMIC_AT_S
        ):
            # hot-apply a NFZ centred on the vehicle's *current* position, so the
            # vehicle is instantly inside a zone it never approached (the design
            # docs' dynamic_nfz stress case, in its hardest placement).
            hot_apply_centred(shield, state)
            dynamic_applied_at = now_s

        # re-point the body at the target each tick (the stub assumes vx = toward target)
        yaw = _bearing_rad(state, target)
        state = VehicleState(lat=state.lat, lon=state.lon, alt_agl_m=state.alt_agl_m, yaw_rad=yaw)
        action = _stub_action(state, target)

        if shield_on:
            decision = shield.tick(state, action, ts=f"t+{now_s:.1f}s")
            emitted = decision.emitted_action
            if decision.intercepted:
                intercepts += 1
        else:
            emitted = action

        ve, vn, vu = body_to_enu(emitted, state.yaw_rad)
        ex, nx = ir.projection.to_xy(state.lat, state.lon)
        new_lat, new_lon = ir.projection.to_latlon(ex + ve * DT, nx + vn * DT)
        state = VehicleState(
            lat=new_lat, lon=new_lon, alt_agl_m=state.alt_agl_m + vu * DT, yaw_rad=yaw
        )

        traj.append(
            {
                "t": round(now_s, 2),
                "lat": state.lat,
                "lon": state.lon,
                "alt": state.alt_agl_m,
                "intercepted": shield_on and decision.intercepted,
            }
        )

        if _reached(state, target):
            reached = True
            break

    if shield_on and audit is not None:
        audit.close()

    # KPI: seconds spent inside any active P0 polygon (dynamic counts only after apply)
    nfz_seconds, entered, exit_s = _time_inside_nfz(ir, traj, dynamic_applied_at)
    # Some scenarios put the vehicle inside a zone by construction (it starts
    # there, or a dynamic NFZ spawns on top of it), so a non-zero in-zone time is
    # unavoidable — the meaningful gate there is whether it actually exited.
    instant_inside = cfg["start_inside"] or scenario == "spawn_on_top"
    _plot(out / "trajectory.png", ir, traj, start, target, dynamic_applied_at, shield_on, scenario)
    _report(
        out / "report.md", scenario, shield_on, ir, traj, intercepts, nfz_seconds,
        reached, dynamic_applied_at, entered, exit_s, instant_inside,
    )
    if instant_inside:
        kpi_ok = exit_s is not None
        kpi_txt = f"exit@{exit_s:.1f}s" if exit_s is not None else "never exited"
    else:
        kpi_ok = nfz_seconds == 0
        kpi_txt = f"{nfz_seconds:.1f}s in NFZ"
    print(
        f"[{scenario}/shield={'on' if shield_on else 'off'}] "
        f"ticks={len(traj)} intercepts={intercepts} {kpi_txt} "
        f"reached={reached} -> {out.name}  (KPI {'PASS' if kpi_ok else 'FAIL'})"
    )
    return 0 if kpi_ok else 1


def _bearing_rad_v0(start: Waypoint, target: Waypoint) -> float:
    return _bearing_rad(
        VehicleState(lat=start.lat, lon=start.lon, alt_agl_m=start.alt_agl_m, yaw_rad=0.0), target
    )


def _reached(state: VehicleState, target: Waypoint) -> bool:
    dn = (target.lat - state.lat) * 111000.0
    de = (target.lon - state.lon) * 101000.0 * math.cos(math.radians(state.lat))
    return float((dn * dn + de * de) ** 0.5) < REACH_M


def _time_inside_nfz(
    ir: PolicyIR,
    traj: list[TrajPoint],
    dynamic_at: float | None,
) -> tuple[float, bool, float | None]:
    """Return (seconds_inside, ever_entered, exit_time_s_or_None).

    ``exit_time_s`` is the sim-time the vehicle first leaves all P0 polygons
    after having been inside — the recovery metric for a start-inside scenario.
    """
    polys = [p for p in ir.polygons if p.priority == "P0"]
    inside_ticks = 0
    entered = False
    exit_s = None
    been_inside = False
    for p in traj:
        if dynamic_at is not None and p["t"] < dynamic_at:
            active = [q for q in polys if q.id != "nfz-dynamic-event"]
        else:
            active = polys
        tick_inside = False
        for rec in active:
            if rec.altitude_floor_m <= p["alt"] <= rec.altitude_ceiling_m:
                if ir.signed_distance(rec, p["lat"], p["lon"]) < 0:
                    inside_ticks += 1
                    entered = True
                    tick_inside = True
                    been_inside = True
                    break
        if been_inside and not tick_inside and exit_s is None:
            exit_s = p["t"]
    return inside_ticks * DT, entered, exit_s


def _plot(
    path: Path,
    ir: PolicyIR,
    traj: list[TrajPoint],
    start: Waypoint,
    target: Waypoint,
    dynamic_at: float | None,
    shield_on: bool,
    scenario: str,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[plot] matplotlib missing — skipped")
        return

    fig, ax = plt.subplots(figsize=(7, 7))
    # NFZ polygons: the IR already stores ring vertices in local ENU metres.
    for rec in [p for p in ir.polygons if p.priority == "P0"]:
        ring = list(rec.polygon.exterior.coords)
        e_ring = [x for x, y in ring]
        n_ring = [y for x, y in ring]
        is_dyn = rec.id == "nfz-dynamic-event"
        ax.fill(
            e_ring, n_ring, alpha=0.25,
            color="purple" if is_dyn else "red",
            label=(f"dynamic NFZ (t={dynamic_at:.0f}s)" if is_dyn else f"NFZ {rec.id}"),
        )
    te = [ir.projection.to_xy(p["lat"], p["lon"])[0] for p in traj]
    tn = [ir.projection.to_xy(p["lat"], p["lon"])[1] for p in traj]
    ax.plot(te, tn, "-", color="tab:blue", linewidth=2, label="flight path")
    ti = [i for i, p in enumerate(traj) if p["intercepted"]]
    if ti:
        ax.plot([te[i] for i in ti], [tn[i] for i in ti], ".",
                color="orange", markersize=3, label="shield active")
    se = ir.projection.to_xy(start.lat, start.lon)
    tg = ir.projection.to_xy(target.lat, target.lon)
    ax.plot(se[0], se[1], "go", markersize=10, label="start")
    ax.plot(tg[0], tg[1], "k*", markersize=16, label="target")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"{scenario} — shield {'ON' if shield_on else 'OFF'}\n{len(traj)} ticks")
    ax.legend(loc="best", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"[plot] {path}")


def _report(
    path: Path,
    scenario: str,
    shield_on: bool,
    ir: PolicyIR,
    traj: list[TrajPoint],
    intercepts: int,
    nfz_seconds: float,
    reached: bool,
    dynamic_at: float | None,
    entered: bool,
    exit_s: float | None,
    instant_inside: bool,
) -> None:
    if instant_inside:
        kpi_ok = exit_s is not None
        kpi_line = f"| **Recovery KPI (exited the NFZ)** | **{'PASS' if kpi_ok else 'FAIL'}** |"
        kpi_detail = f"| Time to exit NFZ | {f'{exit_s:.1f} s' if exit_s else 'never exited'} |"
    else:
        kpi_ok = nfz_seconds == 0
        kpi_line = f"| **P0 KPI (0 NFZ entry)** | **{'PASS' if kpi_ok else 'FAIL'}** |"
        kpi_detail = f"| **Time inside NFZ** | **{nfz_seconds:.1f} s** |"
    body = f"""# Kinematic demo — scenario `{scenario}`, shield {'ON' if shield_on else 'OFF'}

| Item | Value |
|------|-------|
| Policy | `{ir.policy_id}` gen {ir.generation} `{ir.policy_hash}` |
| Ticks flown | {len(traj)} ({len(traj) * DT:.0f} s) |
| Shield intercepts | {intercepts} |
| Target reached | {'yes' if reached else 'NO'} |
| Dynamic NFZ | {f'hot-applied at t={dynamic_at:.0f}s' if dynamic_at else 'not used'} |
| Entered a P0 NFZ | {'yes' if entered else 'no'} |
{kpi_detail}
{kpi_line}
"""
    Path(path).write_text(body, encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=list(SCENARIOS), default="crossing")
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    args = ap.parse_args()

    out = ROOT / "demo" / "out" / f"{args.scenario}-{args.shield}"
    out.mkdir(parents=True, exist_ok=True)
    return run(args.scenario, args.shield == "on", out)


if __name__ == "__main__":
    sys.exit(main())
