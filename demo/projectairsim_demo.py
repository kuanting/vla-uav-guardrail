"""Project AirSim visual demo — the Prof-Lai SafetyShield drives the UE5 drone.

This is the perception rail from docs/03-simulation/dual-rail.md: the Shield
flies Project AirSim's UE5 Neighborhood world directly via its velocity API, so
the result is *watchable* (a real drone in a real city). The Shield core is the
production ``safety_shield`` package; only the bottom adapter differs — here it
is the Project AirSim async client instead of pymavlink/MAVROS.

Pipeline (all grant boxes present, perception-rail topology):

    policy bundle -> PolicyIR
    VLA stub (reckless straight-at-target) -> Action4D (body frame)
    SafetyShield.tick() -> repaired Action4D
    Project AirSim adapter (body -> NED) -> move_by_velocity_async -> UE5 drone

Run (Project AirSim Neighborhood world running, ports 8989/8990 up):

    python -m demo.projectairsim_demo --shield off   # the villain
    python -m demo.projectairsim_demo --shield on    # the hero

Outputs per run under episodes/<tag>/: trajectory.png, report.md,
shield_audit.jsonl (shield on only).
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in ("vlaguard-common", "policy-dsl", "safety-shield"):
    sys.path.insert(0, str(ROOT / "packages" / _p / "src"))
sys.path.insert(0, str(ROOT))

from policy_dsl import ingest_file  # noqa: E402
from projectairsim import Drone, ProjectAirSimClient, World  # noqa: E402
from safety_shield import AuditLog, SafetyShield, VehicleState  # noqa: E402
from vlaguard_common import Action4D  # noqa: E402

TICK = 0.1            # 10 Hz — the grant's action/monitor rate
MAX_S = 60            # mission time cap
REACH_M = 3.0
CRUISE_ALT = 8.0      # AGL, inside the 2-120 m envelope
SCENE = "scene_basic_drone.jsonc"
# The World() client resolves SCENE relative to a sim_config/ dir. Run from the
# repo root, so point it at the examples dir that ships the scene/robot configs.
SIM_CONFIG_DIR = str(Path("D:/ProjectAirSim/repo/client/python/example_user_scripts/sim_config"))

# Mission in the IR's local ENU plane (metres from the projection origin lat/lon 0).
# Target 50 m north; the straight path clips the NFZ (15-35 m north).
TARGET_NORTH_M = 50.0
TARGET_EAST_M = 0.0


def vla_action(north_m: float, east_m: float, alt_m: float) -> tuple[Action4D, float]:
    """Reckless straight-line pilot. Returns (body Action4D, heading_rad NED).

    Body vx = forward speed toward the target, vy = 0 (body points at target).
    heading is the NED yaw (0=North, CW+) to the target.
    """
    dn = TARGET_NORTH_M - north_m
    de = TARGET_EAST_M - east_m
    dist = math.hypot(dn, de)
    if dist < 1e-6:
        return Action4D(), 0.0
    speed = min(4.0, dist)
    vz = max(-2.0, min(2.0, 0.8 * (CRUISE_ALT - alt_m)))
    yaw = math.atan2(de, dn)  # NED yaw: atan2(east, north)
    return Action4D(vx=speed, vy=0.0, vz=vz, yaw_rate=0.0), yaw


def kinematics_to_local(kin: dict) -> tuple[float, float, float]:
    """Project AirSim ground-truth kinematics -> (north_m, east_m, alt_up_m).

    PAS kinematics nests position under ``pose.position`` as NED
    (x=North, y=East, z=Down). We return up-positive altitude and north/east
    metres for the Shield's local-plane queries.
    """
    pos = kin["pose"]["position"]  # NED: x=North, y=East, z=Down
    return pos["x"], pos["y"], -pos["z"]


async def run(shield_on: bool, out: Path, policy_path: str) -> int:
    ir = ingest_file(policy_path)
    print(f"[policy]   {ir.policy_id} gen {ir.generation} {ir.policy_hash}")
    print(f"           {len(ir.polygons)} polygon(s), {len(ir.envelopes)} envelope(s)")

    audit = (
        AuditLog(out / "shield_audit.jsonl", ir.policy_hash, ir.generation) if shield_on else None
    )
    shield = SafetyShield(ir, audit=audit)

    print("[airsim]   connecting to Project AirSim (ports 8989/8990) ...")
    client = ProjectAirSimClient()
    client.connect()
    world = World(client, SCENE, delay_after_load_sec=2, sim_config_path=SIM_CONFIG_DIR)
    drone = Drone(client, world, "Drone1")
    print("[airsim]   world loaded, drone acquired")

    drone.enable_api_control()
    drone.arm()
    print("[airsim]   takeoff ...")
    await drone.takeoff_async()
    # climb to cruise altitude (move up = negative v_down in NED)
    t0 = time.time()
    while time.time() - t0 < 20:
        kin = drone.get_ground_truth_kinematics()
        _n, _e, up = kinematics_to_local(kin)
        if up >= CRUISE_ALT - 0.5:
            break
        await drone.move_by_velocity_async(0.0, 0.0, -1.5, duration=0.5)
        await asyncio.sleep(0.4)
    cruise_up = kinematics_to_local(drone.get_ground_truth_kinematics())[2]
    print(f"[airsim]   at cruise altitude {cruise_up:.1f} m")

    traj = []
    intercepts = 0
    reached = False
    t0 = time.time()
    while time.time() - t0 < MAX_S:
        now = time.time() - t0
        kin = drone.get_ground_truth_kinematics()
        north_m, east_m, alt_m = kinematics_to_local(kin)

        if math.hypot(TARGET_NORTH_M - north_m, TARGET_EAST_M - east_m) < REACH_M:
            reached = True
            print(f"[flight]   target reached at t={now:.1f}s")
            break

        raw, yaw = vla_action(north_m, east_m, alt_m)
        # VehicleState needs lat/lon for the Shield's IR geometry queries. Use the
        # IR's OWN projection (to_latlon(east, north)) so the drone is in exactly
        # the same local plane as the NFZ polygons — regardless of where the IR
        # anchored its projection origin (it picks the first polygon's centroid).
        vlat, vlon = ir.projection.to_latlon(east_m, north_m)
        vstate = VehicleState(lat=vlat, lon=vlon, alt_agl_m=alt_m, yaw_rad=yaw)

        if shield_on:
            decision = shield.tick(vstate, raw, ts=f"t+{now:.1f}s")
            emitted = decision.emitted_action
            intercepted = decision.intercepted
            if intercepted:
                intercepts += 1
        else:
            emitted = raw
            intercepted = False

        # body Action4D -> NED world velocity (vz is up-positive -> negate for down)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        v_north = emitted.vx * cos_y - emitted.vy * sin_y
        v_east = emitted.vx * sin_y + emitted.vy * cos_y
        v_down = -emitted.vz
        await drone.move_by_velocity_async(v_north, v_east, v_down, duration=TICK)
        traj.append({"t": round(now, 2), "north": round(north_m, 2), "east": round(east_m, 2),
                     "alt": round(alt_m, 2), "intercepted": intercepted})
        await asyncio.sleep(TICK)

    print("[airsim]   landing ...")
    await drone.land_async()
    await asyncio.sleep(4)
    drone.disarm()
    drone.disable_api_control()
    client.disconnect()
    if audit is not None:
        audit.close()

    # KPI: time inside any P0 polygon (local-plane containment)
    polys = [p for p in ir.polygons if p.priority == "P0"]
    inside_ticks = 0
    for p in traj:
        for rec in polys:
            if rec.altitude_floor_m <= p["alt"] <= rec.altitude_ceiling_m:
                # build a shapely point in the IR's local (east, north) plane
                from shapely.geometry import Point
                # IR polygons are in (east_m, north_m); the projection origin is
                # (0,0), so to_xy(lat,lon) ~ (east, north). Containment check:
                pt = Point(p["east"], p["north"])
                if rec.polygon.contains(pt):
                    inside_ticks += 1
                    break
    nfz_seconds = inside_ticks * TICK
    _plot(out / "trajectory.png", ir, traj, shield_on)
    _report(out / "report.md", shield_on, ir, len(traj), intercepts, nfz_seconds, reached)
    print(f"[report]   NFZ time {nfz_seconds:.1f}s -> KPI {'PASS' if nfz_seconds == 0 else 'FAIL'} "
          f"| intercepts {intercepts} | reached {reached}")
    return 0 if nfz_seconds == 0 else 1


def _plot(path: Path, ir, traj, shield_on: bool) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[plot] matplotlib missing — skipped")
        return
    fig, ax = plt.subplots(figsize=(7, 7))
    for rec in [p for p in ir.polygons if p.priority == "P0"]:
        ring = list(rec.polygon.exterior.coords)
        ax.fill([x for x, y in ring], [y for x, y in ring], alpha=0.25,
                color="red", label=f"NFZ {rec.id}")
    # IR plane is (east, north); plot east on X, north on Y for a map view
    ax.plot([p["east"] for p in traj], [p["north"] for p in traj], "-",
            color="tab:blue", linewidth=2, label="flight path")
    ti = [i for i, p in enumerate(traj) if p["intercepted"]]
    if ti:
        ax.plot([traj[i]["east"] for i in ti], [traj[i]["north"] for i in ti], ".",
                color="orange", markersize=3, label="shield active")
    ax.plot(0, 0, "go", markersize=10, label="start")
    ax.plot(TARGET_EAST_M, TARGET_NORTH_M, "k*", markersize=16, label="target")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"Project AirSim — shield {'ON' if shield_on else 'OFF'}")
    ax.legend(loc="best", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"[plot]     {path}")


def _report(path: Path, shield_on: bool, ir, nticks: int, intercepts: int,
            nfz_seconds: float, reached: bool) -> None:
    kpi_ok = nfz_seconds == 0
    body = f"""# Project AirSim demo — shield {'ON' if shield_on else 'OFF'}

| Item | Value |
|------|-------|
| Policy | `{ir.policy_id}` gen {ir.generation} `{ir.policy_hash}` |
| Ticks flown | {nticks} ({nticks * TICK:.0f} s) |
| Shield intercepts | {intercepts} |
| Target reached | {'yes' if reached else 'NO'} |
| **Time inside NFZ** | **{nfz_seconds:.1f} s** |
| **P0 KPI (0 NFZ entry)** | **{'PASS' if kpi_ok else 'FAIL'}** |
"""
    path.write_text(body, encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--policy", default=str(ROOT / "bundles" / "projectairsim-demo.yaml"))
    ap.add_argument("--tag", default=None)
    args = ap.parse_args()

    tag = args.tag or f"airsim_shield_{args.shield}"
    out = ROOT / "episodes" / tag
    out.mkdir(parents=True, exist_ok=True)
    return asyncio.run(run(args.shield == "on", out, args.policy))


if __name__ == "__main__":
    sys.exit(main())
