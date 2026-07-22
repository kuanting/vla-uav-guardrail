"""SITL flown A/B demo — the Prof-Lai SafetyShield, through a REAL autopilot.

This is the dev-topology functional rail, minus MAVROS: pymavlink talks directly
to ArduPilot SITL (the known-good path on this host — MAVROS's FCU handshake is
flakey here). The Shield core is the production ``safety_shield`` package; only
the bottom adapter (body-frame action -> MAVLink SET_POSITION_TARGET_LOCAL_NED)
is pymavlink instead of a ROS topic.

Pipeline (grant architecture, all boxes present):

    policy bundle --load--> PolicyIR (policy_dsl)
    VLA stub (reckless straight-at-target) --> Action4D (body frame)
    SafetyShield.tick() --> repaired Action4D
    MAVLink adapter (body -> NED) --> ArduPilot SITL (real flight code) -> copter

Run inside WSL (SITL listening on tcp:127.0.0.1:5760):

    ~/venv-ap/bin/python -m demo.sitl_flight_demo --shield off   # the villain
    ~/venv-ap/bin/python -m demo.sitl_flight_demo --shield on    # the hero
    ~/venv-ap/bin/python -m demo.sitl_flight_demo --shield on --dynamic

Outputs per run under episodes/<tag>/: trajectory.png, report.md,
shield_audit.jsonl (shield on only).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in ("vlaguard-common", "policy-dsl", "safety-shield"):
    sys.path.insert(0, str(ROOT / "packages" / _p / "src"))
sys.path.insert(0, str(ROOT))

from policy_dsl import ingest_file  # noqa: E402
from pymavlink import mavutil  # noqa: E402
from safety_shield import AuditLog, SafetyShield, VehicleState  # noqa: E402
from vlaguard_common import Action4D  # noqa: E402

# type_mask: use velocity + yaw_rate, ignore position/accel/force/yaw.
# bits(1=ignore): x y z | vx vy vz | ax ay az | force yaw yaw_rate
VEL_YAWRATE_MASK = 0b0101_1100_0111  # 1479

TICK = 0.1            # 10 Hz — the grant's action/monitor rate
MAX_S = 90            # mission time cap
REACH_M = 3.0

# Mission: home is the SITL default (-35.363261,149.165230). Target ~40 m north;
# the straight-line path clips the NFZ block (~15-33 m north).
TARGET_LAT = -35.36270
TARGET_LON = 149.165230
CRUISE_ALT = 12.0     # AGL, well inside the 3-120 m envelope

# Dynamic NFZ hot-applied at t=8s on the east detour around the static zone.
DYNAMIC_AT_S = 8.0


class MavlinkAdapter:
    """The single body/up-positive <-> NED boundary (grant rule). pymavlink.

    Mirrors the in-house prototype's adapter, proven on this SITL. The Shield
    emits body-frame (vx forward, vy right, vz up-positive, yaw_rate rad/s);
    this rotates to local-NED using the current heading and sends the velocity
    setpoint. Body->NED conversion lives HERE, never in the Shield.
    """

    def __init__(self, url: str):
        print(f"[mavlink]  connecting {url} ...")
        self.m = mavutil.mavlink_connection(url)
        self.m.wait_heartbeat()
        print(f"[mavlink]  heartbeat from sys {self.m.target_system}")
        # Without a GCS attached, SITL streams nothing by default — ask for
        # everything at 10 Hz or state() would block forever.
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    def _ack(self, cmd: int, timeout: float = 3.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
            if msg and msg.command == cmd:
                return msg.result == 0
        return False

    def prepare(self, alt_m: float) -> None:
        """State-machine bring-up: verify GUIDED via heartbeat, keep (re)arming,
        retry takeoff — whatever the EKF timing, converge or time out."""
        mav = self.m.mav
        mode_id = self.m.mode_mapping()["GUIDED"]
        deadline = time.time() + 120
        while time.time() < deadline:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
            if hb is None:
                continue
            if hb.custom_mode != mode_id:
                self.m.set_mode(mode_id)
                time.sleep(1)
                continue
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                mav.command_long_send(self.m.target_system, self.m.target_component,
                                      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                      0, 1, 0, 0, 0, 0, 0, 0)
                time.sleep(2)
                continue
            mav.command_long_send(self.m.target_system, self.m.target_component,
                                  mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                  0, 0, 0, 0, 0, 0, 0, alt_m)
            if self._ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
                print(f"[mavlink]  armed + takeoff accepted, climbing to {alt_m} m ...")
                break
            time.sleep(2)
        else:
            raise RuntimeError("bring-up timed out (mode/arm/takeoff)")

        t0 = time.time()
        last_up = 0.0
        while time.time() - t0 < 60:
            st = self.state()
            if st is not None:
                last_up = st.up
                if st.up >= alt_m - 1.0:
                    print(f"[mavlink]  at cruise altitude {st.up:.1f} m")
                    return
        raise RuntimeError(f"takeoff did not reach altitude (last {last_up:.1f} m)")

    def state(self):
        msg = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        if msg is None:
            return None
        # LOCAL_POSITION_NED has no yaw field; the VLA here commands body-frame
        # vx/vy directly toward the target, so we don't need the vehicle heading
        # (the adapter derives the rotation from the velocity direction). yaw=0.
        return State(x=msg.x, y=msg.y, up=-msg.z, yaw=0.0)

    def send(self, action: Action4D, yaw_rad: float) -> None:
        """body-frame action -> local-NED velocity setpoint.

        The Shield emits body-frame (vx forward, vy right, vz up-positive,
        yaw_rate rad/s). We rotate to local-NED using the vehicle's current
        heading (NED: 0=North, CW positive). Body->NED conversion lives HERE —
        the single boundary, per the grant rule.
        """
        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
        v_north = action.vx * cos_y - action.vy * sin_y
        v_east = action.vx * sin_y + action.vy * cos_y
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, VEL_YAWRATE_MASK,
            0, 0, 0,                       # position (ignored)
            v_north, v_east, -action.vz,   # velocity, NED (z-down)
            0, 0, 0,                       # accel (ignored)
            0,                             # yaw (ignored by mask)
            action.yaw_rate)               # yaw_rate

    def land(self) -> None:
        self.m.set_mode(self.m.mode_mapping()["LAND"])
        print("[mavlink]  LAND")


class State:
    __slots__ = ("x", "y", "up", "yaw")
    def __init__(self, x: float, y: float, up: float, yaw: float):
        self.x = x
        self.y = y
        self.up = up
        self.yaw = yaw


def vla_action(local_pos, target_local, cruise: float):
    """Reckless straight-line pilot. Returns (body Action4D, heading_rad).

    The vehicle is assumed to point at the target, so body vx = forward speed,
    vy = 0. The heading (target bearing, NED yaw) is returned separately so the
    caller can set VehicleState.yaw_rad for the Shield and rotate to NED.
    """
    dx = target_local[0] - local_pos[0]
    dy = target_local[1] - local_pos[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return Action4D(), 0.0
    heading = math.atan2(dy, dx)  # ENU-plane bearing -> NED yaw (East=+x here)
    speed = min(5.0, dist)
    vz = max(-2.0, min(2.0, 0.8 * (cruise - local_pos[2])))
    return Action4D(vx=speed, vy=0.0, vz=vz, yaw_rate=0.0), heading


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--dynamic", action="store_true")
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--policy", default=str(ROOT / "bundles" / "sitl-flight-demo.yaml"))
    ap.add_argument("--tag", default=None)
    args = ap.parse_args()

    shield_on = args.shield == "on"
    tag = args.tag or (f"sitl_shield_{args.shield}" + ("_dynamic" if args.dynamic else ""))
    out = ROOT / "episodes" / tag
    out.mkdir(parents=True, exist_ok=True)

    # --- 1. Load policy bundle -> PolicyIR ---
    ir = ingest_file(args.policy)
    print(f"[policy]   {ir.policy_id} gen {ir.generation} {ir.policy_hash}")
    print(f"           {len(ir.polygons)} polygon(s), {len(ir.envelopes)} envelope(s)")

    audit = (
        AuditLog(out / "shield_audit.jsonl", ir.policy_hash, ir.generation) if shield_on else None
    )
    shield = SafetyShield(ir, audit=audit)

    # home + target in the IR's local ENU plane (metres from the projection origin,
    # which is the first polygon's centroid)
    home_lat, home_lon = -35.363261, 149.165230
    home_xy = ir.projection.to_xy(home_lat, home_lon)
    target_xy = ir.projection.to_xy(TARGET_LAT, TARGET_LON)
    print(f"[mission]  home ENU={home_xy} target ENU={target_xy} cruise={CRUISE_ALT}m")

    # --- 2. Connect + arm + takeoff (blocking) ---
    mav = MavlinkAdapter(args.url)
    mav.prepare(CRUISE_ALT)

    # --- 3. Fly ---
    traj = []
    intercepts = 0
    dynamic_at = None
    reached = False
    t0 = time.time()
    tick = 0
    while time.time() - t0 < MAX_S:
        tick += 1
        now = time.time() - t0
        st = mav.state()
        if st is None:
            continue
        # LOCAL_POSITION_NED: x = North, y = East (both metres from home). The
        # IR's local plane is (east, north), so swap into it.
        pos_local = (home_xy[0] + st.y, home_xy[1] + st.x, st.up)

        # dynamic NFZ hot-apply at t=DYNAMIC_AT_S (on the east detour path)
        if args.dynamic and dynamic_at is None and now >= DYNAMIC_AT_S:
            from policy_dsl.models import LatLon, PolygonFence, PolygonGeometry
            # ~10 m east of home, blocking the east edge detour
            fence = PolygonFence(
                id="nfz-dynamic", type="polygon_fence", constraint_type="hard",
                scope="global", priority="P0", layer="site", violation_action="project_fix",
                geometry=PolygonGeometry(
                    vertices=[
                        LatLon(lat=home_lat - 0.00005, lon=home_lon + 0.00010),
                        LatLon(lat=home_lat + 0.00015, lon=home_lon + 0.00010),
                        LatLon(lat=home_lat + 0.00015, lon=home_lon + 0.00025),
                        LatLon(lat=home_lat - 0.00005, lon=home_lon + 0.00025),
                    ],
                    altitude_floor_m=0, altitude_ceiling_m=200, altitude_ref="AGL"))
            from policy_dsl.ir import PolygonRecord
            from shapely.geometry import Polygon
            from vlaguard_common import policy_hash
            ring = [ir.projection.to_xy(v.lat, v.lon) for v in fence.geometry.vertices]
            ir.polygons.append(PolygonRecord(
                id=fence.id, constraint_type=fence.constraint_type, priority=fence.priority,
                violation_action=fence.violation_action,
                altitude_floor_m=fence.geometry.altitude_floor_m,
                altitude_ceiling_m=fence.geometry.altitude_ceiling_m, polygon=Polygon(ring)))
            ir.__post_init__()
            ir.generation += 1
            ir.policy_hash = policy_hash(ir.canonical)
            if audit is not None:
                audit.generation = ir.generation
                audit.policy_hash = ir.policy_hash
            dynamic_at = now
            print(f"[dynamic]  t={now:.1f}s NFZ hot-applied -> gen {ir.generation}")

        # vehicle state in lat/lon for the Shield's IR geometry queries
        vlat, vlon = ir.projection.to_latlon(pos_local[0], pos_local[1])

        dist = math.dist(pos_local[:2], target_xy)
        if dist < REACH_M:
            reached = True
            print(f"[flight]   target reached at tick {tick}")
            break

        raw, _heading = vla_action(pos_local, (*target_xy, CRUISE_ALT), CRUISE_ALT)
        # NED yaw (0=North, CW positive). pos_local is in the IR's local plane:
        # x = East, y = North (to_xy returns (east_m, north_m)). So the bearing
        # to the target in NED yaw is atan2(dEast, dNorth) = atan2(dx, dy).
        dx = target_xy[0] - pos_local[0]
        dy = target_xy[1] - pos_local[1]
        yaw_rad = math.atan2(dx, dy)
        vstate = VehicleState(lat=vlat, lon=vlon, alt_agl_m=st.up, yaw_rad=yaw_rad)
        if shield_on:
            decision = shield.tick(vstate, raw, ts=f"t+{now:.1f}s")
            emitted = decision.emitted_action
            if decision.intercepted:
                intercepts += 1
        else:
            emitted = raw

        mav.send(emitted, yaw_rad)
        traj.append({"t": round(now, 2), "lat": vlat, "lon": vlon, "alt": round(st.up, 2),
                     "intercepted": shield_on and decision.intercepted})

    # --- 4. Land ---
    mav.land()
    time.sleep(5)
    if audit is not None:
        audit.close()

    # --- 5. KPI: time inside any active P0 polygon (dynamic counts after apply) ---
    polys = [p for p in ir.polygons if p.priority == "P0"]
    inside_ticks = 0
    for p in traj:
        # the dynamic NFZ only counts after it was applied (fair accounting)
        active = [
            q for q in polys
            if not (q.id == "nfz-dynamic" and dynamic_at and p["t"] < dynamic_at)
        ]
        for rec in active:
            if rec.altitude_floor_m <= p["alt"] <= rec.altitude_ceiling_m:
                if ir.signed_distance(rec, p["lat"], p["lon"]) < 0:
                    inside_ticks += 1
                    break
    nfz_seconds = inside_ticks * TICK
    _plot(out / "trajectory.png", ir, traj, home_xy, target_xy, dynamic_at, shield_on)
    _report(out / "report.md", shield_on, ir, len(traj), intercepts, nfz_seconds,
            reached, dynamic_at)
    print(f"[report]   NFZ time {nfz_seconds:.1f}s -> KPI {'PASS' if nfz_seconds==0 else 'FAIL'} "
          f"| intercepts {intercepts} | reached {reached}")
    return 0 if nfz_seconds == 0 else 1


def _plot(path, ir, traj, home_xy, target_xy, dynamic_at, shield_on):
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
        is_dyn = rec.id == "nfz-dynamic"
        ax.fill([x for x, y in ring], [y for x, y in ring], alpha=0.25,
                color="purple" if is_dyn else "red",
                label=("dynamic NFZ" if is_dyn else f"NFZ {rec.id}"))
    xs = [ir.projection.to_xy(p["lat"], p["lon"])[0] for p in traj]
    ys = [ir.projection.to_xy(p["lat"], p["lon"])[1] for p in traj]
    ax.plot(xs, ys, "-", color="tab:blue", linewidth=2, label="flight path")
    ti = [i for i, p in enumerate(traj) if p["intercepted"]]
    if ti:
        ax.plot([xs[i] for i in ti], [ys[i] for i in ti], ".",
                color="orange", markersize=3, label="shield active")
    ax.plot(home_xy[0], home_xy[1], "go", markersize=10, label="home")
    ax.plot(target_xy[0], target_xy[1], "k*", markersize=16, label="target")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"SITL flight — shield {'ON' if shield_on else 'OFF'}")
    ax.legend(loc="best", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"[plot]     {path}")


def _report(path, shield_on, ir, nticks, intercepts, nfz_seconds, reached, dynamic_at):
    kpi_ok = nfz_seconds == 0
    body = f"""# SITL flight demo — shield {'ON' if shield_on else 'OFF'}

| Item | Value |
|------|-------|
| Policy | `{ir.policy_id}` gen {ir.generation} `{ir.policy_hash}` |
| Ticks flown | {nticks} ({nticks * TICK:.0f} s) |
| Shield intercepts | {intercepts} |
| Target reached | {'yes' if reached else 'NO'} |
| Dynamic NFZ | {f'hot-applied at t={dynamic_at:.0f}s' if dynamic_at else 'not used'} |
| **Time inside NFZ** | **{nfz_seconds:.1f} s** |
| **P0 KPI (0 NFZ entry)** | **{'PASS' if kpi_ok else 'FAIL'}** |
"""
    Path(path).write_text(body, encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())
