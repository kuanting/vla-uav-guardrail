"""
AerialVLA + guardrail on PROJECT AIRSIM (PASBlocks / UE5) — e.g. JapaneseCity.

Fifth sim rail. Same guardrail, same AerialVLA backend; only the adapter edge
changes: Project AirSim's NNG pub/sub client instead of classic AirSim RPC.

Start the sim FIRST (map arg picks the world):
  & "...\PASBlocks\Binaries\Win64\Blocks-Win64-DebugGame.exe" /Game/JapaneseCity/Maps/demo -windowed

Then (vla-real env):
  python demo/aerialvla_pas_demo.py --tag aerialvla_japanesecity
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import AuditLogger, Shield, State, load_policy       # noqa: E402
from guardrail.compiler import ConstraintCompiler                   # noqa: E402
from guardrail.geometry import fence_polygon                        # noqa: E402
from guardrail.models import Action4D, PolygonFence                 # noqa: E402
from shapely.geometry import Point                                  # noqa: E402

from aerialvla_demo import AerialVLABackend, RateLimiter            # noqa: E402
import city_planner                                                 # noqa: E402

TICK = 0.1
MAX_S = 90
DEPTH_SCALE = 1.0     # probe: p50=23, p95=65504 (sky sentinel) -> values ARE meters
SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_guardrail.jsonc"


class ObsStore:
    """Thread-safe store fed by NNG camera subscriptions + the flight loop."""

    def __init__(self):
        self.lock = threading.Lock()
        self.front = None      # PIL 224x224
        self.down = None
        self.depth = None      # float32 (h, w) depth map in meters (FrontCamera)
        self.pose = (0.0, 0.0, 0.0)   # x, y, yaw

    def put_image(self, which: str, msg):
        import cv2
        from PIL import Image
        if not msg or "data" not in msg or not len(msg["data"]):
            return
        buf = np.frombuffer(msg["data"], dtype=np.uint8)
        if msg.get("encoding") == "PNG":
            arr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        else:
            arr = buf.reshape(msg["height"], msg["width"], 3)
        img = Image.fromarray(cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)).resize(
            (224, 224), resample=Image.BICUBIC)
        with self.lock:
            setattr(self, which, img)

    def put_depth(self, msg):
        # pixels-as-float depth images arrive as a raw float32 buffer (see
        # projectairsim.utils.unpack_image); decode minimally + defensively.
        self.depth_msgs = getattr(self, "depth_msgs", 0) + 1
        try:
            if not msg or "data" not in msg or not len(msg["data"]):
                self.depth_err = "empty msg / no data field"
                return
            data = msg["data"]
            raw = (np.array(data, dtype="B") if isinstance(data, list)
                   else np.frombuffer(data, dtype=np.uint8))
            h, w = msg["height"], msg["width"]
            if msg.get("encoding") == "16UC1" or raw.size == h * w * 2:
                u16 = raw.view(np.uint16).reshape(h, w)
                if not getattr(self, "_depth_probed", False):
                    self._depth_probed = True
                    nz = u16[u16 > 0]
                    if nz.size:
                        print(f"  [depth-probe] raw uint16 nonzero p5={np.percentile(nz,5):.0f} "
                              f"p50={np.percentile(nz,50):.0f} p95={np.percentile(nz,95):.0f} "
                              f"max={nz.max()}")
                arr = u16.astype(np.float32) * DEPTH_SCALE
            else:                             # float32 meters fallback
                arr = raw.view(np.float32).reshape(h, w).astype(np.float32)
        except Exception as e:
            self.depth_err = f"{type(e).__name__}: {e} keys={list(msg.keys()) if msg else None}"
            return                            # malformed frame — keep last good map
        with self.lock:
            self.depth = arr

    def get_depth(self):
        with self.lock:
            return self.depth

    def put_pose(self, x, y, yaw):
        with self.lock:
            self.pose = (x, y, yaw)

    def get_obs(self):
        with self.lock:
            if self.front is None or self.down is None:
                return None
            return self.front, self.down, self.pose


def quat_yaw(q: dict) -> float:
    w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


async def fly(args) -> int:
    from projectairsim import Drone, ProjectAirSimClient, World

    out = ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)

    policy = load_policy(args.policy)
    mission = ConstraintCompiler(policy).parse_command(args.command)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    audit = AuditLogger(out / "audit.jsonl", policy.policy_hash)

    # route: either --route "x1,y1; x2,y2; ..." (multi-waypoint patrol) or the
    # single target compiled from --command
    if args.route:
        waypoints = [tuple(float(v) for v in wp.split(","))
                     for wp in args.route.split(";") if wp.strip()]
    else:
        waypoints = [(mission.target_x, mission.target_y)]

    # ---- global planner: expand the user route into obstacle-avoiding legs ----
    # The reactive depth-avoider + Shield further down stay EXACTLY as-is (they
    # remain the final safety net); this only reshapes the *waypoint* list.
    # The planning grid = surveyed BUILDINGS (city map) + drawn/loaded NFZs from
    # the policy, so the planner routes GLOBALLY around a no-fly-zone instead of
    # leaving the Shield to slide reactively along its edge (which just wobbles).
    cmap = None if args.no_planner else city_planner.load_occ(args.citymap)
    fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
    if cmap is None and not fences:
        print("[planner] no city map, no NFZ — reactive only")
    elif args.no_planner:
        print("[planner] disabled (--no-planner) — reactive only")
    else:
        # geometry/grid params: from the city map if present, else a default
        # 80 m half-extent / 2 m grid so NFZ-only planning still works.
        if cmap is not None:
            occ, res = cmap["occ"].copy(), cmap["res"]
            ox, oy = cmap["ox"], cmap["oy"]
        else:
            res, ox, oy = 2.0, -80.0, -80.0
            occ = np.zeros((int(-2 * ox / res), int(-2 * oy / res)), dtype=np.uint8)
        # stamp every policy NFZ (inflated by its own margin) into the grid
        n_fence_cells = 0
        for f, poly in fences:
            buf = poly.buffer(f.margin_m)
            minx, miny, maxx, maxy = buf.bounds
            i0 = max(0, int((minx - ox) / res)); i1 = min(occ.shape[0], int((maxx - ox) / res) + 1)
            j0 = max(0, int((miny - oy) / res)); j1 = min(occ.shape[1], int((maxy - oy) / res) + 1)
            for i in range(i0, i1):
                for j in range(j0, j1):
                    if occ[i, j] == 0 and buf.contains(Point(ox + i * res, oy + j * res)):
                        occ[i, j] = 1
                        n_fence_cells += 1
        if fences:
            print(f"[planner] stamped {len(fences)} NFZ -> {n_fence_cells} grid cells")
        # clearance is constant, so inflate once and reuse for every leg
        grid = city_planner.inflate(occ, res, args.clearance)
        n_user = len(waypoints)
        # full ordered stop list = [spawn/current pos] + user waypoints.
        # no pre-loop pose available here, so use the spawn origin (35, -20).
        stops = [(35.0, -20.0)] + list(waypoints)
        expanded = []
        for a, b in zip(stops[:-1], stops[1:]):
            # plan() snaps a blocked goal via nearest_free; flag when it will
            if city_planner.is_blocked(grid, res, ox, oy, b[0], b[1]):
                print("[planner] wp inside building, snapped")
            sub = city_planner.plan(grid, res, ox, oy, a, b)
            if sub is None:
                print(f"[planner] leg -> {a}..{b} UNREACHABLE, flying direct")
                expanded.append(b)
            else:
                expanded.extend(sub[1:])   # drop the duplicate start point
        if expanded:                       # never collapse to an empty route
            waypoints = expanded
        print(f"[planner] {n_user} user waypoints -> {len(waypoints)} "
              f"sub-waypoints (clearance {args.clearance}m)")

    wp_i = 0
    target = waypoints[0]
    print(f"[policy] {policy.policy_id} {policy.policy_hash}")
    print(f"[task]   route={waypoints}  object={args.object!r}")

    store = ObsStore()
    vla = AerialVLABackend(args.object, target, obs_factory=lambda: store.get_obs,
                           lora_id=args.adapter)

    traj, n_touched, reached = [], 0, False    # safe defaults if flight aborts
    client = ProjectAirSimClient()
    client.connect()
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        drone = Drone(client, world, "Drone1")

        client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                         lambda _, msg: store.put_image("front", msg))
        client.subscribe(drone.sensors["DownCamera"]["scene_camera"],
                         lambda _, msg: store.put_image("down", msg))
        try:  # image-type 2 (depth perspective) — needs the capture entry in robot_guardrail_quad.jsonc
            client.subscribe(drone.sensors["FrontCamera"]["depth_camera"],
                             lambda _, msg: store.put_depth(msg))
        except Exception as e:
            print(f"[warn] no FrontCamera depth stream ({type(e).__name__}: {e}) "
                  f"— obstacle avoidance disabled")

        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        # climb to cruise altitude
        for _ in range(150):
            kin = drone.get_ground_truth_kinematics()
            up = -kin["pose"]["position"]["z"]
            if up >= mission.cruise_alt_m - 0.5:
                break
            await drone.move_by_velocity_async(0.0, 0.0, -2.0, duration=0.2)
            await asyncio.sleep(0.1)
        vla.start()
        print("[flight] cruise reached — AerialVLA flying on Project AirSim (guardrail ON)")

        limiter = RateLimiter(args.dv_h, args.dv_z)
        traj, n_touched = [], 0
        t0 = time.time()
        tick = 0
        reached = False
        flight_budget_s = MAX_S * max(1, len(waypoints))   # more time for routes
        last_pos, stall_since = None, None
        escape_until, escape_done = None, False   # ESCAPE mode (stall recovery)
        last_dlr = (100.0, 100.0)                 # freshest left/right depth
        avoid_state, prev_avoid = "", ""          # depth-avoider status
        live_path, live_tmp = out / "live.json", out / "live.json.tmp"
        while time.time() - t0 < flight_budget_s:
            tick += 1
            kin = drone.get_ground_truth_kinematics()
            p = kin["pose"]["position"]
            yaw = quat_yaw(kin["pose"]["orientation"])
            state = State(x=p["x"], y=p["y"], up=-p["z"])
            store.put_pose(p["x"], p["y"], yaw)

            escaping = escape_until is not None and time.time() < escape_until
            if escaping:
                # ESCAPE mode: move toward the FREEST direction the depth camera
                # last saw (left/right/back), + climb; still limiter+shield
                dl, dr = last_dlr
                if dl > 12.0 or dr > 12.0:
                    side = -1.0 if dl >= dr else 1.0
                    esc = yaw + side * math.pi / 2       # strafe to open side
                else:
                    esc = yaw + math.pi                   # both blocked: back out
                # climb HARD toward the ceiling — if the building is shorter
                # than the altitude band's top, going over it clears the pin
                # (shield clamps vz_up to the climb cap and alt to the ceiling)
                raw = Action4D(vx=1.2 * math.cos(esc), vy=1.2 * math.sin(esc),
                               vz_up=3.0, yaw_rate=0.0)
                avoid_state = prev_avoid = ""
            else:
                if escape_until is not None:
                    escape_until = None
                    print("  [flight] ESCAPE window over — resuming VLA control")
                fwd, dwn, yr, land = vla.latest()
                vx, vy = fwd * math.cos(yaw), fwd * math.sin(yaw)
                a = args.goal_blend
                if prev_avoid == "slow":
                    a *= 0.25                 # obstacle near: soften goal-pull
                elif prev_avoid == "climb+steer":
                    a = 0.0                   # pinned/close: goal-pull OFF so the
                                              # strafe+escape isn't fighting it
                if a > 0:
                    gd = math.hypot(target[0] - state.x, target[1] - state.y)
                    if gd > 1e-6:
                        gvx = (target[0] - state.x) / gd * mission.speed_pref_mps
                        gvy = (target[1] - state.y) / gd * mission.speed_pref_mps
                        vx, vy = (1 - a) * vx + a * gvx, (1 - a) * vy + a * gvy
                vz_up, yaw_rate = -dwn, yr * args.yaw_gain
                # depth avoider: world-frame action, AFTER goal-blend, BEFORE
                # the RateLimiter — the Shield stays the final authority
                avoid_state = ""
                depth = store.get_depth()
                if depth is not None:
                    dm = np.nan_to_num(depth, nan=100.0, posinf=100.0,
                                       neginf=100.0)
                    # <=0 invalid; <1.5 m = the drone's own props in frame edge
                    dm = np.where(dm <= 1.5, 100.0, np.minimum(dm, 100.0))
                    h, w = dm.shape
                    mid = dm[h // 4:3 * h // 4, :]   # central rows: skip sky+ground
                    # 5th percentile, not min — robust to stray pixels
                    d_center = float(np.percentile(
                        mid[:, w // 4:3 * w // 4], 5))
                    d_left = float(np.percentile(mid[:, :w // 2], 5))
                    d_right = float(np.percentile(mid[:, w // 2:], 5))
                    # HARD RULE: horizontal speed never exceeds d_center/4 —
                    # momentum can then always stop before the wall
                    vcap = max(0.4, min(4.0, d_center / 4.0))
                    hn = math.hypot(vx, vy)
                    if hn > vcap:
                        vx, vy = vx / hn * vcap, vy / hn * vcap
                    if d_center < 15.0:
                        avoid_state = "slow"
                        if d_center < 8.0:
                            avoid_state = "climb+steer"
                            vz_up += 1.2
                            # steer SIDEWAYS toward the freer side (world-frame
                            # strafe, not just yaw — actually moves off the wall)
                            side = -1.0 if d_left > d_right else 1.0
                            sx = 2.0 * math.cos(yaw + side * math.pi / 2)
                            sy = 2.0 * math.sin(yaw + side * math.pi / 2)
                            yaw_rate += side * 0.5
                            if d_center < 5.0:
                                # kill the forward component entirely: only
                                # strafe + climb until the nose sees space
                                fdot = vx * math.cos(yaw) + vy * math.sin(yaw)
                                if fdot > 0:
                                    vx -= fdot * math.cos(yaw)
                                    vy -= fdot * math.sin(yaw)
                            vx, vy = vx + sx, vy + sy
                    last_dlr = (d_left, d_right)      # escape picks its direction
                    if avoid_state != prev_avoid:
                        print(f"  [avoid] {avoid_state or 'clear'}: "
                              f"d_center={d_center:.1f}m d_left={d_left:.1f}m "
                              f"d_right={d_right:.1f}m")
                        prev_avoid = avoid_state
                # FACE-MOTION: the depth camera only protects the direction the
                # NOSE points. Rotate the nose toward the velocity vector and cap
                # speed by how far off it still is — never fly fast sideways into
                # geometry the camera can't see.
                sp = math.hypot(vx, vy)
                if sp > 0.5:
                    herr = (math.atan2(vy, vx) - yaw + math.pi) % (2 * math.pi) - math.pi
                    face_rate = float(np.clip(1.2 * herr, -0.9, 0.9))
                    yaw_rate = 0.75 * face_rate + 0.25 * yaw_rate
                    align = max(0.25, math.cos(min(abs(herr), math.pi / 2)))
                    vx, vy = vx * align, vy * align
                raw = Action4D(vx=vx, vy=vy, vz_up=vz_up, yaw_rate=yaw_rate)
            smooth = limiter(raw)
            if args.no_shield:
                # COMPARISON BASELINE: no guardrail. Emit the raw action as-is —
                # the drone WILL enter no-fly-zones / bust the altitude band.
                # (still rate-limited so the sim stays numerically stable.)
                d = SimpleNamespace(emitted=smooth, touched=False)
            else:
                d = shield.filter(state, smooth)
                audit.log(tick, d)
            if d.touched:
                n_touched += 1
            traj.append({"x": state.x, "y": state.y, "up": state.up,
                         "touched": d.touched})
            if tick % 5 == 0:
                # live status for the GUI: atomic tmp+os.replace; OneDrive can
                # throw PermissionError mid-sync — silently skip that cycle
                try:
                    live = {"tick": tick, "x": float(state.x),
                            "y": float(state.y), "up": float(state.up),
                            "wp_i": wp_i,
                            "waypoints": [[float(wx), float(wy)]
                                          for wx, wy in waypoints],
                            "touched": bool(d.touched), "avoid": avoid_state}
                    live_tmp.write_text(json.dumps(live), encoding="utf-8")
                    os.replace(live_tmp, live_path)
                except Exception:
                    pass
            e = d.emitted
            await drone.move_by_velocity_async(
                e.vx, e.vy, -e.vz_up, duration=0.3,
                yaw_is_rate=True, yaw=e.yaw_rate)
            # stuck detector: HORIZONTAL position only (x, y) — NOT altitude.
            # During an ESCAPE climb 'up' changes every tick; if it were in the
            # key the stall timer would reset forever and the drone would climb
            # to the ceiling and hover instead of skipping an unreachable point.
            pos_key = (round(state.x / 3.0), round(state.y / 3.0))  # 3 m buckets
            if pos_key == last_pos:
                stall_since = stall_since or time.time()
                stalled_s = time.time() - stall_since
                if stalled_s > 9.0:
                    print(f"  [flight] STUCK at {pos_key} even after ESCAPE — "
                          f"skipping waypoint {wp_i + 1}")
                    wp_i += 1
                    stall_since, escape_until, escape_done = None, None, False
                    if wp_i >= len(waypoints):
                        break
                    target = waypoints[wp_i]
                    vla.target_xy = target
                elif stalled_s > 6.0 and not escape_done:
                    escape_until = time.time() + 4.0
                    escape_done = True
                    print(f"  [flight] ESCAPE: stalled {stalled_s:.1f}s at {pos_key}"
                          f" — reversing heading at 1.5 m/s + climb for 4 s")
            else:
                last_pos, stall_since = pos_key, None
                escape_done = False       # moved again -> a future stall may escape

            if math.hypot(state.x - target[0], state.y - target[1]) < 3.0:
                wp_i += 1
                if wp_i >= len(waypoints):
                    reached = True
                    print(f"  [flight] final waypoint reached at tick {tick}")
                    break
                target = waypoints[wp_i]
                vla.target_xy = target          # update the VLA's direction hint
                print(f"  [flight] waypoint {wp_i}/{len(waypoints)-1} done -> next {target}")
            if tick % 50 == 0:
                dstat = (f"msgs={getattr(store, 'depth_msgs', 0)} "
                         f"err={getattr(store, 'depth_err', None)}")
                _dm = store.get_depth()
                if _dm is not None:
                    dstat = f"min={float(np.nanmin(_dm)):.1f} shape={_dm.shape}"
                print(f"  tick {tick}: pos=({state.x:5.1f},{state.y:5.1f},{state.up:4.1f}) "
                      f"cmd=({e.vx:.1f},{e.vy:.1f}) shield={'HIT' if d.touched else '-'} "
                      f"depth[{dstat}]")
            await asyncio.sleep(TICK)

        vla.stop()
        # CONTROLLED descent from ANY altitude (the drone may be at the 55 m
        # ceiling when the flight ends). Rate scales with height so it's brisk
        # up high and gentle near the ground — never a free-fall. Generous
        # iteration cap so it actually reaches the ground before land_async.
        for _ in range(600):
            kin = drone.get_ground_truth_kinematics()
            up = -kin["pose"]["position"]["z"]
            if up <= 1.2:
                break
            v_down = max(0.6, min(2.0, up * 0.15))    # 2 m/s high → 0.6 m/s low
            await drone.move_by_velocity_async(0.0, 0.0, v_down, duration=0.3)
            await asyncio.sleep(0.1)
        await (await drone.land_async())
        drone.disarm()
        drone.disable_api_control()
    except Exception as e:
        # never let an RPC hiccup (e.g. NNG timeout while wedged on a roof)
        # swallow the report/plot — the trajectory so far is still valid data
        print(f"[warn] flight aborted early: {type(e).__name__}: {e}")
    finally:
        client.disconnect()

    fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
    inside = sum(1 for pt in traj for f, poly in fences
                 if f.altitude_floor_m <= pt["up"] <= f.altitude_ceiling_m
                 and poly.contains(Point(pt["x"], pt["y"])))
    nfz_s = inside * TICK

    (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 6.5),
                                      gridspec_kw={"width_ratios": [1.1, 1]})
        for f, poly in fences:
            xs, ys = poly.exterior.xy
            ax.fill(ys, xs, alpha=0.25, color="red", label=f"NFZ {f.id}")
            bx, by = poly.buffer(f.margin_m).exterior.xy
            ax.plot(by, bx, "--", color="red", linewidth=1, alpha=0.6)
        ax.plot([pt["y"] for pt in traj], [pt["x"] for pt in traj], "-",
                color="tab:blue", linewidth=2, label="flight path")
        tx = [pt["y"] for pt in traj if pt["touched"]]
        ty = [pt["x"] for pt in traj if pt["touched"]]
        if tx:
            ax.plot(tx, ty, ".", color="orange", markersize=6, label="shield active")
        ax.plot(traj[0]["y"], traj[0]["x"], "go", markersize=10, label="start")
        for wi, wp in enumerate(waypoints):
            ax.plot(wp[1], wp[0], "k*", markersize=16,
                    label="waypoints" if wi == 0 else None)
            ax.annotate(str(wi + 1), (wp[1], wp[0]), textcoords="offset points",
                        xytext=(8, 8), fontsize=10, fontweight="bold")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        guard = "GUARDRAIL OFF" if args.no_shield else "guardrail ON"
        ax.set_title(f"AerialVLA — {guard} — PROJECT AIRSIM ({args.map_label})\n"
                     f"NFZ {nfz_s:.1f}s "
                     f"{'VIOLATED' if (args.no_shield and nfz_s > 0) else ('PASS' if nfz_s==0 else 'FAIL')}"
                     f" | interventions {n_touched} | "
                     f"{'REACHED' if reached else 'not reached'}")
        ax.legend(loc="upper left", fontsize=9)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)

        ts = [i * TICK for i in range(len(traj))]
        ax2.plot(ts, [pt["up"] for pt in traj], color="tab:blue", label="altitude")
        from guardrail.models import AltitudeEnvelope
        for env in policy.by_type(AltitudeEnvelope):
            ax2.axhspan(env.alt_min_m, env.alt_max_m, alpha=0.12, color="green",
                        label=f"allowed band [{env.alt_min_m:.0f},{env.alt_max_m:.0f}]m")
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("altitude (m)")
        ax2.set_title("Altitude vs time")
        ax2.legend(fontsize=9)
        ax2.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / "trajectory.png", dpi=130)
        print(f"[plot] {out / 'trajectory.png'}")
    except ImportError:
        print("[plot] matplotlib missing — skipped")

    guard = "GUARDRAIL OFF" if args.no_shield else "guardrail ON"
    verdict = ("VIOLATED" if (args.no_shield and nfz_s > 0)
               else ("PASS" if nfz_s == 0 else "FAIL"))
    print(f"\n[report] AerialVLA on Project AirSim | {guard} | ticks {len(traj)} | "
          f"shield interventions {n_touched} | NFZ {nfz_s:.1f}s -> {verdict} | "
          f"target {'REACHED' if reached else 'not reached'}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--command", default="fly to (40, 40) at 6 m/s altitude 20")
    ap.add_argument("--object", default="an open area near the target point")
    ap.add_argument("--policy", default=str(ROOT / "policies" / "urban_demo_policy.yaml"))
    ap.add_argument("--tag", default="aerialvla_japanesecity")
    ap.add_argument("--map-label", default="JapaneseCity")
    ap.add_argument("--dv-h", type=float, default=0.25)
    ap.add_argument("--dv-z", type=float, default=0.15)
    ap.add_argument("--yaw-gain", type=float, default=1.0)
    ap.add_argument("--goal-blend", type=float, default=0.0)
    ap.add_argument("--best", action="store_true",
                    help="load tuned params from models/aerialvla_adapter_best.json")
    ap.add_argument("--adapter", default="D:/models/aerialvla-lora/aero_vla",
                    help="LoRA adapter dir (use the fine-tuned run2/epoch1)")
    ap.add_argument("--route", default="",
                    help='multi-waypoint route "x1,y1; x2,y2; ..." (overrides --command target)')
    ap.add_argument("--citymap",
                    default="D:/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/demo/out/citymap/occ.npz",
                    help="city occupancy map .npz for the global planner")
    ap.add_argument("--clearance", type=float, default=6.0,
                    help="building clearance (m) the global planner inflates by")
    ap.add_argument("--no-planner", action="store_true",
                    help="disable global planning (reactive depth avoider only)")
    ap.add_argument("--no-shield", action="store_true",
                    help="COMPARISON: bypass the Safety Shield entirely (raw VLA "
                         "action, will violate NFZ/altitude) — for before/after demos")
    args = ap.parse_args()
    if args.best:
        import json as _json
        bp = ROOT / "models" / "aerialvla_adapter_best.json"
        p = _json.loads(bp.read_text(encoding="utf-8"))["params"]
        args.yaw_gain, args.goal_blend = p["yaw_gain"], p["goal_blend"]
        args.dv_h, args.dv_z = p["dv_h"], p["dv_z"]
        print(f"[tuned] using best adapter params: {p}")
    return asyncio.run(fly(args))


if __name__ == "__main__":
    sys.exit(main())
