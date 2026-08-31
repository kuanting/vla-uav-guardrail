"""
VLA Drone Guardrail — end-to-end demo runner.

Full meeting-architecture pipeline, live in AirSim:

    User Command -> Constraint Compiler -> YAML Prompt -> (Stub) VLA
                 -> Safety Shield -> velocity command -> AirSim drone

Usage (conda env: airsim, Blocks sim running):

    python demo/run_demo.py --shield on
    python demo/run_demo.py --shield off        # A/B: watch the NFZ get violated
    python demo/run_demo.py --command "fly to the north pad" --shield on

Outputs per run, under demo/out/<tag>/:
    prompt.yaml        the compiled YAML prompt the VLA received
    trajectory.png     top-down plot: NFZ, path, start/target
    report.md          KPI summary (NFZ entry seconds, shield stats)
    audit.jsonl        Shield audit log (only when shield=on)
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import airsim

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import AuditLogger, Shield, State, load_policy          # noqa: E402
from guardrail.compiler import ConstraintCompiler                      # noqa: E402
from guardrail.geometry import fence_polygon                           # noqa: E402
from guardrail.models import PolygonFence, XY                          # noqa: E402
from guardrail.vla_stub import StubVLA                                 # noqa: E402

from shapely.geometry import Point                                     # noqa: E402

TICK = 0.1          # 10 Hz — the grant's action rate
MAX_S = 90          # mission time cap
REACH_M = 2.0
FRAME_EVERY = 30    # save a camera frame every N ticks (~3 s)


class RateLimiter:
    """Smooth the EMITTED velocity: cap per-tick change so the autopilot is
    not whipsawed by the VLA-vs-Shield tug-of-war at zone edges (visible as
    wobble + lift-loss dips). Never applied to the raw VLA action — and the
    caller re-checks the smoothed action against the shield, so smoothing can
    never smuggle a violation through."""

    def __init__(self, dv_h: float = 0.45, dv_z: float = 0.25):
        self.dv_h = dv_h            # max horizontal delta per tick (m/s)
        self.dv_z = dv_z            # max vertical delta per tick (m/s)
        self.prev = None

    @staticmethod
    def _clip(new: float, old: float, lim: float) -> float:
        return old + max(-lim, min(lim, new - old))

    def smooth(self, a):
        if self.prev is None:
            self.prev = a
            return a
        from guardrail.models import Action4D
        sm = Action4D(
            vx=self._clip(a.vx, self.prev.vx, self.dv_h),
            vy=self._clip(a.vy, self.prev.vy, self.dv_h),
            vz_up=self._clip(a.vz_up, self.prev.vz_up, self.dv_z),
            yaw_rate=a.yaw_rate,
        )
        self.prev = sm
        return sm

# The dynamic NFZ spawned by --dynamic: sits on the east-edge detour path the
# drone takes around the static zone, so it forces a SECOND, wider reroute.
DYNAMIC_AT_S = 8.0
DYNAMIC_FENCE = PolygonFence(
    id="nfz-dynamic", type="polygon_fence",
    vertices=[XY(x=23, y=6), XY(x=31, y=6), XY(x=31, y=14), XY(x=23, y=14)],
    margin_m=1.0,
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--command", default="fly to the northeast pad at 6 m/s")
    ap.add_argument("--policy", default=str(ROOT / "policies" / "sim_demo_policy.yaml"))
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--dynamic", action="store_true",
                    help=f"hot-apply a dynamic NFZ at t={DYNAMIC_AT_S:.0f}s (grant: dynamic_nfz)")
    ap.add_argument("--no-smooth", action="store_true",
                    help="disable the emitted-action rate limiter (raw jitter)")
    ap.add_argument("--frame-every", type=int, default=FRAME_EVERY,
                    help="camera frame every N ticks (2 = 5 Hz, for video)")
    ap.add_argument("--dv-h", type=float, default=0.25,
                    help="max horizontal velocity change per tick, m/s (smaller = smoother, slower to react)")
    ap.add_argument("--dv-z", type=float, default=0.15,
                    help="max vertical velocity change per tick, m/s")
    ap.add_argument("--api", action="store_true",
                    help="serve the hot-apply REST API on 127.0.0.1:8071 during the mission")
    ap.add_argument("--vla", choices=["stub", "bc", "v3"], default="stub",
                    help="pilot in the VLA slot: stub, old BC model, or the "
                         "gate-passing auto-research policy (v3)")
    ap.add_argument("--tag", default=None, help="output folder name")
    args = ap.parse_args()

    # Accept --policy relative to the current dir OR the project root, so the
    # script works no matter which folder it is launched from.
    if not Path(args.policy).exists():
        alt = ROOT / args.policy
        if alt.exists():
            args.policy = str(alt)
        else:
            sys.exit(f"policy file not found: {args.policy!r} "
                     f"(also tried {alt}) — available: "
                     f"{[p.name for p in (ROOT / 'policies').glob('*.yaml')]}")

    shield_on = args.shield == "on"
    tag = args.tag or (f"shield_{args.shield}" + ("_dynamic" if args.dynamic else ""))
    out = ROOT / "demo" / "out" / tag
    out.mkdir(parents=True, exist_ok=True)

    # ---- 1. Policy + Compiler (before-VLA half) ----
    policy = load_policy(args.policy)
    compiler = ConstraintCompiler(policy)
    mission = compiler.parse_command(args.command)
    prompt = compiler.build_prompt(mission)
    (out / "prompt.yaml").write_text(prompt, encoding="utf-8")
    print(f"[compiler] {args.command!r} -> target=({mission.target_x:.0f},"
          f"{mission.target_y:.0f}) alt={mission.cruise_alt_m:.0f}m "
          f"speed_pref={mission.speed_pref_mps:.0f}m/s")
    print(f"[policy]   {policy.policy_id}  {policy.policy_hash}")

    # ---- 2. VLA slot + Shield (after-VLA half) ----
    if args.vla == "bc":
        from guardrail.vla_bc import BCVLA
        vla = BCVLA(mission, policy)
        print("[vla]      learned BC policy in the slot (models/bc_policy.pt)")
    elif args.vla == "v3":
        from guardrail.vla_bc import BCVLAv3
        vla = BCVLAv3(mission, policy)
        print("[vla]      gate-passing policy in the slot (models/vla_policy_v2.pt)")
    else:
        vla = StubVLA(mission)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    audit = AuditLogger(out / "audit.jsonl", policy)   # the POLICY, so a hot-applied rule restamps the hash

    if args.api:
        from guardrail.api import serve_in_background
        serve_in_background(shield)
        print("[api]      hot-apply API on http://127.0.0.1:8071 "
              "(POST /nfz, GET /policy, GET /health)")

    # ---- 3. Fly ----
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    for _ in range(150):                       # verified climb to cruise alt
        up_now = -client.simGetVehiclePose().position.z_val
        if up_now >= mission.cruise_alt_m - 0.5:
            break
        client.moveByVelocityAsync(0, 0, -2.0, duration=0.2)
        time.sleep(0.1)
    print(f"[flight]   cruise altitude {-client.simGetVehiclePose().position.z_val:.1f} m, "
          f"shield={'ON' if shield_on else 'OFF'}")

    frames_dir = out / "frames"
    frames_dir.mkdir(exist_ok=True)

    limiter = RateLimiter(dv_h=args.dv_h, dv_z=args.dv_z)
    jitter_raw = jitter_out = 0.0        # sum |Δv| between consecutive commands
    jit_max_raw = jit_max_out = 0.0      # worst single-tick whipsaw
    prev_cmd = None

    traj: list[dict] = []
    n_touched = n_braked = n_frames = 0
    spawn_t = None                              # sim-time the dynamic NFZ went live
    spawn_pos = None
    t0 = time.time()
    tick = 0
    reached = False
    while time.time() - t0 < MAX_S:
        tick += 1
        now = time.time() - t0
        pos = client.simGetVehiclePose().position
        state = State(x=pos.x_val, y=pos.y_val, up=-pos.z_val)

        # --- dynamic NFZ hot-apply (grant: dynamic_nfz + generation bump) ---
        if args.dynamic and spawn_t is None and now >= DYNAMIC_AT_S:
            shield.hot_apply(DYNAMIC_FENCE)
            if hasattr(vla, "refresh_fences"):
                vla.refresh_fences(policy)             # BC model sees the new zone too
            audit.policy_hash = policy.policy_hash     # records now carry the new hash
            spawn_t, spawn_pos = now, (state.x, state.y)
            print(f"[dynamic]  t={now:.1f}s NFZ '{DYNAMIC_FENCE.id}' hot-applied "
                  f"-> generation {policy.generation}, {policy.policy_hash}")

        # --- camera frame: what a real VLA would see ---
        if tick % args.frame_every == 1:
            png = client.simGetImage("0", airsim.ImageType.Scene)
            if png:
                (frames_dir / f"tick_{tick:04d}.png").write_bytes(png)
                n_frames += 1

        dist = ((mission.target_x - state.x) ** 2 + (mission.target_y - state.y) ** 2) ** 0.5
        if dist < REACH_M:
            reached = True
            print(f"[flight]   target reached at tick {tick}")
            break

        raw = vla.act(state)
        if shield_on:
            decision = shield.filter(state, raw)
            audit.policy_hash = policy.policy_hash   # API may have hot-applied
            audit.log(tick, decision)
            emitted = decision.emitted
            if decision.touched:
                n_touched += 1
                n_braked += int(decision.braked)
        else:
            emitted = raw                       # A/B: no protection

        # --- jerk shaping: smooth the safe action, then RE-CHECK it. If the
        # smoothed version would violate (e.g. delaying a brake), fall back to
        # the shield's exact output — safety always wins over comfort.
        final = emitted
        if not args.no_smooth:
            sm = limiter.smooth(emitted)
            if shield_on and shield._check(state, sm):
                limiter.prev = emitted           # resync; use the strict action
            else:
                final = sm
        if prev_cmd is not None:
            dr = abs(emitted.vx - prev_cmd[0]) + abs(emitted.vy - prev_cmd[1])
            do = abs(final.vx - prev_cmd[2]) + abs(final.vy - prev_cmd[3])
            jitter_raw += dr
            jitter_out += do
            jit_max_raw = max(jit_max_raw, dr)
            jit_max_out = max(jit_max_out, do)
        prev_cmd = (emitted.vx, emitted.vy, final.vx, final.vy)

        traj.append({"t": round(now, 2), "x": state.x, "y": state.y,
                     "up": state.up,
                     "touched": shield_on and decision.touched})
        client.moveByVelocityAsync(final.vx, final.vy, -final.vz_up,
                                   duration=TICK * 2)
        time.sleep(TICK)

    # ---- gentle landing: slow controlled descent, camera keeps rolling.
    # landAsync drops fast and reads as "falling" on video; this descends at
    # 0.7 m/s, easing to 0.35 m/s for the final meters, then lets landAsync
    # do only the last 30 cm touchdown.
    print("[flight]   gentle descent...")
    t_land = time.time()
    while time.time() - t_land < 60:
        tick += 1
        up_now = -client.simGetVehiclePose().position.z_val
        if up_now < 0.3:
            break
        v_down = 0.7 if up_now > 4.0 else 0.35
        client.moveByVelocityAsync(0, 0, v_down, duration=TICK * 2)  # NED: +z = down
        if tick % args.frame_every == 1:
            png = client.simGetImage("0", airsim.ImageType.Scene)
            if png:
                (frames_dir / f"tick_{tick:04d}.png").write_bytes(png)
                n_frames += 1
        time.sleep(TICK)
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("[flight]   landed")

    # ---- 4. KPI: seconds spent inside a raw NFZ polygon ----
    # A dynamic fence only counts AFTER it went live (fair accounting).
    fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]

    def _active(f, p) -> bool:
        if f.id == DYNAMIC_FENCE.id:
            return spawn_t is not None and p["t"] >= spawn_t
        return True

    inside_ticks = sum(
        1 for p in traj
        for f, poly in fences
        if _active(f, p)
        and f.altitude_floor_m <= p["up"] <= f.altitude_ceiling_m
        and poly.contains(Point(p["x"], p["y"]))
    )
    nfz_seconds = inside_ticks * TICK

    # ---- 5. Plot ----
    plot_path = out / "trajectory.png"
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(7, 7))
        for f, poly in fences:
            dyn = f.id == DYNAMIC_FENCE.id
            color = "purple" if dyn else "red"
            label = (f"dynamic NFZ (t={spawn_t:.0f}s)" if dyn else f"NFZ {f.id}")
            xs, ys = poly.exterior.xy
            ax.fill(ys, xs, alpha=0.25, color=color, label=label)
            bx, by = poly.buffer(f.margin_m).exterior.xy
            ax.plot(by, bx, "--", color=color, linewidth=1, alpha=0.6)
        if spawn_pos is not None:
            ax.plot(spawn_pos[1], spawn_pos[0], "X", color="purple", markersize=12,
                    label="drone @ hot-apply")
        xs = [p["y"] for p in traj]                     # plot as East-vs-North (map view)
        ys = [p["x"] for p in traj]
        ax.plot(xs, ys, "-", color="tab:blue", linewidth=2, label="flight path")
        tx = [p["y"] for p in traj if p["touched"]]
        ty = [p["x"] for p in traj if p["touched"]]
        if tx:
            ax.plot(tx, ty, ".", color="orange", markersize=4, label="shield active")
        ax.plot(traj[0]["y"], traj[0]["x"], "go", markersize=10, label="start")
        ax.plot(mission.target_y, mission.target_x, "k*", markersize=16, label="target")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_title(f"Guardrail demo — shield {'ON' if shield_on else 'OFF'}\n"
                     f"NFZ time: {nfz_seconds:.1f}s | "
                     f"{'reached' if reached else 'NOT reached'}")
        ax.legend(loc="upper left", fontsize=9)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(plot_path, dpi=130)
        print(f"[plot]     {plot_path}")
    except ImportError:
        print("[plot]     matplotlib missing — skipped")

    # ---- 6. Report ----
    kpi_ok = nfz_seconds == 0
    report = f"""# Guardrail demo report — shield {'ON' if shield_on else 'OFF'}

| Item | Value |
|------|-------|
| Command | `{args.command}` |
| Policy | `{policy.policy_id}` `{policy.policy_hash}` |
| Target | ({mission.target_x:.0f}, {mission.target_y:.0f}) @ {mission.cruise_alt_m:.0f} m |
| Target reached | {'yes' if reached else 'NO'} |
| Ticks flown | {len(traj)} ({len(traj) * TICK:.0f} s) |
| Shield interventions | {n_touched} ticks |
| Brakes | {n_braked} |
| Dynamic NFZ | {f'hot-applied at t={spawn_t:.1f}s -> generation {policy.generation}' if spawn_t else 'not used'} |
| Camera frames captured | {n_frames} (`frames/`) |
| **Time inside NFZ** | **{nfz_seconds:.1f} s** |
| **P0 KPI (0 NFZ entry)** | **{'PASS' if kpi_ok else 'FAIL'}** |
"""
    (out / "report.md").write_text(report, encoding="utf-8")
    (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")
    n_cmd = max(len(traj) - 1, 1)
    print(f"[report]   NFZ time {nfz_seconds:.1f}s -> KPI {'PASS' if kpi_ok else 'FAIL'} "
          f"| shield touched {n_touched} | braked {n_braked}")
    print(f"[smooth]   cmd jitter |dv|/tick: mean {jitter_raw / n_cmd:.2f} -> "
          f"{jitter_out / n_cmd:.2f} m/s, worst {jit_max_raw:.2f} -> {jit_max_out:.2f} m/s "
          f"({'smoothing OFF' if args.no_smooth else 'rate-limited'})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
