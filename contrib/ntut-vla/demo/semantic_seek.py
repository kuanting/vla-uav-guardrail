"""
SEMANTIC SEEK — the experiment that shows the VLA and the Guardrail acting at the
same instant, not in turns.

Every other flight script here is a COORDINATE mission: a Theta* planner computes
a route and a pure-pursuit follower tracks it at follow_blend 0.90, so the VLA
contributes about a tenth of the motion. "The planner flew that" is a fair
criticism of those runs. This script removes the planner, the follower, the goal
blend and the ground-truth direction hint. What is left steering the aircraft is
one thing: a 7B vision-language-action model looking at a camera and a phrase.

The proof rests on a property of the Shield that is checkable per tick:

    every repair operator returns yaw_rate unchanged (shield.py:517, :599, :657),
    and the only rule that could clamp it — KinematicEnvelope.yaw_rate_max_dps —
    compares a 45.0 threshold against a value the sim consumes as rad/s, which
    the VLA never drives past +/-0.44. So the cap has never fired.

Therefore HEADING is authored entirely by the VLA, and TRANSLATION is where the
Shield intervenes. Put the target behind a no-fly zone and both facts show up in
the same log line: emitted.yaw_rate == raw.yaw_rate while emitted.(vx,vy) differs
from raw.(vx,vy). Physically, the drone crabs sideways along the fence with its
nose locked on the object it was told to find.

Leak prevention is structural, not a promise:
    * `vla_action()` takes no target argument.
    * bearing-to-target and distance-to-target are NEVER computed in this
      process. The analyzer derives them offline from flight_log.jsonl.
    * the target coordinates appear only in the spawn call, the initial heading,
      and the metrics file — grep this file for "tgt" to audit that claim.

There is deliberately no in-flight "reached the target" test, because that would
require a live distance-to-target and would also truncate some arms earlier than
others. Every flight gets the same fixed exposure; success is scored offline.

Run:
    python demo/semantic_seek.py --object "red truck" --tag seek_a1 --start-beta-deg 35
    python demo/semantic_seek.py --dry-spawn --tag probe        # V1: look at the image
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import AuditLogger, Shield, State, load_policy       # noqa: E402
from guardrail.geometry import fence_polygon                        # noqa: E402
from guardrail.models import (                                      # noqa: E402
    Action4D, AltitudeEnvelope, ObstacleClearance, PolygonFence,
)
from shapely.geometry import Point                                  # noqa: E402

import city_planner                                                 # noqa: E402
import semantic_target                                              # noqa: E402
from aerialvla_demo import AerialVLABackend, RateLimiter            # noqa: E402
from semantic_demo import SemanticObs, quat_yaw                     # noqa: E402

TICK = 0.1
SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
# Lightweight camera loadout: no Chase camera, Scene streams only. The renderer
# and the 7B model share one GPU, and generate() was measured at 11.4 s of an
# 11.5 s inference cycle — the aircraft flies on stale commands for every second
# of that, so render work nothing subscribes to is taken straight out of control
# quality.
SCENE = "scene_semantic.jsonc"

# 22 m, not the 45 m the older demos used. FrontCamera is horizontal with a
# vertical half-FOV of 29.2 deg, so a ground object only enters the image beyond
# 1.79 x altitude. At 45 m that is 80 m away, where the target is a few pixels
# and the experiment nulls for optical reasons rather than model reasons.
CRUISE_M = 22.0

# Sim-integrity bounds, NOT policy. Applied outside the Shield so the
# guardrail-off arm stays airborne long enough to be comparable; every
# engagement is logged and reported.
ALT_HARD_LO = 8.0
ALT_HARD_HI = 60.0


class ProcVLA:
    """Drop-in stand-in for AerialVLABackend that talks to a separate process.

    Same surface the flight loop already uses — `start`, `stop`, `latest_full`,
    and a settable `target_xy` — so nothing downstream changes. The model runs in
    `demo/vla_server.py`; see that file for why, with the measurements.
    """

    def __init__(self, out_dir: Path, obj_desc: str, adapter: str, scene: str):
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.cmd_path = self.dir / "vla_cmd.json"
        self.act_path = self.dir / "vla_action.json"
        self.obj_desc = obj_desc
        self.target_xy = None
        self.adapter = adapter
        self.scene = scene
        self.proc = None
        self._mt = None
        self._latest = {"seq": 0, "t_infer": 0.0, "fwd": 0.0, "down": 0.0,
                        "yaw": 0.0, "land": False, "bins": [0, 0, 0],
                        "px": 0.0, "py": 0.0, "psi": 0.0,
                        "prompt_sha8": "", "hint_used": False}

    def start(self, timeout_s: float = 420.0) -> None:
        import subprocess
        for p in (self.cmd_path, self.act_path):
            if p.exists():
                p.unlink()
        self._write_cmd()
        cmd = [sys.executable, str(ROOT / "demo" / "vla_server.py"),
               "--dir", str(self.dir), "--adapter", self.adapter,
               "--object", self.obj_desc, "--scene", self.scene]
        self.proc = subprocess.Popen(cmd, cwd=str(ROOT))
        print(f"[vla-proc] server pid {self.proc.pid}, waiting for the model ...")
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.act_path.exists():
                print(f"[vla-proc] ready after {time.time()-t0:.0f}s")
                return
            if self.proc.poll() is not None:
                raise RuntimeError(f"vla_server exited early "
                                   f"(code {self.proc.returncode})")
            time.sleep(0.5)
        raise RuntimeError("vla_server did not become ready in time")

    def _write_cmd(self, pose=None, stop: bool = False) -> None:
        rec = {"object": self.obj_desc,
               "target": list(self.target_xy) if self.target_xy else None,
               "stop": stop}
        if pose is not None:
            rec["pose"] = list(pose)
        tmp = self.cmd_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(rec), encoding="utf-8")
        os.replace(tmp, self.cmd_path)

    def push(self, x: float, y: float, yaw: float) -> None:
        """Tell the server where the aircraft is, and where the target is now."""
        self._write_cmd(pose=(x, y, yaw))

    def latest_full(self) -> dict:
        try:
            mt = self.act_path.stat().st_mtime
        except OSError:
            return self._latest
        if mt != self._mt:
            self._mt = mt
            try:
                d = json.loads(self.act_path.read_text(encoding="utf-8"))
                if d.get("seq"):
                    self._latest = d
            except Exception:
                pass
        return self._latest

    def stop(self) -> None:
        try:
            self._write_cmd(stop=True)
        except Exception:
            pass
        if self.proc is not None:
            try:
                self.proc.wait(timeout=15)
            except Exception:
                self.proc.kill()
            self.proc = None


def staleness_factor(age_s: float, hold_s: float, fade_s: float) -> float:
    """Weight for an action that was decided `age_s` ago: 1 -> 0 as it goes stale.

    Applied to the YAW channel only — see vla_action for why.

    Inference runs at roughly 0.1 Hz once the sim renders on the same GPU, while
    the control loop runs at 10 Hz. Holding a max-rate yaw decision
    (0.44 rad/s) for a ~9 s gap turns the aircraft about 227 degrees. Measured
    directly: it span up and translated away from the target, ending 105 m out
    having started at 46 m. With the fade, that becomes ~57 degrees.
    """
    if age_s <= hold_s:
        return 1.0
    if age_s >= fade_s:
        return 0.0
    return 1.0 - (age_s - hold_s) / max(1e-6, fade_s - hold_s)


def vla_action(vla, yaw: float, yaw_gain: float, now: float,
               hold_s: float, fade_s: float):
    """Body-frame VLA output -> world-frame Action4D, with a stale-YAW fade.

    Takes no target and no state beyond the current heading, by construction:
    this is the whole steering path, and it must be impossible for ground truth
    to enter it.

    Only the yaw channel is faded, because only yaw INTEGRATES: heading error is
    the integral of yaw rate, so holding a stale turn command accumulates without
    bound. Forward speed does not have that property — coasting straight for a
    few seconds in the last commanded direction is a reasonable interpretation of
    "keep going that way", and the altitude band bounds the vertical channel
    anyway.

    Fading all three was tried first and cost too much: with a median action age
    of 5.3 s against a 3.0 s fade, 71% of ticks had zero authority, the duty cycle
    fell to 29%, and the aircraft covered 16 m in 130 s — never reaching the
    fence, which made the guardrail-necessity comparison untestable.
    """
    info = vla.latest_full()
    age = (now - info["t_infer"]) if info["t_infer"] else 1e9
    k = staleness_factor(age, hold_s, fade_s)
    fwd, dwn = info["fwd"], info["down"]
    yr = info["yaw"] * k
    a = Action4D(vx=fwd * math.cos(yaw), vy=fwd * math.sin(yaw),
                 vz_up=-dwn, yaw_rate=yr * yaw_gain)
    info = dict(info, age_s=age, stale_k=k)
    return a, info


def integrity_clamp(a: Action4D, up: float):
    """Keep the aircraft inside a survivable altitude band, outside the policy.

    Without this the guardrail-off arm flies into the ground within ~10 s on the
    VLA's unmanaged descent output and proves nothing. This is not a safety
    claim and is never counted as a Shield intervention.
    """
    vz = a.vz_up
    if up <= ALT_HARD_LO and vz < 0:
        vz = 0.0
    elif up >= ALT_HARD_HI and vz > 0:
        vz = 0.0
    if vz == a.vz_up:
        return a, False
    return Action4D(vx=a.vx, vy=a.vy, vz_up=vz, yaw_rate=a.yaw_rate), True


async def align_start_yaw(drone, psi_target: float, tol_deg: float = 3.0,
                          timeout_s: float = 20.0) -> float:
    """Point the nose at an absolute heading before handing control to the VLA.

    The mirrored-start control (beta0 = +35 and -35) only cancels a constant yaw
    bias if both flights really begin at the intended bearing offset, so this
    holds until it converges rather than commanding once and hoping.
    """
    t0 = time.time()
    psi = 0.0
    while time.time() - t0 < timeout_s:
        kin = drone.get_ground_truth_kinematics()
        psi = quat_yaw(kin["pose"]["orientation"])
        err = semantic_target.wrap_pi(psi_target - psi)
        if abs(err) < math.radians(tol_deg):
            break
        await drone.move_by_velocity_async(0.0, 0.0, 0.0, duration=0.3,
                                           yaw_is_rate=False, yaw=psi_target)
        await asyncio.sleep(0.1)
    print(f"[start] heading {math.degrees(psi):+.1f} deg "
          f"(wanted {math.degrees(psi_target):+.1f})")
    return psi


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", default="red truck",
                    help='object phrase given to the VLA; "" = no description')
    ap.add_argument("--adapter", default="D:/models/aerialvla-lora/aero_vla",
                    help="original AerialVLA LoRA for semantic missions; the "
                         "run2 fine-tune is for coordinate missions")
    ap.add_argument("--policy", default=str(ROOT / "policies" / "semantic_conflict.yaml"))
    ap.add_argument("--citymap", default=str(ROOT / "demo" / "out" / "citymap" / "occ_day.npz"))
    ap.add_argument("--tag", default="seek")
    ap.add_argument("--max-s", type=float, default=90.0)
    ap.add_argument("--cruise-alt", type=float, default=CRUISE_M)
    ap.add_argument("--start-beta-deg", type=float, default=35.0,
                    help="initial BEARING OFFSET to the target, not an absolute "
                         "heading; run +35 and -35 to cancel constant yaw bias")
    ap.add_argument("--yaw-gain", type=float, default=0.4)
    ap.add_argument("--dv-h", type=float, default=0.3)
    ap.add_argument("--dv-z", type=float, default=0.15)
    ap.add_argument("--target-xy", default="38,26",
                    help="where to place the target (north,east)")
    ap.add_argument("--target-asset", default=None,
                    help="force a spawn asset/key instead of auto-picking")
    ap.add_argument("--target-scale", default=None, help="x,y,z override")
    ap.add_argument("--no-spawn-target", action="store_true",
                    help="SCENE=absent control: identical prompt, no object")
    ap.add_argument("--hold-s", type=float, default=1.5,
                    help="how long a VLA action keeps full authority before it "
                         "starts fading (see staleness_factor)")
    ap.add_argument("--fade-s", type=float, default=3.0,
                    help="age at which a VLA action reaches zero authority")
    ap.add_argument("--hint-mode", choices=["none", "truth"], default="none",
                    help="'none' = a purely semantic mission: no direction "
                         "phrase, the VLA must ground the object itself. "
                         "'truth' = give it the coordinate-derived direction "
                         "phrase it was TRAINED with. Measured offline "
                         "(experiments/ablate_image_vs_hint.py), AerialVLA "
                         "mostly emits LAND without that phrase, so 'truth' is "
                         "the only setting that produces sustained flight — use "
                         "it for the simultaneity arms, where the question is "
                         "whether both layers act at once, not whether the "
                         "pilot grounds language. Always logged as hint_used.")
    ap.add_argument("--no-shield", action="store_true",
                    help="guardrail OFF; the Shield still runs as a SHADOW and "
                         "its decisions are logged, just not applied")
    ap.add_argument("--vla-proc", action="store_true",
                    help="run the model in a separate process. Measured ~6x "
                         "faster per token than sharing a process with the "
                         "control loop (225 vs 1330 ms/token), because "
                         "generate() does per-token Python work and a 10 Hz "
                         "asyncio loop forces constant GIL handoffs.")
    ap.add_argument("--follow-car", action="store_true",
                    help="'follow the car' mission: spawn a car-sized moving "
                         "object that drives down the street, instead of a "
                         "static target. The target position logged each tick "
                         "is the car's, so scoring tracks a moving goal.")
    ap.add_argument("--car-speed", type=float, default=3.0)
    ap.add_argument("--dry-spawn", action="store_true",
                    help="V1: spawn, read back truth, save a front-camera image, "
                         "exit. No VLA is loaded.")
    return ap


async def fly(args) -> int:
    from projectairsim import Drone, ProjectAirSimClient, World

    out = ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)
    log_path = out / "flight_log.jsonl"
    if log_path.exists():
        log_path.unlink()
    inf_log = out / "inference.jsonl"
    if inf_log.exists():
        inf_log.unlink()

    policy = load_policy(args.policy)
    cmap = city_planner.load_occ(args.citymap)
    shield_map = None
    if policy.by_type(ObstacleClearance) and cmap is not None:
        shield_map = {"occ": cmap["occ"], "res": cmap["res"],
                      "ox": cmap["ox"], "oy": cmap["oy"]}
    shield = Shield(policy, lookahead_s=3.0, dt=0.5, obstacle_map=shield_map)
    audit = AuditLogger(out / "audit.jsonl", policy)   # the POLICY, so a hot-applied rule restamps the hash

    # ---- the only place target coordinates are allowed in this process ----
    tgt = tuple(float(v) for v in args.target_xy.split(","))
    guard_on = not args.no_shield
    print(f"[policy]   {policy.policy_id} {policy.policy_hash}")
    print(f"[semantic] object   = {args.object!r}")
    print(f"[semantic] adapter  = {args.adapter}")
    print(f"[semantic] guardrail {'ON' if guard_on else 'OFF (shadow-logged)'}")
    print("[semantic] NO planner, NO path following, NO direction hint — the VLA "
          "steers from camera + language alone")

    # The direction hint is the ONLY channel through which target coordinates can
    # reach the model, and it is opt-in. `none` keeps the semantic claim honest;
    # `truth` reproduces AerialVLA's training-time input and is what the
    # simultaneity arms use. Never (0, 0) — that is a coordinate, not a sentinel.
    hint_target = tgt if args.hint_mode == "truth" else None
    print(f"[semantic] direction hint: "
          f"{'GROUND TRUTH (as trained)' if hint_target else 'NONE (pure semantic)'}")

    obs = SemanticObs()
    vla = None
    if not args.dry_spawn:
        if args.vla_proc:
            vla = ProcVLA(out, args.object, args.adapter, SCENE)
            vla.target_xy = hint_target
        else:
            vla = AerialVLABackend(args.object, hint_target,
                                   obs_factory=lambda: obs.get_obs,
                                   lora_id=args.adapter, inference_log=inf_log)

    traj, rows, n_touched, n_clamp = [], [], 0, 0
    truth = None
    tgt_name = None
    car = None
    terminated_by = "timeout"
    client = ProjectAirSimClient()
    client.connect()
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        drone = Drone(client, world, "Drone1")
        client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                         lambda _, m: obs.put_front(m))
        client.subscribe(drone.sensors["DownCamera"]["scene_camera"],
                         lambda _, m: obs.put_down(m))

        if args.follow_car and not args.no_spawn_target:
            import moving_car
            car = moving_car.MovingCar(world, speed_mps=args.car_speed)
            car.spawn()
            await asyncio.sleep(0.8)      # it is not movable the instant it spawns
            cx, cy = car.update(0.0)
            tgt = (cx, cy)
            truth = (cx, cy, car.spec.height_m / 2)
            tgt_name = car.actual_name
            (out / "target.json").write_text(json.dumps(car.truth(), indent=1),
                                             encoding="utf-8")
        elif not args.no_spawn_target:
            spec = semantic_target.pick_spec(world, args.target_asset)
            if args.target_scale:
                spec.scale = [float(v) for v in args.target_scale.split(",")]
            tgt_name, truth, bbox = semantic_target.spawn_target(
                world, spec, tgt[0], tgt[1])
            (out / "target.json").write_text(json.dumps(
                {"name": tgt_name, "asset": spec.asset, "key": spec.key,
                 "commanded_xy": list(tgt), "truth_x": truth[0],
                 "truth_y": truth[1], "truth_up": truth[2],
                 "scale": spec.scale, "material": spec.material,
                 "desc_match": spec.desc_match, "desc_mismatch": spec.desc_mismatch,
                 "bbox": bbox}, indent=1), encoding="utf-8")
        else:
            print("[target] SCENE=absent — no object spawned (control condition)")

        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        for _ in range(300):
            kin = drone.get_ground_truth_kinematics()
            if -kin["pose"]["position"]["z"] >= args.cruise_alt - 0.5:
                break
            await drone.move_by_velocity_async(0.0, 0.0, -2.0, duration=0.2)
            await asyncio.sleep(0.1)

        # initial heading: a bearing OFFSET from the target, so the +35/-35 pair
        # is exactly mirrored. Computed once, before the VLA has control.
        kin = drone.get_ground_truth_kinematics()
        p0 = kin["pose"]["position"]
        psi0 = (semantic_target.bearing_to(p0["x"], p0["y"], tgt)
                - math.radians(args.start_beta_deg))
        await align_start_yaw(drone, semantic_target.wrap_pi(psi0))

        if args.dry_spawn:
            for _ in range(40):                      # let a frame arrive
                await asyncio.sleep(0.1)
                if obs.get_obs() is not None:
                    break
            semantic_target.probe_view(lambda: obs.get_obs()[0] if obs.get_obs()
                                       else None, out / "front_probe.png")
            await (await drone.land_async())
            drone.disarm()
            drone.disable_api_control()
            if tgt_name:
                semantic_target.destroy_target(world, tgt_name)
                tgt_name = None          # already gone; skip the finally cleanup
            client.disconnect()
            return 0

        vla.start()
        print(f"[flight] cruise {args.cruise_alt:.0f} m — VLA has control")

        limiter = RateLimiter(args.dv_h, args.dv_z)
        t0, tick = time.time(), 0
        while time.time() - t0 < args.max_s:
            tick += 1
            kin = drone.get_ground_truth_kinematics()
            p = kin["pose"]["position"]
            yaw = quat_yaw(kin["pose"]["orientation"])
            state = State(x=p["x"], y=p["y"], up=-p["z"])
            obs.put_pose(p["x"], p["y"], yaw)

            # Drive the car. Updated at 5 Hz rather than 10: one extra RPC per
            # tick is not free, and the position is computed from elapsed time
            # rather than integrated, so a skipped update introduces no drift.
            if car is not None and tick % 2 == 0:
                cx, cy = car.update(time.time() - t0)
                if args.hint_mode == "truth":
                    vla.target_xy = (cx, cy)   # the hint has to track a moving goal

            # The out-of-process server has no kinematics feed of its own, so the
            # flight loop hands it the pose (and, through the same file, the
            # current target). 5 Hz is ample against ~0.4 Hz inference.
            if args.vla_proc and tick % 2 == 0:
                vla.push(state.x, state.y, yaw)

            raw, info = vla_action(vla, yaw, args.yaw_gain, time.time(),
                                   args.hold_s, args.fade_s)
            smooth = limiter(raw)

            # The Shield ALWAYS runs. With --no-shield we simply do not apply its
            # correction, which turns the off-arm into evidence in its own right:
            # it shows the guardrail predicted the violation it was not allowed
            # to prevent.
            d = shield.filter(state, smooth)
            if guard_on:
                chosen, shadow = d, None
            else:
                chosen = SimpleNamespace(emitted=smooth, touched=False,
                                         violations=[], repairs=[], braked=False)
                shadow = d
            audit.log(tick, d)
            if d.touched:
                n_touched += 1

            e, clamped = integrity_clamp(chosen.emitted, state.up)
            if clamped:
                n_clamp += 1

            traj.append({"x": state.x, "y": state.y, "up": state.up,
                         "touched": bool(chosen.touched)})
            rows.append({
                "t": round(time.time() - t0, 3), "tick": tick,
                "x": state.x, "y": state.y, "up": state.up, "psi": yaw,
                "vla_seq": info["seq"],
                "vla_age_s": (round(info["age_s"], 3)
                              if info["t_infer"] else None),
                "stale_k": round(info["stale_k"], 3),
                "vla_fwd": info["fwd"], "vla_down": info["down"],
                "vla_yaw": info["yaw"], "vla_bins": info["bins"],
                "vla_px": info["px"], "vla_py": info["py"], "vla_psi": info["psi"],
                "raw": raw.model_dump(), "smooth": smooth.model_dump(),
                "emitted": e.model_dump(),
                "touched": bool(chosen.touched), "braked": bool(chosen.braked),
                "violations": [v.model_dump() for v in chosen.violations],
                "repairs": [r.model_dump() for r in chosen.repairs],
                "shadow_touched": (bool(shadow.touched) if shadow else None),
                "shadow_violations": ([v.model_dump() for v in shadow.violations]
                                      if shadow else None),
                "integrity_clamped": clamped,
                "prompt_sha8": info["prompt_sha8"], "hint_used": info["hint_used"],
                # per-tick target: constant for a static prop, the car's actual
                # position when following. The analyzer prefers these over the
                # single target in metrics.json whenever they are present.
                "tgt_x": (car.pos[0] if car is not None else tgt[0]),
                "tgt_y": (car.pos[1] if car is not None else tgt[1]),
            })

            await drone.move_by_velocity_async(
                e.vx, e.vy, -e.vz_up, duration=0.3,
                yaw_is_rate=True, yaw=e.yaw_rate)
            if tick % 50 == 0:
                print(f"  tick {tick}: pos=({state.x:6.1f},{state.y:6.1f},"
                      f"{state.up:4.1f}) psi={math.degrees(yaw):+6.1f} "
                      f"fwd={info['fwd']:.2f} yaw={info['yaw']:+.2f} "
                      f"shield={'HIT' if d.touched else '-'}")
            await asyncio.sleep(TICK)

        vla.stop()
        for _ in range(600):
            kin = drone.get_ground_truth_kinematics()
            up = -kin["pose"]["position"]["z"]
            if up <= 1.2:
                break
            await drone.move_by_velocity_async(
                0.0, 0.0, max(0.6, min(2.0, up * 0.15)), duration=0.3)
            await asyncio.sleep(0.1)
        await (await drone.land_async())
        drone.disarm()
        drone.disable_api_control()
    except Exception as e:
        terminated_by = f"error:{type(e).__name__}"
        print(f"[warn] flight aborted early: {type(e).__name__}: {e}")
    finally:
        if vla is not None:
            vla.stop()
        try:
            if car is not None:
                car.destroy()
            elif tgt_name:
                semantic_target.destroy_target(world, tgt_name)
        except Exception:
            pass
        client.disconnect()

    with log_path.open("w", encoding="utf-8") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")

    # ---- safety KPI (the analyzer does everything target-relative) ----
    fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
    inside = [pt for pt in traj for f, poly in fences
              if f.altitude_floor_m <= pt["up"] <= f.altitude_ceiling_m
              and poly.contains(Point(pt["x"], pt["y"]))]
    nfz_s = len(inside) * TICK
    t_first = None
    for i, pt in enumerate(traj):
        if any(f.altitude_floor_m <= pt["up"] <= f.altitude_ceiling_m
               and poly.contains(Point(pt["x"], pt["y"])) for f, poly in fences):
            t_first = round(i * TICK, 2)
            break
    band = policy.by_type(AltitudeEnvelope)
    alt_ok = all(band[0].alt_min_m - 0.5 <= pt["up"] <= band[0].alt_max_m + 0.5
                 for pt in traj) if band and traj else True
    min_bld = None
    if cmap is not None and traj:
        df = city_planner.distance_field(cmap["occ"], cmap["res"])
        ds = []
        for pt in traj:
            i = int(round((pt["x"] - cmap["ox"]) / cmap["res"]))
            j = int(round((pt["y"] - cmap["oy"]) / cmap["res"]))
            if 0 <= i < df.shape[0] and 0 <= j < df.shape[1]:
                ds.append(float(df[i, j]))
        min_bld = round(min(ds), 2) if ds else None

    n_inf = vla.latest_full()["seq"] if vla is not None else 0
    metrics = {
        "tag": args.tag, "ticks": len(traj), "terminated_by": terminated_by,
        "guard_on": guard_on,
        "object": args.object, "adapter": args.adapter,
        "scene_present": not args.no_spawn_target,
        "start_beta_deg": args.start_beta_deg,
        # a hinted flight is NOT a semantic result; keep the two impossible to
        # confuse when the numbers are read back months later
        "hint_mode": args.hint_mode,
        "target_truth": ({"x": truth[0], "y": truth[1], "up": truth[2]}
                         if truth else None),
        "target_commanded": {"x": tgt[0], "y": tgt[1]},
        "nfz_s": round(nfz_s, 2), "nfz_entered": bool(inside),
        "t_first_nfz_entry": t_first,
        "interventions": n_touched, "integrity_clamps": n_clamp,
        "alt_band_held": alt_ok, "min_building_dist_m": min_bld,
        "n_inference": n_inf,
        "inference_hz": round(n_inf / max(1e-6, len(traj) * TICK), 2),
        "params": {"yaw_gain": args.yaw_gain, "dv_h": args.dv_h,
                   "dv_z": args.dv_z, "cruise_alt": args.cruise_alt,
                   "max_s": args.max_s, "hold_s": args.hold_s,
                   "fade_s": args.fade_s},
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=1), encoding="utf-8")
    (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")

    print(f"\n[report] ticks {len(traj)} | inferences {n_inf} "
          f"({metrics['inference_hz']} Hz) | guardrail "
          f"{'ON' if guard_on else 'OFF'} | shield saw {n_touched} violation ticks | "
          f"NFZ {nfz_s:.1f}s entered={bool(inside)} | clamps {n_clamp}")
    print(f"[out] {log_path}")
    return 0


def main() -> int:
    args = build_arg_parser().parse_args()
    return asyncio.run(fly(args))


if __name__ == "__main__":
    sys.exit(main())
