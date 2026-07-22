"""
AerialVLA (UAV-trained VLA) in the guardrail slot — goal-directed real flight.

AerialVLA = openvla-7b base + LoRA fine-tuned on the TravelUAV benchmark
(https://huggingface.co/XuPeng23/AerialVLA). Unlike base OpenVLA (arm-trained,
aimless on a drone), this model was TRAINED TO FLY: it reads a front+down
camera mosaic plus a text hint of where the target is, and emits
(forward m/s, down m/s, yaw rad/s) — nearly 1:1 with our Action4D contract.

Pipeline (unchanged, as always):
    cameras -> AerialVLA -> body->world convert -> RateLimiter -> Shield -> sim

Run (vla-real env, base model + LoRA downloaded, AirSim NH running):
    python demo/aerialvla_demo.py
"""
from __future__ import annotations

import argparse
import math
import re
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import AuditLogger, Shield, State, load_policy       # noqa: E402
from guardrail.compiler import ConstraintCompiler                   # noqa: E402
from guardrail.geometry import fence_polygon                        # noqa: E402
from guardrail.models import Action4D, PolygonFence                 # noqa: E402
from shapely.geometry import Point                                  # noqa: E402

TICK = 0.1
MAX_S = 90
BASE_ID = "D:/models/openvla-7b"
LORA_ID = "D:/models/aerialvla-lora/aero_vla"

NUM_BINS = 99
NORM = {"forward": (0.0, 5.0), "down": (-5.0, 5.0), "yaw": (-1.1, 1.1)}


def dequantize(bin_val: int, axis: str) -> float:
    vmin, vmax = NORM[axis]
    return (max(0, min(NUM_BINS - 1, bin_val)) / (NUM_BINS - 1)) * (vmax - vmin) + vmin


def semantic_direction(pos, yaw, target) -> str:
    """AerialVLA's training-time hint: rough direction of the target, body frame."""
    dx, dy = target[0] - pos[0], target[1] - pos[1]
    if math.hypot(dx, dy) < 0.5:
        return ""
    ang = math.degrees(math.atan2(dy, dx) - yaw)
    ang = (ang + 180) % 360 - 180
    if -15 <= ang <= 15:
        return "straight ahead "
    if 15 < ang <= 60:
        return "forward-right "
    if 60 < ang <= 120:
        return "to your right "
    if 120 < ang <= 180:
        return "to your right rear "
    if -60 <= ang < -15:
        return "forward-left "
    if -120 <= ang < -60:
        return "to your left "
    return "to your left rear "


def make_airsim_obs():
    """Default obs source: classic AirSim RPC (called inside the VLA thread)."""
    import airsim
    import cv2
    from PIL import Image

    cam = airsim.MultirotorClient()              # own connection (thread safety)
    cam.confirmConnection()

    def grab(camera: str):
        png = cam.simGetImage(camera, airsim.ImageType.Scene)
        if not png:
            return None
        bgr = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        return Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)).resize(
            (224, 224), resample=Image.BICUBIC)

    def get_obs():
        front, down = grab("0"), grab("3")
        if front is None or down is None:
            return None
        pose = cam.simGetVehiclePose()
        q = pose.orientation
        yaw = math.atan2(2 * (q.w_val * q.z_val + q.x_val * q.y_val),
                         1 - 2 * (q.y_val ** 2 + q.z_val ** 2))
        return front, down, (pose.position.x_val, pose.position.y_val, yaw)

    return get_obs


class AerialVLABackend:
    """UAV-trained VLA in the slot. Threaded inference, never blocks the loop.

    Emits BODY-frame (fwd, down, yaw_rate); `latest()` returns it raw — the
    flight loop converts to world frame (needs current yaw) before the shield.
    """

    def __init__(self, obj_desc: str, target_xy, obs_factory=None,
                 lora_id: str = LORA_ID):
        """
        obs_factory: callable invoked ONCE inside the worker thread; must
        return a zero-arg callable producing (front_PIL, down_PIL, (x, y, yaw))
        or None. Defaults to the classic-AirSim implementation below.
        """
        import torch
        from peft import PeftModel
        from transformers import (AutoImageProcessor, AutoModelForVision2Seq,
                                  AutoTokenizer, BitsAndBytesConfig)

        self.obj_desc = obj_desc
        self.target_xy = target_xy
        self.obs_factory = obs_factory or make_airsim_obs
        self._lock = threading.Lock()
        self._stop = False
        self._n_inf = 0
        self._fwd, self._down, self._yaw = 0.0, 0.0, 0.0
        self._land = False
        self.torch = torch

        self.lora_id = lora_id
        print(f"[vla] loading base {BASE_ID} + LoRA {lora_id} (4-bit)...")
        t0 = time.time()
        self.tok = AutoTokenizer.from_pretrained(BASE_ID, trust_remote_code=True)
        self.imgproc = AutoImageProcessor.from_pretrained(BASE_ID, trust_remote_code=True)
        base = AutoModelForVision2Seq.from_pretrained(
            BASE_ID,
            attn_implementation="eager",
            torch_dtype=torch.bfloat16,
            quantization_config=BitsAndBytesConfig(
                load_in_4bit=True, bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_quant_type="nf4",
                # LoRA's modules_to_save replaces the projector wholesale —
                # keep it un-quantized (bf16) or PEFT can't wrap it.
                llm_int8_skip_modules=["projector"]),
            device_map={"": 0},
            low_cpu_mem_usage=True,
            trust_remote_code=True,
        )
        # (upstream code calls resize_token_embeddings(len(tok)); unnecessary
        # here — the adapter repo ships no tokenizer, so vocab == base — and it
        # crashes on a quantized lm_head. Skip it.)
        self.model = PeftModel.from_pretrained(base, self.lora_id).eval()
        print(f"[vla] loaded in {time.time() - t0:.0f}s "
              f"(VRAM {torch.cuda.memory_allocated()/1e9:.1f} GB)")
        self._thread = threading.Thread(target=self._worker, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True

    def _worker(self):
        from PIL import Image

        get_obs = self.obs_factory()             # created IN this thread
        while not self._stop:
            obs = get_obs()
            if obs is None:
                time.sleep(0.05)
                continue
            front, down, (px, py, yaw) = obs
            mosaic = Image.new("RGB", (224, 448), (0, 0, 0))
            mosaic.paste(front, (0, 0))
            mosaic.paste(down, (0, 224))

            dir_text = semantic_direction((px, py), yaw, self.target_xy)

            prompt = f"<image>\nFly {dir_text}and find the target. {self.obj_desc}\nAction: "
            enc = self.tok(prompt, return_tensors="pt")
            pv = self.imgproc(images=mosaic, return_tensors="pt")["pixel_values"]
            with self.torch.inference_mode():
                out = self.model.generate(
                    input_ids=enc["input_ids"].to("cuda"),
                    attention_mask=enc["attention_mask"].to("cuda"),
                    pixel_values=pv.to("cuda", dtype=self.torch.bfloat16),
                    max_new_tokens=20, do_sample=False,
                    eos_token_id=[self.tok.eos_token_id])
            text = self.tok.decode(out[0], skip_special_tokens=False)
            tail = text.split("Action:")[-1]
            ints = re.findall(r"\d+", tail)
            if len(ints) >= 3:
                fwd = dequantize(int(ints[-3]), "forward")
                dwn = dequantize(int(ints[-2]), "down")
                yr = dequantize(int(ints[-1]), "yaw")
                land = ("LAND" in tail) or (fwd < 0.01 and abs(dwn) < 0.01 and abs(yr) < 0.01)
                with self._lock:
                    self._fwd, self._down, self._yaw, self._land = fwd, dwn, yr, land
                    self._n_inf += 1
                if self._n_inf % 5 == 1:
                    print(f"  [vla#{self._n_inf}] dir={dir_text.strip() or 'here'!r} "
                          f"fwd={fwd:.2f} down={dwn:.2f} yaw={yr:.2f}"
                          f"{' LAND' if land else ''}")

    def latest(self):
        with self._lock:
            return self._fwd, self._down, self._yaw, self._land


class RateLimiter:
    def __init__(self, dv_h=0.25, dv_z=0.15):
        self.dv_h, self.dv_z = dv_h, dv_z
        self.prev = Action4D()

    def __call__(self, a: Action4D) -> Action4D:
        p = self.prev
        out = Action4D(
            vx=p.vx + float(np.clip(a.vx - p.vx, -self.dv_h, self.dv_h)),
            vy=p.vy + float(np.clip(a.vy - p.vy, -self.dv_h, self.dv_h)),
            vz_up=p.vz_up + float(np.clip(a.vz_up - p.vz_up, -self.dv_z, self.dv_z)),
            yaw_rate=a.yaw_rate,
        )
        self.prev = out
        return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--command", default="fly to (40, 40) at 6 m/s altitude 20")
    ap.add_argument("--object", default="an open area near the target point",
                    help="target description fed to the VLA prompt")
    ap.add_argument("--policy", default=str(ROOT / "policies" / "urban_demo_policy.yaml"))
    ap.add_argument("--tag", default="aerialvla")
    ap.add_argument("--dv-h", type=float, default=0.25)
    ap.add_argument("--dv-z", type=float, default=0.15)
    ap.add_argument("--yaw-gain", type=float, default=1.0,
                    help="multiplier on the VLA's yaw-rate output")
    ap.add_argument("--goal-blend", type=float, default=0.0,
                    help="0..1: mission-direction assist blended into the VLA "
                         "action (0 = pure VLA, guardrail-side guidance)")
    ap.add_argument("--max-s", type=float, default=90.0)
    ap.add_argument("--adapter", default=LORA_ID,
                    help="LoRA adapter dir (original or a fine-tuned checkpoint)")
    args = ap.parse_args()
    global MAX_S
    MAX_S = args.max_s

    import airsim
    out = ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)

    policy = load_policy(args.policy)
    mission = ConstraintCompiler(policy).parse_command(args.command)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    audit = AuditLogger(out / "audit.jsonl", policy.policy_hash)
    target = (mission.target_x, mission.target_y)
    print(f"[policy] {policy.policy_id} {policy.policy_hash}")
    print(f"[task]   target={target}  object={args.object!r}")

    vla = AerialVLABackend(args.object, target, lora_id=args.adapter)

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    for _ in range(150):
        up = -client.simGetVehiclePose().position.z_val
        if up >= mission.cruise_alt_m - 0.5:
            break
        client.moveByVelocityAsync(0, 0, -2.0, duration=0.2)
        time.sleep(0.1)
    vla.start()
    print("[flight] cruise reached — AerialVLA flying (guardrail ON)")

    limiter = RateLimiter(args.dv_h, args.dv_z)
    traj, n_touched = [], 0
    t0 = time.time()
    tick = 0
    reached = False
    while time.time() - t0 < MAX_S:
        tick += 1
        pose = client.simGetVehiclePose()
        pos = pose.position
        q = pose.orientation
        yaw = math.atan2(2 * (q.w_val * q.z_val + q.x_val * q.y_val),
                         1 - 2 * (q.y_val ** 2 + q.z_val ** 2))
        state = State(x=pos.x_val, y=pos.y_val, up=-pos.z_val)

        fwd, dwn, yr, land = vla.latest()
        # body (fwd along heading) -> world frame for the shield's forecast
        vx, vy = fwd * math.cos(yaw), fwd * math.sin(yaw)
        # optional mission-direction assist: blend a unit vector toward the
        # compiled target into the VLA's horizontal action (guidance lives on
        # the guardrail side; the VLA still does the seeing and steering)
        a = args.goal_blend
        if a > 0:
            gd = math.hypot(target[0] - state.x, target[1] - state.y)
            if gd > 1e-6:
                gvx = (target[0] - state.x) / gd * mission.speed_pref_mps
                gvy = (target[1] - state.y) / gd * mission.speed_pref_mps
                vx, vy = (1 - a) * vx + a * gvx, (1 - a) * vy + a * gvy
        raw = Action4D(vx=vx, vy=vy, vz_up=-dwn, yaw_rate=yr * args.yaw_gain)
        smooth = limiter(raw)
        d = shield.filter(state, smooth)
        audit.log(tick, d)
        if d.touched:
            n_touched += 1
        traj.append({"x": state.x, "y": state.y, "up": state.up, "touched": d.touched})
        e = d.emitted
        client.moveByVelocityAsync(
            e.vx, e.vy, -e.vz_up, duration=0.3,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(e.yaw_rate)))
        if math.hypot(state.x - target[0], state.y - target[1]) < 3.0:
            reached = True
            print(f"  [flight] target reached at tick {tick}")
            break
        if tick % 50 == 0:
            print(f"  tick {tick}: pos=({state.x:5.1f},{state.y:5.1f},{state.up:4.1f}) "
                  f"cmd=({e.vx:.1f},{e.vy:.1f}) shield={'HIT' if d.touched else '-'}")
        time.sleep(TICK)

    vla.stop()
    for _ in range(200):
        up = -client.simGetVehiclePose().position.z_val
        if up <= 1.0:
            break
        client.moveByVelocityAsync(0, 0, 0.8, duration=0.3)
        time.sleep(0.1)
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)

    fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
    inside = sum(1 for p in traj for f, poly in fences
                 if f.altitude_floor_m <= p["up"] <= f.altitude_ceiling_m
                 and poly.contains(Point(p["x"], p["y"])))
    nfz_s = inside * TICK

    import json
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
        ax.plot([p["y"] for p in traj], [p["x"] for p in traj], "-",
                color="tab:blue", linewidth=2, label="flight path")
        tx = [p["y"] for p in traj if p["touched"]]
        ty = [p["x"] for p in traj if p["touched"]]
        if tx:
            ax.plot(tx, ty, ".", color="orange", markersize=6, label="shield active")
        ax.plot(traj[0]["y"], traj[0]["x"], "go", markersize=10, label="start")
        ax.plot(target[1], target[0], "k*", markersize=16, label="target")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_title(f"AerialVLA (UAV-trained) through the guardrail\n"
                     f"NFZ time {nfz_s:.1f}s | interventions {n_touched} | "
                     f"{'REACHED' if reached else 'not reached'}")
        ax.legend(loc="upper left", fontsize=9)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)

        ts = [i * TICK for i in range(len(traj))]
        ax2.plot(ts, [p["up"] for p in traj], color="tab:blue", label="altitude")
        from guardrail.models import AltitudeEnvelope
        for env in policy.by_type(AltitudeEnvelope):
            ax2.axhspan(env.alt_min_m, env.alt_max_m, alpha=0.12, color="green",
                        label=f"allowed band [{env.alt_min_m:.0f},{env.alt_max_m:.0f}]m")
        hit_t = [i * TICK for i, p in enumerate(traj) if p["touched"]]
        hit_a = [p["up"] for p in traj if p["touched"]]
        if hit_t:
            ax2.plot(hit_t, hit_a, ".", color="orange", markersize=6, label="shield active")
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

    # efficiency metrics (for the auto-tuning loop)
    path_len = sum(math.hypot(traj[i]["x"] - traj[i - 1]["x"],
                              traj[i]["y"] - traj[i - 1]["y"])
                   for i in range(1, len(traj)))
    straight = math.hypot(target[0] - traj[0]["x"], target[1] - traj[0]["y"])
    spl = (straight / path_len) if (reached and path_len > 0) else 0.0
    metrics = {"reached": reached, "ticks": len(traj), "nfz_s": nfz_s,
               "interventions": n_touched, "path_len_m": round(path_len, 1),
               "straight_m": round(straight, 1), "efficiency": round(spl, 3),
               "params": {"yaw_gain": args.yaw_gain, "goal_blend": args.goal_blend,
                          "dv_h": args.dv_h, "dv_z": args.dv_z}}
    (out / "metrics.json").write_text(json.dumps(metrics, indent=1), encoding="utf-8")

    print(f"\n[report] AerialVLA in the loop | ticks {len(traj)} | "
          f"shield interventions {n_touched} | NFZ {nfz_s:.1f}s -> "
          f"{'PASS' if nfz_s == 0 else 'FAIL'} | "
          f"target {'REACHED' if reached else 'not reached'} | "
          f"efficiency {spl:.3f} (path {path_len:.0f}m vs straight {straight:.0f}m)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
