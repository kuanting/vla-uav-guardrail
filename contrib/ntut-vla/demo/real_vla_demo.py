"""
Real VLA in the guardrail slot — OpenVLA-7B flying AirSim, guarded.

This is the "plug a real VLA in" demo. A genuine 7-billion-parameter
camera+language model (openvla/openvla-7b — the base AeroVLA fine-tunes) sits
in the slot: it SEES the drone's camera frame, READS a natural-language
instruction, and emits actions. Our Safety Shield filters every action before
it reaches the sim.

Smooth-flight design (v2 — fixes the freeze-then-jerk look):
  - Inference (~0.9 s at 4-bit) runs in a BACKGROUND THREAD with its own
    AirSim connection; the 10 Hz control loop never blocks.
  - A rate limiter caps per-tick velocity change BEFORE the shield, so action
    hand-offs blend instead of snapping. Shield stays the final authority.

HONEST NOTE: base OpenVLA is trained on robot-ARM manipulation, not UAVs, so
its raw actions do NOT navigate sensibly — that is expected. This demo proves:
  1. The slot works with a REAL camera+language VLA — same
     `observation -> Action4D` contract, small adapter.
  2. The guardrail keeps even a mismatched real VLA SAFE: zero NFZ seconds.
Swapping in AeroVLA's UAV LoRA (sensible flight) is then a weights change only.

Run (vla-real env, model downloaded, AirSim NH running):
    python demo/real_vla_demo.py --instruction "fly forward and stay clear of buildings"
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import AuditLogger, Shield, State, load_policy       # noqa: E402
from guardrail.compiler import ConstraintCompiler                   # noqa: E402
from guardrail.geometry import fence_polygon                        # noqa: E402
from guardrail.models import Action4D, PolygonFence                 # noqa: E402
from shapely.geometry import Point                                  # noqa: E402

TICK = 0.1
MAX_S = 60
MODEL_ID = "D:/models/openvla-7b"   # local copy (Windows symlink-free download)


class RateLimiter:
    """Cap per-tick velocity change so action hand-offs blend, not snap."""

    def __init__(self, dv_h: float = 0.25, dv_z: float = 0.15):
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


class OpenVLABackend:
    """Real OpenVLA-7B in the slot.

    Inference runs in a background thread with its OWN AirSim client (the
    msgpack-rpc client is not thread-safe), continuously refreshing the
    latest action. `act()` never blocks — it just returns the latest.
    """

    def __init__(self, instruction: str, scale: float = 150.0):
        import torch
        from transformers import (AutoModelForVision2Seq, AutoProcessor,
                                  BitsAndBytesConfig)

        self.instruction = instruction
        self.scale = scale
        self._last = Action4D()
        self._lock = threading.Lock()
        self._stop = False
        self._n_inf = 0
        self.torch = torch

        print(f"[vla] loading {MODEL_ID} ...")
        t0 = time.time()
        self.proc = AutoProcessor.from_pretrained(MODEL_ID, trust_remote_code=True)
        # 4-bit NF4 quantization: 15.1 GB -> 4.7 GB VRAM so the UE4 sim and the
        # model share the 16 GB card without starving each other (full bf16
        # caused RPC timeouts). Also ~4x faster: 0.9 s/inference vs 3.4 s.
        self.model = AutoModelForVision2Seq.from_pretrained(
            MODEL_ID,
            attn_implementation="eager",           # no flash-attn on Windows
            torch_dtype=torch.bfloat16,
            quantization_config=BitsAndBytesConfig(
                load_in_4bit=True, bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_quant_type="nf4"),
            device_map={"": 0},
            low_cpu_mem_usage=True,
            trust_remote_code=True,
        ).eval()
        print(f"[vla] loaded in {time.time() - t0:.0f}s "
              f"(VRAM {torch.cuda.memory_allocated()/1e9:.1f} GB)")
        self._thread = threading.Thread(target=self._worker, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop = True

    def _worker(self) -> None:
        import airsim
        import cv2
        from PIL import Image

        cam = airsim.MultirotorClient()          # own connection for this thread
        cam.confirmConnection()
        prompt = f"In: What action should the robot take to {self.instruction}?\nOut:"
        while not self._stop:
            png = cam.simGetImage("0", airsim.ImageType.Scene)
            if not png:
                time.sleep(0.05)
                continue
            bgr = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                continue
            img = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)).resize((224, 224))
            inputs = self.proc(prompt, img).to("cuda", dtype=self.torch.bfloat16)
            with self.torch.no_grad():
                # NOTE: pass input_ids + pixel_values ONLY. predict_action appends
                # one token (29871) to input_ids but does NOT extend a passed
                # attention_mask -> off-by-one crash inside generate. Omit the
                # mask and generate builds a correct full-ones mask itself.
                raw = self.model.predict_action(
                    input_ids=inputs["input_ids"], pixel_values=inputs["pixel_values"],
                    unnorm_key="bridge_orig", do_sample=False)
            # OpenVLA emits 7-DoF arm deltas; map first 3 to body velocities,
            # scaled into the drone's envelope. NOT UAV-tuned — the guardrail is
            # what makes this safe. (AeroVLA's LoRA would replace this mapping.)
            r = np.asarray(raw, dtype=float).flatten()
            act = Action4D(
                vx=float(np.clip(r[0] * self.scale, -4, 4)),
                vy=float(np.clip(r[1] * self.scale, -4, 4)),
                vz_up=float(np.clip(r[2] * self.scale * 0.5, -2, 2)),
                yaw_rate=0.0,
            )
            with self._lock:
                self._last = act
                self._n_inf += 1
            if self._n_inf % 5 == 1:
                print(f"  [vla#{self._n_inf}] raw={r[:3].round(4)} -> "
                      f"act=({act.vx:.2f},{act.vy:.2f},{act.vz_up:.2f})")

    def act(self, state: State) -> Action4D:
        with self._lock:
            return self._last


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--instruction", default="fly forward and avoid restricted areas")
    ap.add_argument("--policy", default=str(ROOT / "policies" / "urban_demo_policy.yaml"))
    ap.add_argument("--tag", default="real_vla")
    ap.add_argument("--scale", type=float, default=150.0,
                    help="gain from OpenVLA arm-deltas to m/s (bigger = livelier)")
    ap.add_argument("--dv-h", type=float, default=0.25, help="max m/s change per tick, horizontal")
    ap.add_argument("--dv-z", type=float, default=0.15, help="max m/s change per tick, vertical")
    args = ap.parse_args()

    import airsim
    out = ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)

    policy = load_policy(args.policy)
    mission = ConstraintCompiler(policy).parse_command("fly to (40, 40) at 6 m/s altitude 20")
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    audit = AuditLogger(out / "audit.jsonl", policy)   # the POLICY, so a hot-applied rule restamps the hash
    print(f"[policy] {policy.policy_id} {policy.policy_hash}")
    print(f"[task]   instruction = {args.instruction!r}")

    # load the model BEFORE opening the control connection (load takes minutes)
    vla = OpenVLABackend(args.instruction, scale=args.scale)

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
    vla.start()                                   # inference thread from here on
    print("[flight] cruise reached — real VLA now flying (guardrail ON)")

    limiter = RateLimiter(args.dv_h, args.dv_z)
    traj, n_touched = [], 0
    t0 = time.time()
    tick = 0
    while time.time() - t0 < MAX_S:
        tick += 1
        pos = client.simGetVehiclePose().position
        state = State(x=pos.x_val, y=pos.y_val, up=-pos.z_val)
        raw = vla.act(state)                      # never blocks
        smooth = limiter(raw)                     # blend hand-offs BEFORE shield
        d = shield.filter(state, smooth)          # shield = final authority
        audit.log(tick, d)
        if d.touched:
            n_touched += 1
        traj.append({"x": state.x, "y": state.y, "up": state.up, "touched": d.touched})
        e = d.emitted
        client.moveByVelocityAsync(e.vx, e.vy, -e.vz_up, duration=0.3)
        if tick % 50 == 0:
            print(f"  tick {tick}: pos=({state.x:5.1f},{state.y:5.1f},{state.up:4.1f}) "
                  f"cmd=({e.vx:.1f},{e.vy:.1f}) shield={'HIT' if d.touched else '-'}")
        time.sleep(TICK)

    vla.stop()
    # gentle landing: slow descent instead of landAsync's drop
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
        pxs = [p["y"] for p in traj]                    # map view: East vs North
        pys = [p["x"] for p in traj]
        ax.plot(pxs, pys, "-", color="tab:blue", linewidth=2, label="flight path")
        tx = [p["y"] for p in traj if p["touched"]]
        ty = [p["x"] for p in traj if p["touched"]]
        if tx:
            ax.plot(tx, ty, ".", color="orange", markersize=6, label="shield active")
        ax.plot(traj[0]["y"], traj[0]["x"], "go", markersize=10, label="start")
        ax.plot(traj[-1]["y"], traj[-1]["x"], "ks", markersize=8, label="end")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_title(f"REAL VLA (OpenVLA-7B) through the guardrail\n"
                     f"instruction: {args.instruction!r}\n"
                     f"NFZ time {nfz_s:.1f}s | shield interventions {n_touched}")
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
    print(f"\n[report] real VLA in the loop | ticks {len(traj)} | "
          f"shield interventions {n_touched} | time inside NFZ {nfz_s:.1f}s -> "
          f"{'PASS' if nfz_s == 0 else 'FAIL'}")
    print("[note] raw flight is not UAV-sensible (base OpenVLA is arm-trained); "
          "the guardrail keeping it inside the rules is the result.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
