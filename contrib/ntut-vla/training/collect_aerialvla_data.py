"""
Data collector for AerialVLA fine-tuning — expert flights in AirSimNH.

A scripted expert flies randomized go-to-target episodes while we record, at
2 Hz, exactly what the VLA would see and what it should do:

    image  : front+down camera mosaic (224x448 PNG)
    text   : "<image>\nFly {semantic_dir}and find the target. {description}"
    label  : (fwd m/s, down m/s, yaw rad/s)  — the expert's action

Same schema as AerialVLA's own aerovla_train_dataset.json, so the fine-tune
script can mix both if ever needed. Episodes teleport the drone to a fresh
random spot on the map (visual diversity) instead of restarting the sim.

Scope honesty: the expert teaches direction-following, speed discipline and
altitude keeping — the skills raw AerialVLA lacked in our worlds. No-fly-zone
geometry stays the GUARDRAIL's job (invisible to a camera; see scope doc).

Run (vla-real env, AirSimNH running):
    python training/collect_aerialvla_data.py --episodes 60
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "dataset" / "aerialvla_ft"

CRUISE = 20.0
FWD_MAX = 5.0
DESCRIPTIONS = [
    "an open area near the target point",
    "a clear spot on the ground below",
    "the marked landing area ahead",
    "an open space between the buildings",
    "a clear area near the road",
]


def semantic_direction(pos, yaw, target) -> str:
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


def main() -> int:
    global OUT
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=60)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="", help="dataset output dir")
    ap.add_argument("--decide-every", type=int, default=1,
                    help="expert re-decides every N ticks (12 ~ VLA latency), "
                         "holds the action in between; records AT decision ticks")
    ap.add_argument("--yaw-p", type=float, default=1.2, help="expert yaw P gain")
    ap.add_argument("--yaw-max", type=float, default=1.1)
    args = ap.parse_args()
    random.seed(args.seed)
    if args.out:
        OUT = Path(args.out)

    import airsim
    import cv2

    (OUT / "images").mkdir(parents=True, exist_ok=True)
    samples_path = OUT / "samples.json"
    samples: list = []
    if samples_path.exists():
        samples = json.loads(samples_path.read_text(encoding="utf-8"))
        print(f"[collect] resuming, {len(samples)} samples already on disk")

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    def grab(camera: str):
        png = client.simGetImage(camera, airsim.ImageType.Scene)
        if not png:
            return None
        bgr = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        return cv2.resize(bgr, (224, 224), interpolation=cv2.INTER_AREA)

    def pose_xyzyaw():
        p = client.simGetVehiclePose()
        q = p.orientation
        yaw = math.atan2(2 * (q.w_val * q.z_val + q.x_val * q.y_val),
                         1 - 2 * (q.y_val ** 2 + q.z_val ** 2))
        return p.position.x_val, p.position.y_val, -p.position.z_val, yaw

    ep_offset = len(list((OUT / "images" / "nh").glob("*"))) \
        if (OUT / "images" / "nh").exists() else 0
    if ep_offset:
        print(f"[collect] episode numbering resumes at {ep_offset}")

    t_start = time.time()
    ep_done = 0
    for ep in range(ep_offset, ep_offset + args.episodes):
        # --- teleport to a fresh random spot, reset attitude ---
        sx = random.uniform(-120, 120)
        sy = random.uniform(-120, 120)
        pose = airsim.Pose(airsim.Vector3r(sx, sy, -CRUISE),
                           airsim.to_quaternion(0, 0, random.uniform(-math.pi, math.pi)))
        client.simSetVehiclePose(pose, ignore_collision=True)
        client.moveByVelocityAsync(0, 0, 0, duration=0.5).join()

        bearing = random.uniform(-math.pi, math.pi)
        dist = random.uniform(35, 70)
        tx, ty = sx + dist * math.cos(bearing), sy + dist * math.sin(bearing)
        desc = random.choice(DESCRIPTIONS)
        ep_dir = f"nh/{ep:04d}"
        (OUT / "images" / ep_dir).mkdir(parents=True, exist_ok=True)

        # v2: retarget mid-episode so the dataset is rich in TURNS (v1 was ~90%
        # fly-straight samples -> the model unlearned turning; classic BC collapse)
        retarget_at = random.randint(60, 120)  # ticks between target switches

        n, tick = 0, 0
        ep_samples = []
        yaw_rate, fwd, down = 0.0, 0.0, 0.0
        while tick < 900:                     # max 90 s
            tick += 1
            if tick % retarget_at == 0:
                b2 = random.uniform(-math.pi, math.pi)
                d2 = random.uniform(35, 70)
                x0, y0, _, _ = pose_xyzyaw()
                tx, ty = x0 + d2 * math.cos(b2), y0 + d2 * math.sin(b2)
            x, y, up, yaw = pose_xyzyaw()
            gd = math.hypot(tx - x, ty - y)
            if gd < 2.5:
                break

            # --- expert action (body frame); re-decided every N ticks and HELD
            # in between, so the data matches the VLA's decision latency ---
            decide = (tick % args.decide_every == 1) or args.decide_every == 1
            if decide:
                des_yaw = math.atan2(ty - y, tx - x)
                err = (des_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
                yaw_rate = float(np.clip(args.yaw_p * err, -args.yaw_max, args.yaw_max))
                fwd = float(np.clip(gd * 0.35, 0.8, FWD_MAX))
                fwd *= max(0.15, math.cos(min(abs(err), math.pi / 2)))
                down = float(np.clip(-(CRUISE - up) * 0.8, -3.0, 3.0))

            # --- record at decision ticks; downsample boring straight samples ---
            straight = abs(yaw_rate) < 0.08
            keep = (not straight) or (random.random() < 0.3)
            rec = decide if args.decide_every > 1 else (tick % 5 == 1)
            if rec and keep:
                f = grab("0")
                d = grab("3")
                if f is not None and d is not None:
                    mosaic = np.vstack([f, d])              # 448x224 BGR
                    img_name = f"{n:06d}.png"
                    cv2.imwrite(str(OUT / "images" / ep_dir / img_name), mosaic)
                    ep_samples.append({
                        "traj_rel_dir": ep_dir,
                        "img_name": img_name,
                        "instruction": f"<image>\nFly {semantic_direction((x, y), yaw, (tx, ty))}"
                                       f"and find the target. {desc}",
                        "label": {"fwd": fwd, "down": down, "yaw": yaw_rate},
                        "is_last_step": False,
                    })
                    n += 1

            # --- act (expert flies body-frame like the VLA contract) ---
            vx = fwd * math.cos(yaw)
            vy = fwd * math.sin(yaw)
            client.moveByVelocityAsync(
                vx, vy, down, duration=0.3,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(yaw_rate)))
            time.sleep(0.1)

        if ep_samples:
            ep_samples[-1]["is_last_step"] = True
            # terminal LAND sample: hover action at the target
            samples.extend(ep_samples)
        ep_done += 1
        if ep % 5 == 0 or ep == args.episodes - 1:
            samples_path.write_text(json.dumps(samples), encoding="utf-8")
            el = time.time() - t_start
            print(f"[collect] ep {ep + 1}/{args.episodes} | samples {len(samples)} "
                  f"| {el/60:.1f} min", flush=True)

    samples_path.write_text(json.dumps(samples), encoding="utf-8")
    print(f"[collect] DONE: {len(samples)} samples, {ep_done} episodes -> {OUT}")
    client.armDisarm(False)
    client.enableApiControl(False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
