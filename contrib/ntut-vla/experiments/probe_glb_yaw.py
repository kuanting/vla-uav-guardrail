"""Which way does a spawned GLB car actually face?

glTF is Y-up with -Z forward; the sim is Z-up NED with heading 0 = +x (North).
Those conventions do not agree, and the offset between them is not something to
reason about from one screenshot - a fleet driving 90 degrees sideways would look
worse than the buggy it replaced.

Measured, not guessed. The camera looks due East. A car is ~4.3 m long and ~1.9 m
wide, so the yaw at which it faces NORTH shows the camera its full flank and
produces the WIDEST detection box; facing East or West shows the narrow end.
Sweeping yaw and taking the argmax of box width therefore recovers the offset
directly, and the aspect ratio says how trustworthy the answer is.

    python experiments/probe_glb_yaw.py
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_semantic.jsonc"
DETECTOR_ID = "google/owlvit-base-patch32"


async def run(args) -> int:
    import torch
    from transformers import OwlViTForObjectDetection, OwlViTProcessor
    from projectairsim import Drone, ProjectAirSimClient, World
    from projectairsim.types import Pose, Quaternion, Vector3
    from semantic_demo import SemanticObs

    proc = OwlViTProcessor.from_pretrained(DETECTOR_ID)
    model = OwlViTForObjectDetection.from_pretrained(DETECTOR_ID).to("cuda").eval()

    client = ProjectAirSimClient()
    client.connect()
    world = World(client, SCENE, delay_after_load_sec=2, sim_config_path=SIM_CONFIG_DIR)
    drone = Drone(client, world, "Drone1")
    obs = SemanticObs()
    client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                     lambda _, m: obs.put_front(m))

    def pose_at(x, y, yaw_deg):
        h = math.radians(yaw_deg) / 2.0
        return Pose({"translation": Vector3({"x": x, "y": y, "z": 0.0}),
                     "rotation": Quaternion({"w": math.cos(h), "x": 0.0,
                                             "y": 0.0, "z": math.sin(h)}),
                     "frame_id": "DEFAULT_ID"})

    async def frame():
        for _ in range(40):
            await asyncio.sleep(0.25)
            img = obs.get_front_native()
            if img is not None:
                return img
        return None

    def box_of(img, query="a car"):
        inputs = proc(text=[[query]], images=img, return_tensors="pt").to("cuda")
        with torch.no_grad():
            out = model(**inputs)
        W, H = img.size
        res = proc.post_process_object_detection(
            out, threshold=0.0, target_sizes=torch.tensor([[H, W]]).to("cuda"))[0]
        sc, bx = res["scores"], res["boxes"]
        if not len(sc):
            return None, 0.0
        i = int(sc.argmax())
        return [float(v) for v in bx[i].tolist()], float(sc[i])

    drone.enable_api_control()
    drone.arm()
    await (await drone.takeoff_async())
    await (await drone.move_to_position_async(args.drone_x, args.drone_y,
                                              -args.alt, velocity=3.0))
    await asyncio.sleep(1.0)
    bearing = math.atan2(args.y - args.drone_y, args.x - args.drone_x)
    await (await drone.rotate_to_yaw_async(yaw=bearing))
    await asyncio.sleep(1.5)

    out_dir = ROOT / "experiments" / "out" / "glb_yaw"
    out_dir.mkdir(parents=True, exist_ok=True)
    data = pathlib.Path(args.glb).read_bytes()

    rows = []
    for yaw in args.yaws:
        name = world.spawn_object_from_file(
            f"yaw_{int(yaw)}", "gltf", data, True,
            pose_at(args.x, args.y, yaw), [args.scale] * 3, False)
        await asyncio.sleep(1.0)
        img = await frame()
        if img is not None:
            img.save(out_dir / f"yaw_{int(yaw):03d}.png")
        box, sc = box_of(img) if img is not None else (None, 0.0)
        if box:
            w, h = box[2] - box[0], box[3] - box[1]
            rows.append({"yaw_deg": yaw, "w_px": round(w, 1), "h_px": round(h, 1),
                         "aspect": round(w / max(h, 1e-6), 2), "score": round(sc, 4)})
            print(f"  yaw {yaw:6.1f}  w {w:6.1f}px  h {h:5.1f}px  "
                  f"aspect {w / max(h, 1e-6):5.2f}  score {sc:.4f}")
        try:
            world.destroy_object(name)
        except Exception:
            pass
        await asyncio.sleep(0.3)

    if rows:
        best = max(rows, key=lambda r: r["aspect"])
        print(f"\n[yaw] broadest flank at yaw={best['yaw_deg']} deg "
              f"(aspect {best['aspect']})")
        print(f"[yaw] the camera looks EAST, so that yaw points the car NORTH.")
        print(f"[yaw] heading 0 means +x/North, so GLB_YAW_OFFSET_DEG = "
              f"{best['yaw_deg']}")
    (out_dir / "yaw.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    client.disconnect()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--glb", default="D:/models/kenney_car-kit/glb/sedan.glb")
    ap.add_argument("--x", type=float, default=35.0)
    ap.add_argument("--y", type=float, default=2.0)
    ap.add_argument("--drone-x", type=float, default=35.0)
    ap.add_argument("--drone-y", type=float, default=-20.0)
    ap.add_argument("--alt", type=float, default=9.0)
    ap.add_argument("--scale", type=float, default=2.0)
    ap.add_argument("--yaws", type=float, nargs="+",
                    default=[0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0])
    return asyncio.run(run(ap.parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
