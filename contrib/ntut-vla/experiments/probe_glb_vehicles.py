"""Do the downloaded glTF vehicles actually beat the meshes we already have?

Spawns each candidate at a fixed pose in front of the drone camera, captures one
frame, and measures the two things that decide it:

  * does OWL-ViT score it as "a car"?  Baseline to beat, measured on the same
    camera at the same pose: SM_Offroad_Body 0.0395, SKM_SportsCar 0.1272.
  * does its colour actually render?  The whole reason for downloading these is
    that this build binds exactly one material (M_Orange) and refuses every
    other, so colour has to arrive inside the mesh file.

Run it BEFORE adopting anything. Kenney's models are deliberately low-poly and
OWL-ViT is trained on photographs, so a stylised car scoring WORSE than the
roll-cage buggy is a real possible outcome and the point of measuring.

    python experiments/probe_glb_vehicles.py --glb-dir D:/models/kenney_car-kit/glb
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_semantic.jsonc"
DETECTOR_ID = "google/owlvit-base-patch32"

# Baselines from docs/FINDING-vehicle-mesh-and-materials.md, same camera, same
# pose. A candidate that cannot beat the buggy is not worth the complexity.
BASELINE = {"SM_Offroad_Body": 0.0395, "SKM_SportsCar": 0.1272}

# The vehicles worth trying, and the colour each should render if the embedded
# atlas is being read. Everything else in the kit is debris, cones or karts.
CANDIDATES = [
    ("sedan.glb", "a car"), ("taxi.glb", "a yellow car"),
    ("police.glb", "a car"), ("ambulance.glb", "a white car"),
    ("firetruck.glb", "a red truck"), ("van.glb", "a car"),
    ("suv.glb", "a car"), ("delivery.glb", "a truck"),
    ("sedan-sports.glb", "a car"), ("garbage-truck.glb", "a truck"),
]


async def run(args) -> int:
    import torch
    from PIL import Image
    from transformers import OwlViTForObjectDetection, OwlViTProcessor
    from projectairsim import Drone, ProjectAirSimClient, World
    from projectairsim.types import Pose, Quaternion, Vector3
    from semantic_demo import SemanticObs
    import follow_vlm as fv

    glb_dir = pathlib.Path(args.glb_dir)
    proc = OwlViTProcessor.from_pretrained(DETECTOR_ID)
    model = OwlViTForObjectDetection.from_pretrained(DETECTOR_ID).to("cuda").eval()

    client = ProjectAirSimClient()
    client.connect()
    world = World(client, SCENE, delay_after_load_sec=2, sim_config_path=SIM_CONFIG_DIR)
    drone = Drone(client, world, "Drone1")
    obs = SemanticObs()
    client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                     lambda _, m: obs.put_front(m))

    def pose_at(x, y, yaw_deg=0.0):
        h = math.radians(yaw_deg) / 2.0
        return Pose({"translation": Vector3({"x": x, "y": y, "z": args.z}),
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

    def score(img, query):
        inputs = proc(text=[[query]], images=img, return_tensors="pt").to("cuda")
        with torch.no_grad():
            out = model(**inputs)
        W, H = img.size
        res = proc.post_process_object_detection(
            out, threshold=0.0,
            target_sizes=torch.tensor([[H, W]]).to("cuda"))[0]
        sc, bx = res["scores"], res["boxes"]
        if not len(sc):
            return 0.0, None
        i = int(sc.argmax())
        return float(sc[i]), [float(v) for v in bx[i].tolist()]

    # TAKE OFF FIRST. Without this the camera sits at ground level pitched 20 deg
    # down and photographs the asphalt in front of the nose - the first run of
    # this probe scored every model at ~0.006 and the frames showed only road.
    # The candidate has to be framed the way the flight frames it.
    drone.enable_api_control()
    drone.arm()
    await (await drone.takeoff_async())
    await (await drone.move_to_position_async(args.drone_x, args.drone_y,
                                              -args.alt, velocity=3.0))
    await asyncio.sleep(1.5)

    # AIM THE NOSE AT THE SUBJECT. x is North and y is East, so a subject at the
    # same x but +22 y sits due EAST while a yaw-0 drone stares North - 90 deg
    # out of a 90 deg-wide frame. Both earlier runs of this probe framed nothing
    # but background city because of exactly that, and the control cube proved
    # it: a KNOWN-GOOD 1M_Cube spawned at the same pose was equally invisible.
    bearing = math.atan2(args.y - args.drone_y, args.x - args.drone_x)
    await (await drone.rotate_to_yaw_async(yaw=bearing))
    await asyncio.sleep(1.5)
    out_dir = ROOT / "experiments" / "out" / "glb_probe"
    (out_dir / "frames").mkdir(parents=True, exist_ok=True)
    empty = await frame()
    if empty is not None:
        empty.save(out_dir / "frames" / "empty.png")

    # Score the EMPTY frame. This street has its own parked cars, so 'a car'
    # never returns zero here - and a candidate that merely ties this number has
    # not been detected at all, it has been missed while the background was
    # found. Every candidate score below is reported against this floor.
    bg_car, bg_box = score(empty, "a car") if empty is not None else (0.0, None)
    print(f"[probe] background floor: empty frame scores 'a car' {bg_car:.4f}")

    rows = []
    for fname, query in CANDIDATES:
        path = glb_dir / fname
        if not path.exists():
            print(f"  {fname:22s} MISSING")
            continue
        data = path.read_bytes()
        name = None
        try:
            name = world.spawn_object_from_file(
                f"glb_{path.stem}", "gltf", data, True,
                pose_at(args.x, args.y), [args.scale] * 3, False)
        except Exception as e:
            print(f"  {fname:22s} SPAWN FAILED ({type(e).__name__}: {e})")
            continue
        await asyncio.sleep(1.0)
        img = await frame()
        if img is None:
            print(f"  {fname:22s} no frame")
            continue
        img.save(out_dir / "frames" / f"{path.stem}.png")
        s_car, box = score(img, "a car")
        s_q, _ = score(img, query)
        arr = np.asarray(img)
        cw = fv.colour_word(query)
        cm = fv.colour_match(arr, box, cw) if (box and cw) else None
        rows.append({"model": fname, "query": query, "score_a_car": round(s_car, 4),
                     "score_query": round(s_q, 4),
                     "colour_word": cw, "colour_match": None if cm is None else round(cm, 3),
                     "box_w_px": None if not box else round(box[2] - box[0], 1)})
        rows[-1]["over_background"] = round(s_car - bg_car, 4)
        beat = [k for k, v in BASELINE.items() if s_car > v]
        print(f"  {fname:22s} 'a car' {s_car:.4f}  {query!r} {s_q:.4f}  "
              f"(bg+{s_car - bg_car:+.4f})  "
              f"colour {('n/a' if cm is None else f'{cm:.2f}')}  "
              f"{'beats ' + '+'.join(beat) if beat else 'beats nothing'}")
        try:
            world.destroy_object(name)
        except Exception:
            pass
        await asyncio.sleep(0.3)

    (out_dir / "results.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(f"\n[probe] baseline to beat: {BASELINE}")
    if rows:
        best = max(rows, key=lambda r: r["score_a_car"])
        print(f"[probe] best candidate: {best['model']} at {best['score_a_car']}")
    print(f"[probe] frames + results -> {out_dir}")
    client.disconnect()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--glb-dir", default="D:/models/kenney_car-kit/glb")
    # Same geometry the material survey used: aircraft at 9 m, subject 22 m ahead.
    ap.add_argument("--x", type=float, default=35.0)
    ap.add_argument("--y", type=float, default=2.0)
    ap.add_argument("--drone-x", type=float, default=35.0)
    ap.add_argument("--drone-y", type=float, default=-20.0)
    ap.add_argument("--alt", type=float, default=9.0)
    ap.add_argument("--z", type=float, default=0.0)
    # Kenney ships at roughly 1 unit = 1 metre but their cars are ~2 units long,
    # so a real 4.3 m car needs about 2x. Measured per model by the probe output.
    ap.add_argument("--scale", type=float, default=2.0)
    return asyncio.run(run(ap.parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
