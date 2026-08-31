"""Can the downloaded GLB vehicles actually replace the traffic fleet?

Detection score and colour are settled (see docs/FINDING-glb-vehicles-aug15.md).
What is NOT settled is whether they can do the job `city_traffic.py` needs, which
is a harder ask than standing still:

  T1  movable   - the fleet is driven by SetObjectPose at 10 Hz. A prop that
                  spawns beautifully and then refuses to move is useless here,
                  and this build has already produced one "not movable" fault.
  T2  multiple  - four vehicles must coexist in one scene.
  T3  fair fight - the orange buggy measured at the SAME pose in the SAME run,
                  so the comparison is not against a number from another day.

T3 matters because the earlier baselines (0.0395 / 0.1272) came from a different
script at a different pose. Re-measuring both here is the only way the claim
"the GLB detects better" survives contact with a sceptic.

    python experiments/probe_glb_traffic_fitness.py
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

# Four vehicles that scored well and read as four different colours.
FLEET = [("taxi.glb", "a yellow car"), ("police.glb", "a white car"),
         ("sedan.glb", "a red car"), ("van.glb", "a car")]


async def run(args) -> int:
    import torch
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

    def pose_at(x, y, z=0.0, yaw_deg=0.0):
        h = math.radians(yaw_deg) / 2.0
        return Pose({"translation": Vector3({"x": x, "y": y, "z": z}),
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
            out, threshold=0.0, target_sizes=torch.tensor([[H, W]]).to("cuda"))[0]
        sc, bx = res["scores"], res["boxes"]
        if not len(sc):
            return 0.0, None
        i = int(sc.argmax())
        return float(sc[i]), [float(v) for v in bx[i].tolist()]

    def measure(img, query):
        s, box = score(img, query)
        cw = fv.colour_word(query)
        cm = fv.colour_match(np.asarray(img), box, cw) if (box and cw) else None
        return {"query": query, "score": round(s, 4), "colour_word": cw,
                "colour_match": None if cm is None else round(cm, 3)}

    drone.enable_api_control()
    drone.arm()
    await (await drone.takeoff_async())
    await (await drone.move_to_position_async(args.drone_x, args.drone_y,
                                              -args.alt, velocity=3.0))
    await asyncio.sleep(1.0)
    bearing = math.atan2(args.y - args.drone_y, args.x - args.drone_x)
    await (await drone.rotate_to_yaw_async(yaw=bearing))
    await asyncio.sleep(1.5)

    out_dir = ROOT / "experiments" / "out" / "glb_fitness"
    out_dir.mkdir(parents=True, exist_ok=True)
    report: dict = {}

    # --- T3: the incumbent, measured here and now ----------------------------
    try:
        buggy = world.spawn_object("fit_buggy", "SM_Offroad_Body",
                                   pose_at(args.x, args.y), [1.0] * 3, False)
        try:
            world.set_object_material(buggy, "/Game/Geometry/Materials/M_Orange")
        except Exception as e:
            print(f"[T3] material failed: {type(e).__name__}: {e}")
        await asyncio.sleep(1.0)
        img = await frame()
        if img is not None:
            img.save(out_dir / "incumbent_buggy.png")
            report["incumbent"] = {"asset": "SM_Offroad_Body+M_Orange",
                                   "a_car": measure(img, "a car"),
                                   "coloured": measure(img, "an orange car")}
            print(f"[T3] incumbent buggy -> {json.dumps(report['incumbent'])}")
        world.destroy_object(buggy)
    except Exception as e:
        print(f"[T3] incumbent SPAWN FAILED {type(e).__name__}: {e}")
        report["incumbent"] = {"error": f"{type(e).__name__}: {e}"}
    await asyncio.sleep(0.5)

    # --- T1: movability, the go/no-go ----------------------------------------
    taxi = glb_dir / "taxi.glb"
    name = world.spawn_object_from_file("fit_move", "gltf", taxi.read_bytes(), True,
                                        pose_at(args.x, args.y), [args.scale] * 3, False)
    await asyncio.sleep(1.0)
    moved, failed = [], []
    for i in range(args.steps):
        y = args.y + 0.4 * i          # 0.4 m per step == 4 m/s at the 10 Hz rate
        try:
            world.set_object_pose(name, pose_at(args.x, y), True)
        except Exception as e:
            failed.append(f"step {i}: {type(e).__name__}: {e}")
            continue
        await asyncio.sleep(0.1)
        try:
            got = world.get_object_pose(name)
            gy = got["translation"]["y"] if isinstance(got, dict) else None
            moved.append(None if gy is None else round(float(gy) - y, 3))
        except Exception as e:
            failed.append(f"readback {i}: {type(e).__name__}: {e}")
    errs = [m for m in moved if m is not None]
    report["movable"] = {
        "steps": args.steps, "readbacks": len(errs),
        "max_abs_error_m": None if not errs else round(max(abs(e) for e in errs), 3),
        "failures": failed[:5], "n_failures": len(failed),
    }
    print(f"[T1] movable -> {json.dumps(report['movable'])}")
    img = await frame()
    if img is not None:
        img.save(out_dir / "after_move.png")
    world.destroy_object(name)
    await asyncio.sleep(0.5)

    # --- T2: four at once, each measured for its own colour -------------------
    spawned, fleet_rows = [], []
    for i, (fname, query) in enumerate(FLEET):
        p = glb_dir / fname
        try:
            n = world.spawn_object_from_file(
                f"fit_fleet_{i}", "gltf", p.read_bytes(), True,
                pose_at(args.x - 3.0 + 3.0 * i, args.y + 4.0 * i),
                [args.scale] * 3, False)
            spawned.append((n, fname, query))
        except Exception as e:
            fleet_rows.append({"model": fname, "error": f"{type(e).__name__}: {e}"})
    await asyncio.sleep(1.5)
    img = await frame()
    if img is not None:
        img.save(out_dir / "fleet_of_four.png")
        for _, fname, query in spawned:
            fleet_rows.append({"model": fname, **measure(img, query)})
    report["fleet"] = {"spawned": len(spawned), "rows": fleet_rows}
    print(f"[T2] fleet of {len(spawned)} -> {json.dumps(fleet_rows)}")
    for n, _, _ in spawned:
        try:
            world.destroy_object(n)
        except Exception:
            pass

    (out_dir / "fitness.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\n[fitness] -> {out_dir}")
    client.disconnect()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--glb-dir", default="D:/models/kenney_car-kit/glb")
    ap.add_argument("--x", type=float, default=35.0)
    ap.add_argument("--y", type=float, default=2.0)
    ap.add_argument("--drone-x", type=float, default=35.0)
    ap.add_argument("--drone-y", type=float, default=-20.0)
    ap.add_argument("--alt", type=float, default=9.0)
    ap.add_argument("--scale", type=float, default=2.0)
    ap.add_argument("--steps", type=int, default=20)
    return asyncio.run(run(ap.parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
