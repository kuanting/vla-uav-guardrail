"""Does `spawn_object_from_file` actually put geometry in the world?

The first two probe runs both produced frames with no candidate vehicle in them.
The first was my framing bug (the drone never took off). The second took off and
the scores rose to ~0.085 — but every frame is still the bare street, so those
scores are OWL-ViT finding the city's OWN parked cars, not the spawned model.
Scoring the background and calling it a result would be worse than no result.

So stop looking at pixels and ask the sim. For each candidate:

  * does `get_object_pose` return the pose we asked for?  (object exists)
  * does `get_3d_bounding_box` return a non-degenerate extent?  (geometry loaded)

Those two separate the three live hypotheses:

  A. spawn silently no-ops        -> pose lookup fails
  B. geometry never loads         -> pose fine, bbox zero/absent
  C. it IS there, wrongly placed  -> pose and bbox fine, so sweep scale and z

A positive control runs first: `1M_Cube`, spawned through the ordinary
`spawn_object` path, at the same pose. If the cube is invisible too then the
pose is wrong and no GLB conclusion can be drawn at all.

    python experiments/diagnose_glb_spawn.py
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


def _describe(world, name: str) -> dict:
    """Everything the sim will tell us about an object, without trusting any of it."""
    out: dict = {"name": name}
    for key, fn in (("pose", world.get_object_pose), ("bbox", world.get_3d_bounding_box)):
        try:
            out[key] = fn(name)
        except Exception as e:
            out[key] = f"ERROR {type(e).__name__}: {e}"
    return out


async def run(args) -> int:
    from projectairsim import Drone, ProjectAirSimClient, World
    from projectairsim.types import Pose, Quaternion, Vector3
    from semantic_demo import SemanticObs

    client = ProjectAirSimClient()
    client.connect()
    world = World(client, SCENE, delay_after_load_sec=2, sim_config_path=SIM_CONFIG_DIR)
    drone = Drone(client, world, "Drone1")
    obs = SemanticObs()
    client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                     lambda _, m: obs.put_front(m))

    def pose_at(x, y, z):
        return Pose({"translation": Vector3({"x": x, "y": y, "z": z}),
                     "rotation": Quaternion({"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}),
                     "frame_id": "DEFAULT_ID"})

    async def frame():
        for _ in range(40):
            await asyncio.sleep(0.25)
            img = obs.get_front_native()
            if img is not None:
                return img
        return None

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
    out_dir = ROOT / "experiments" / "out" / "glb_diag"
    out_dir.mkdir(parents=True, exist_ok=True)
    base = await frame()
    if base is not None:
        base.save(out_dir / "00_empty.png")
    base_arr = None if base is None else np.asarray(base).astype(np.int16)

    def changed(img) -> float:
        """Fraction of pixels that moved by more than render jitter.

        Frame hashes are useless here - every capture differs by a few LSBs from
        temporal AA. A spawned 4 m car at 22 m fills thousands of pixels, so a
        real spawn is unmistakable against a >12/255 threshold."""
        if base_arr is None or img is None:
            return float("nan")
        d = np.abs(np.asarray(img).astype(np.int16) - base_arr).max(axis=2)
        return float((d > 12).mean())

    report = []

    # --- positive control: the ordinary spawn path, same pose -----------------
    try:
        cube = world.spawn_object("diag_cube", "1M_Cube",
                                  pose_at(args.x, args.y, args.z),
                                  [4.0, 4.0, 4.0], False)
        await asyncio.sleep(1.0)
        img = await frame()
        if img is not None:
            img.save(out_dir / "01_cube.png")
        info = _describe(world, cube)
        info["pixels_changed"] = round(changed(img), 4)
        print(f"[control] 1M_Cube -> {json.dumps(info, default=str)[:400]}")
        report.append({"stage": "control_cube", **{k: str(v) for k, v in info.items()}})
        world.destroy_object(cube)
    except Exception as e:
        print(f"[control] 1M_Cube SPAWN FAILED {type(e).__name__}: {e}")
        report.append({"stage": "control_cube", "error": f"{type(e).__name__}: {e}"})
    await asyncio.sleep(0.5)

    # --- the candidate, swept over scale and height ---------------------------
    glb = pathlib.Path(args.glb)
    data = glb.read_bytes()
    print(f"[glb] {glb.name}  {len(data)} bytes")

    for scale in args.scales:
        for z in args.zs:
            tag = f"{glb.stem}_s{scale:g}_z{z:g}"
            try:
                name = world.spawn_object_from_file(
                    f"diag_{tag}", "gltf", data, True,
                    pose_at(args.x, args.y, z), [scale] * 3, False)
            except Exception as e:
                print(f"  {tag:28s} SPAWN FAILED {type(e).__name__}: {e}")
                report.append({"stage": tag, "error": f"{type(e).__name__}: {e}"})
                continue
            await asyncio.sleep(1.0)
            img = await frame()
            if img is not None:
                img.save(out_dir / f"{tag}.png")
            info = _describe(world, name)
            pix = changed(img)
            info["pixels_changed"] = round(pix, 4)
            print(f"  {tag:28s} pixels_changed={pix:.4f}  "
                  f"pose={str(info.get('pose'))[:90]}  bbox={str(info.get('bbox'))[:90]}")
            report.append({"stage": tag, **{k: str(v) for k, v in info.items()}})
            try:
                world.destroy_object(name)
            except Exception:
                pass
            await asyncio.sleep(0.4)

    (out_dir / "diag.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\n[diag] -> {out_dir}")
    print("[diag] read 01_cube.png first: if the CUBE is invisible the pose is "
          "wrong and nothing here says anything about the GLBs.")
    client.disconnect()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--glb", default="D:/models/kenney_car-kit/glb/sedan.glb")
    ap.add_argument("--x", type=float, default=35.0)
    ap.add_argument("--y", type=float, default=2.0)
    ap.add_argument("--z", type=float, default=0.0)
    ap.add_argument("--drone-x", type=float, default=35.0)
    ap.add_argument("--drone-y", type=float, default=-20.0)
    ap.add_argument("--alt", type=float, default=9.0)
    # Kenney units are unknown, so bracket widely rather than guessing twice.
    ap.add_argument("--scales", type=float, nargs="+", default=[2.0, 10.0, 50.0])
    # z is NED-down in this world: negative is ABOVE ground. If 0.0 buries the
    # car in the asphalt, a raised copy will show it.
    ap.add_argument("--zs", type=float, nargs="+", default=[0.0, -3.0])
    return asyncio.run(run(ap.parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
