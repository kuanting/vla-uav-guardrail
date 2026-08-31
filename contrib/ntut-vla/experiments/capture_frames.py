"""
Capture real 224x448 front+down mosaics at fixed poses, with and without the
semantic target, so the image-vs-hint ablation can run offline.

Flying 16 missions to discover what the model responds to is the expensive way
round. The same question — does AerialVLA's action depend on the pixels, on the
object phrase, or only on the direction hint? — is answerable from a handful of
frames and a few hundred forward passes, with no simulator in the loop and no
flight-to-flight variance to average out.

Poses are the real experimental ones: the spawn point at cruise altitude, headed
so the target sits right (+35 deg bearing offset), left (-35), or centred (0).

Run:  python experiments/capture_frames.py
Out:  experiments/out/frames/<scene>_<beta>.png  + frames.json
"""
from __future__ import annotations

import asyncio
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import semantic_target                                      # noqa: E402
from semantic_demo import SemanticObs, quat_yaw             # noqa: E402
from semantic_seek import align_start_yaw                   # noqa: E402

SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_guardrail.jsonc"
TARGET = (38.0, 26.0)
CRUISE = 22.0
BETAS = [35.0, 0.0, -35.0]
OUTDIR = ROOT / "experiments" / "out" / "frames"


def mosaic(front, down):
    from PIL import Image
    m = Image.new("RGB", (224, 448), (0, 0, 0))
    m.paste(front, (0, 0))
    m.paste(down, (0, 224))
    return m


async def capture(drone, obs, tag: str, beta: float, meta: list) -> None:
    p = drone.get_ground_truth_kinematics()["pose"]["position"]
    psi_t = semantic_target.wrap_pi(
        semantic_target.bearing_to(p["x"], p["y"], TARGET) - math.radians(beta))
    await align_start_yaw(drone, psi_t)
    for _ in range(40):
        await asyncio.sleep(0.1)
        if obs.get_obs() is not None:
            break
    got = obs.get_obs()
    if got is None:
        print(f"[capture] {tag} beta={beta}: no frame")
        return
    front, down, _ = got
    kin = drone.get_ground_truth_kinematics()
    pp = kin["pose"]["position"]
    psi = quat_yaw(kin["pose"]["orientation"])
    name = f"{tag}_{int(beta):+03d}.png"
    mosaic(front, down).save(OUTDIR / name)
    meta.append({"file": name, "scene": tag, "beta_cmd": beta,
                 "x": pp["x"], "y": pp["y"], "up": -pp["z"], "psi": psi,
                 "beta_actual_deg": math.degrees(semantic_target.wrap_pi(
                     semantic_target.bearing_to(pp["x"], pp["y"], TARGET) - psi)),
                 "target_present": tag == "present"})
    print(f"[capture] {name}  beta_actual={meta[-1]['beta_actual_deg']:+.1f} deg")


async def main() -> int:
    from projectairsim import Drone, ProjectAirSimClient, World

    OUTDIR.mkdir(parents=True, exist_ok=True)
    obs = SemanticObs()
    client = ProjectAirSimClient()
    client.connect()
    meta: list = []
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        drone = Drone(client, world, "Drone1")
        client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                         lambda _, m: obs.put_front(m))
        client.subscribe(drone.sensors["DownCamera"]["scene_camera"],
                         lambda _, m: obs.put_down(m))
        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        for _ in range(300):
            kin = drone.get_ground_truth_kinematics()
            if -kin["pose"]["position"]["z"] >= CRUISE - 0.5:
                break
            await drone.move_by_velocity_async(0.0, 0.0, -2.0, duration=0.2)
            await asyncio.sleep(0.1)

        # absent FIRST, so the two scenes differ only by the object
        for beta in BETAS:
            await capture(drone, obs, "absent", beta, meta)
        spec = semantic_target.pick_spec(world)
        name, truth, _ = semantic_target.spawn_target(world, spec, *TARGET)
        await asyncio.sleep(1.0)
        for beta in BETAS:
            await capture(drone, obs, "present", beta, meta)
        semantic_target.destroy_target(world, name)

        (OUTDIR / "frames.json").write_text(json.dumps(
            {"target": list(TARGET), "target_truth": truth, "cruise": CRUISE,
             "desc_match": spec.desc_match, "desc_mismatch": spec.desc_mismatch,
             "frames": meta}, indent=1), encoding="utf-8")
        print(f"[capture] {len(meta)} frames -> {OUTDIR}")

        await (await drone.land_async())
        drone.disarm()
        drone.disable_api_control()
    except Exception as e:
        print(f"[capture] error: {type(e).__name__}: {e}")
        return 1
    finally:
        client.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
