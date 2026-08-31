"""
Look at the car before trusting any number that depends on it.

Spawns the moving car, hovers the drone at several altitudes with the nose on it,
and saves a front-camera frame at each. If the object is not recognisably a
vehicle-sized coloured thing on a road in these images, "follow the car" is
untestable and every downstream metric is measuring noise.

Also verifies that teleporting actually moves it: the car is sampled at two times
and the reported poses are compared.

Run:  python experiments/probe_moving_car.py
Out:  experiments/out/car_probe/alt_<h>.png
"""
from __future__ import annotations

import asyncio
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import moving_car                                            # noqa: E402
import semantic_target                                       # noqa: E402
from semantic_demo import SemanticObs, quat_yaw              # noqa: E402
from semantic_seek import align_start_yaw                    # noqa: E402

SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_guardrail.jsonc"
OUT = ROOT / "experiments" / "out" / "car_probe"
ALTS = [12.0, 18.0, 25.0]


async def climb_to(drone, alt: float) -> None:
    for _ in range(400):
        kin = drone.get_ground_truth_kinematics()
        up = -kin["pose"]["position"]["z"]
        if abs(up - alt) < 0.6:
            return
        v = max(-2.5, min(2.5, (alt - up) * 0.8))
        await drone.move_by_velocity_async(0.0, 0.0, -v, duration=0.2)
        await asyncio.sleep(0.1)


async def main() -> int:
    from projectairsim import Drone, ProjectAirSimClient, World

    OUT.mkdir(parents=True, exist_ok=True)
    obs = SemanticObs()
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

        car = moving_car.MovingCar(world, speed_mps=3.0)
        car.spawn()

        # does teleporting actually move it?
        p0 = car.update(0.0)
        await asyncio.sleep(0.5)
        p1 = car.update(5.0)
        moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        print(f"[probe] car at t=0 {p0}, at t=5 {p1}  -> moved {moved:.1f} m "
              f"(expected {car.speed*5:.1f})")
        try:
            back = world.get_object_pose(car.actual_name).translation
            bx = getattr(back, "x", None) or back["x"]
            by = getattr(back, "y", None) or back["y"]
            print(f"[probe] sim reports the car at ({bx:.1f}, {by:.1f}) "
                  f"-> teleport {'CONFIRMED' if abs(by-p1[1])<1.0 else 'NOT APPLIED'}")
        except Exception as e:
            print(f"[probe] pose read-back failed ({type(e).__name__}: {e})")

        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        # Hold one altitude and sample the car at several ranges. Sweeping
        # altitude while the car sat 27 m away produced three near-identical
        # frames in which it was a few pale pixels at the bottom edge — the
        # variable that matters for recognisability is RANGE, not height.
        await climb_to(drone, 9.0)
        kin = drone.get_ground_truth_kinematics()
        p = kin["pose"]["position"]
        for rng_want in (15.0, 22.0, 30.0):
            # put the car that far up the street from the drone
            cy = p["y"] + math.sqrt(max(0.0, rng_want ** 2 - (38.0 - p["x"]) ** 2))
            t = (cy - car.route[0][1]) / car.speed
            car.update(max(0.0, t))
            await asyncio.sleep(0.6)
            cx, cy = car.pos
            psi = semantic_target.wrap_pi(
                semantic_target.bearing_to(p["x"], p["y"], (cx, cy)))
            await align_start_yaw(drone, psi)
            for _ in range(30):
                await asyncio.sleep(0.1)
                if obs.get_obs() is not None:
                    break
            got = obs.get_obs()
            if got is None:
                print(f"[probe] range {rng_want}: no frame")
                continue
            front, down, _ = got
            front.save(OUT / f"rng_{int(rng_want)}_front.png")
            down.save(OUT / f"rng_{int(rng_want)}_down.png")
            rng = math.hypot(p["x"] - cx, p["y"] - cy)
            px = 4.5 / rng * (224 / math.radians(90))
            dep = math.degrees(math.atan2(9.0, rng))
            print(f"[probe] range {rng:5.1f} m  car ~{px:4.1f} px wide  "
                  f"depression {dep:4.1f} deg (front cam half-FOV 29.2) "
                  f"-> rng_{int(rng_want)}_front.png")

        await (await drone.land_async())
        drone.disarm()
        drone.disable_api_control()
        car.destroy()
    except Exception as e:
        print(f"[probe] error: {type(e).__name__}: {e}")
        return 1
    finally:
        client.disconnect()
    print("\nOPEN THE IMAGES. If the car is not obviously a vehicle-sized "
          "coloured object on a road, the follow-the-car mission is untestable.")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
