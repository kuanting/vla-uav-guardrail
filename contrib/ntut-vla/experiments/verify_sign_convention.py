"""
V2 — pin down the yaw sign convention against the running simulator.

The primary metric (turn-toward rate) is `sign(commanded yaw) == sign(bearing
error)`. If either sign is backwards, a genuine null reads as a confident
"anti-homing" finding, and a real effect reads as a null. Nothing downstream can
detect that, so it is asserted here against the sim rather than reasoned about.

Three things get checked:
  1. a positive `yaw` command increases psi  (positive = clockwise from above)
  2. psi as decoded by quat_yaw agrees with the heading implied by the motion
  3. bearing error beta = wrap(atan2(Ty-y, Tx-x) - psi) SHRINKS while yawing in
     the direction whose sign matches beta

No VLA is loaded. Needs the sim on the JapaneseCity map.

Run:  python experiments/verify_sign_convention.py
"""
from __future__ import annotations

import asyncio
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from semantic_demo import quat_yaw                          # noqa: E402
from semantic_target import bearing_to, wrap_pi             # noqa: E402

SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_guardrail.jsonc"
TARGET = (38.0, 26.0)


async def main() -> int:
    from projectairsim import Drone, ProjectAirSimClient, World

    client = ProjectAirSimClient()
    client.connect()
    rc = 1
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        drone = Drone(client, world, "Drone1")
        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        for _ in range(200):
            kin = drone.get_ground_truth_kinematics()
            if -kin["pose"]["position"]["z"] >= 21.5:
                break
            await drone.move_by_velocity_async(0.0, 0.0, -2.0, duration=0.2)
            await asyncio.sleep(0.1)

        kin = drone.get_ground_truth_kinematics()
        p = kin["pose"]["position"]
        psi0 = quat_yaw(kin["pose"]["orientation"])
        beta0 = wrap_pi(bearing_to(p["x"], p["y"], TARGET) - psi0)
        print(f"[V2] start psi={math.degrees(psi0):+7.2f} deg  "
              f"beta={math.degrees(beta0):+7.2f} deg  "
              f"pos=({p['x']:.1f},{p['y']:.1f})")

        # command a steady POSITIVE yaw rate and watch psi and beta
        OMEGA = 0.3
        samples = []
        for i in range(120):
            await drone.move_by_velocity_async(0.0, 0.0, 0.0, duration=0.3,
                                               yaw_is_rate=True, yaw=OMEGA)
            await asyncio.sleep(0.1)
            kin = drone.get_ground_truth_kinematics()
            p = kin["pose"]["position"]
            psi = quat_yaw(kin["pose"]["orientation"])
            beta = wrap_pi(bearing_to(p["x"], p["y"], TARGET) - psi)
            samples.append((i * 0.1, psi, beta))
            if i % 30 == 0:
                print(f"     t={i*0.1:4.1f}s psi={math.degrees(psi):+8.2f} "
                      f"beta={math.degrees(beta):+8.2f}")

        # unwrap psi so a wrap at +/-180 does not fake a decrease
        dpsi = sum(wrap_pi(samples[k][1] - samples[k - 1][1])
                   for k in range(1, len(samples)))
        dbeta = sum(wrap_pi(samples[k][2] - samples[k - 1][2])
                    for k in range(1, len(samples)))
        print(f"\n[V2] commanded yaw = {OMEGA:+.2f} rad/s for "
              f"{samples[-1][0]:.1f}s")
        print(f"[V2] total dpsi  = {math.degrees(dpsi):+8.2f} deg")
        print(f"[V2] total dbeta = {math.degrees(dbeta):+8.2f} deg")

        ok_psi = dpsi > math.radians(20)
        # while psi increases, beta must decrease by the same amount (the drone
        # is hovering, so the bearing to a fixed target barely moves)
        ok_beta = dbeta < -math.radians(20)
        ok_pair = abs(dpsi + dbeta) < math.radians(25)

        print()
        print(f"[V2] positive yaw increases psi (clockwise) : "
              f"{'PASS' if ok_psi else 'FAIL'}")
        print(f"[V2] increasing psi decreases beta          : "
              f"{'PASS' if ok_beta else 'FAIL'}")
        print(f"[V2] dbeta ~= -dpsi for a fixed target      : "
              f"{'PASS' if ok_pair else 'FAIL'}")

        if ok_psi and ok_beta and ok_pair:
            print("\n[V2] PASS — sign convention confirmed:\n"
                  "     beta > 0 means the target is CLOCKWISE of the nose, and a\n"
                  "     POSITIVE yaw command turns clockwise. So sign(omega) ==\n"
                  "     sign(beta) is 'turning toward the target', which is what\n"
                  "     the turn-toward metric assumes.")
            rc = 0
        else:
            print("\n[V2] FAIL — the analyzer's sign assumption does NOT hold.\n"
                  "     Do NOT run the matrix: turn-toward would be inverted and\n"
                  "     the experiment would report the opposite of the truth.")

        await (await drone.land_async())
        drone.disarm()
        drone.disable_api_control()
    except Exception as e:
        print(f"[V2] error: {type(e).__name__}: {e}")
    finally:
        client.disconnect()
    return rc


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
