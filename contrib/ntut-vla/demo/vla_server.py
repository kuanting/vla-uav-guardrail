"""
Run the VLA in its own process, so the flight loop cannot starve it.

Measured, on one GPU with a flight already airborne and two 7B models resident
at the same moment:

    same process as the control loop   ~1330 ms/token   (~12 s per action)
    separate process                     225 ms/token   (~2.5 s per action)

Nearly six times faster at the same instant on the same hardware. The cause is
not GPU contention — that test had both models on the card simultaneously — but
the GIL: `generate()` does per-token Python work, and an asyncio loop ticking at
10 Hz with RPC round-trips forces constant interpreter handoffs.

Three cheaper explanations were tested first and all rejected: decoding camera
frames in the subscription callback (made lazy — no change), the unused Chase
camera and depth streams (removed — no change), and camera publish rate
(throttled to 2 Hz — no change).

Protocol is two small JSON files, which is enough at ~0.4 Hz:

    <dir>/vla_cmd.json      written by the flight loop, read here
                            {"object": str, "target": [x, y] | null, "stop": bool}
    <dir>/vla_action.json   written here, read by the flight loop
                            {"seq", "t", "fwd", "down", "yaw", "land", "bins",
                             "px", "py", "psi", "prompt_sha8", "hint_used", ...}

Both are written atomically (tmp + os.replace) so a half-written file is never
observed.

Run directly for debugging; normally `semantic_seek.py --vla-proc` starts it.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")


def atomic_write(path: Path, obj: dict) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj), encoding="utf-8")
    os.replace(tmp, path)


def read_cmd(path: Path, cache: dict) -> dict:
    try:
        mt = path.stat().st_mtime
    except OSError:
        return cache
    if mt == cache.get("_mt"):
        return cache
    try:
        d = json.loads(path.read_text(encoding="utf-8"))
        d["_mt"] = mt
        return d
    except Exception:
        return cache


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="directory for the two JSON files")
    ap.add_argument("--adapter", default="D:/models/aerialvla-lora/aero_vla")
    ap.add_argument("--object", default="")
    ap.add_argument("--scene", default="scene_semantic.jsonc")
    ap.add_argument("--max-new-tokens", type=int, default=12)
    args = ap.parse_args()

    out = Path(args.dir)
    out.mkdir(parents=True, exist_ok=True)
    cmd_path, act_path = out / "vla_cmd.json", out / "vla_action.json"
    inf_log = out / "inference.jsonl"

    from projectairsim import Drone, ProjectAirSimClient, World
    from aerialvla_demo import AerialVLABackend
    from semantic_demo import SemanticObs

    obs = SemanticObs()
    client = ProjectAirSimClient()
    client.connect()
    world = World(client, args.scene, delay_after_load_sec=2,
                  sim_config_path=SIM_CONFIG_DIR)
    drone = Drone(client, world, "Drone1")
    client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                     lambda _, m: obs.put_front(m))
    client.subscribe(drone.sensors["DownCamera"]["scene_camera"],
                     lambda _, m: obs.put_down(m))
    print("[vla-server] subscribed to cameras", flush=True)

    vla = AerialVLABackend(args.object, None, obs_factory=lambda: obs.get_obs,
                           lora_id=args.adapter, inference_log=inf_log)
    vla.max_new_tokens = args.max_new_tokens
    vla.start()
    atomic_write(act_path, {"seq": 0, "ready": True})
    print("[vla-server] model loaded, serving", flush=True)

    cache: dict = {}
    last_seq = -1
    try:
        while True:
            cache = read_cmd(cmd_path, cache)
            if cache.get("stop"):
                break
            # the flight loop owns the pose and the (possibly moving) target
            if cache.get("pose"):
                obs.put_pose(*cache["pose"])
            tgt = cache.get("target")
            vla.target_xy = tuple(tgt) if tgt else None
            if cache.get("object") is not None:
                vla.obj_desc = cache["object"]

            info = vla.latest_full()
            if info["seq"] != last_seq and info["seq"] > 0:
                last_seq = info["seq"]
                atomic_write(act_path, {**info, "ready": True})
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        vla.stop()
        client.disconnect()
        print("[vla-server] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
