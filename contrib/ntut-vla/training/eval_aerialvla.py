"""
Closed-loop evaluation of an AerialVLA adapter — N randomized episodes,
PURE VLA (goal_blend 0) so the score measures the MODEL, not the guidance.

Each episode: restart-free (fresh subprocess per flight, sim restarted every
`--sim-every` episodes), random target bearing/distance, fixed policy. Metrics
aggregated to training/eval_<name>.json:

    reached_rate, mean_efficiency, mean_interventions, nfz_total_s

Run (vla-real env, sim NOT required to be running - handles it):
    python training/eval_aerialvla.py --adapter D:/models/aerialvla-lora/aero_vla --name baseline
    python training/eval_aerialvla.py --adapter D:/models/aerialvla-ft/run1/epoch1 --name ft_e1
"""
from __future__ import annotations

import argparse
import json
import math
import random
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PY = r"C:\Users\natha\.conda\envs\vla-real\python.exe"
SIM = r"D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe"


def port_open(port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


def restart_sim() -> bool:
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Get-Process -Name AirSimNH -ErrorAction SilentlyContinue "
                    "| Stop-Process -Force"], capture_output=True)
    time.sleep(4)
    subprocess.Popen([SIM, "-ResX=960", "-ResY=540", "-windowed"])
    for _ in range(60):
        time.sleep(3)
        if port_open(41451):
            time.sleep(3)
            return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--adapter", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--goal-blend", type=float, default=0.0)
    ap.add_argument("--yaw-gain", type=float, default=1.0)
    args = ap.parse_args()
    random.seed(args.seed)

    results = []
    for i in range(args.episodes):
        if not restart_sim():
            print("[eval] sim failed to start")
            return 1
        bearing = random.uniform(-math.pi, math.pi)
        dist = random.uniform(40, 60)
        tx, ty = dist * math.cos(bearing), dist * math.sin(bearing)
        tag = f"eval_{args.name}_{i}"
        cmd = [PY, str(ROOT / "demo" / "aerialvla_demo.py"),
               "--adapter", args.adapter, "--tag", tag, "--max-s", "75",
               "--goal-blend", str(args.goal_blend),
               "--yaw-gain", str(args.yaw_gain),
               "--command", f"fly to ({tx:.0f}, {ty:.0f}) at 6 m/s altitude 20"]
        r = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True,
                           timeout=600)
        mfile = ROOT / "demo" / "out" / tag / "metrics.json"
        if r.returncode != 0 or not mfile.exists():
            print(f"[eval] ep {i} CRASHED: {(r.stderr or '')[-200:]}", flush=True)
            results.append({"reached": False, "efficiency": 0.0,
                            "interventions": 0, "nfz_s": 0.0, "crashed": True})
            continue
        m = json.loads(mfile.read_text(encoding="utf-8"))
        m["target"] = [round(tx), round(ty)]
        results.append(m)
        print(f"[eval] ep {i}: reached={m['reached']} eff={m['efficiency']} "
              f"interv={m['interventions']} nfz={m['nfz_s']}", flush=True)

    n = len(results)
    agg = {
        "name": args.name,
        "adapter": args.adapter,
        "episodes": n,
        "reached_rate": sum(1 for r in results if r.get("reached")) / n,
        "mean_efficiency": round(sum(r.get("efficiency", 0) for r in results) / n, 3),
        "mean_interventions": round(sum(r.get("interventions", 0) for r in results) / n, 1),
        "nfz_total_s": round(sum(r.get("nfz_s", 0) for r in results), 1),
        "results": results,
    }
    out = ROOT / "training" / f"eval_{args.name}.json"
    out.write_text(json.dumps(agg, indent=1), encoding="utf-8")
    print(f"[eval] {args.name}: reached {agg['reached_rate']:.0%} "
          f"eff {agg['mean_efficiency']} interv {agg['mean_interventions']} "
          f"nfz {agg['nfz_total_s']}s -> {out.name}")
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Get-Process -Name AirSimNH -ErrorAction SilentlyContinue "
                    "| Stop-Process -Force"], capture_output=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
