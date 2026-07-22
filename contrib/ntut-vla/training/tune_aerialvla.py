"""
Auto-tuning loop for the AerialVLA adapter — fly, score, adjust, repeat.

Honest framing: this does NOT retrain the 7B weights (that needs the TravelUAV
dataset + days of GPU). It tunes the ADAPTER around the frozen model — yaw
gain, mission-direction assist (goal_blend), smoothing — exactly the knobs a
deployment engineer would tune, and scores each full closed-loop flight:

    score gate:  reached target  AND  NFZ 0.0s  AND  efficiency >= EFF_GATE
    efficiency = straight-line distance / actual path length (SPL-style)

Runs trials sequentially, restarting AirSimNH between flights. Keeps the best
config in models/aerialvla_adapter_best.json and logs every trial.

Run (vla-real env):  python training/tune_aerialvla.py
"""
from __future__ import annotations

import json
import socket
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PY = r"C:\Users\natha\.conda\envs\vla-real\python.exe"
SIM = r"D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe"
LOG = ROOT / "training" / "aerialvla_tune_log.md"
BEST = ROOT / "models" / "aerialvla_adapter_best.json"

EFF_GATE = 0.80
MAX_TRIALS = 12

# Staged search: pure VLA baseline first, then increasing mission assist and
# calmer yaw. dv_h slightly raised where assist is strong (straighter = can
# afford faster transitions).
TRIALS = [
    {"yaw_gain": 1.0, "goal_blend": 0.0,  "dv_h": 0.25, "dv_z": 0.15},
    {"yaw_gain": 1.0, "goal_blend": 0.25, "dv_h": 0.25, "dv_z": 0.15},
    {"yaw_gain": 1.0, "goal_blend": 0.4,  "dv_h": 0.25, "dv_z": 0.15},
    {"yaw_gain": 0.6, "goal_blend": 0.4,  "dv_h": 0.25, "dv_z": 0.15},
    {"yaw_gain": 0.6, "goal_blend": 0.55, "dv_h": 0.3,  "dv_z": 0.15},
    {"yaw_gain": 0.4, "goal_blend": 0.55, "dv_h": 0.3,  "dv_z": 0.15},
    {"yaw_gain": 0.4, "goal_blend": 0.7,  "dv_h": 0.3,  "dv_z": 0.15},
    {"yaw_gain": 0.6, "goal_blend": 0.7,  "dv_h": 0.35, "dv_z": 0.15},
    {"yaw_gain": 0.3, "goal_blend": 0.8,  "dv_h": 0.35, "dv_z": 0.15},
    {"yaw_gain": 0.5, "goal_blend": 0.6,  "dv_h": 0.4,  "dv_z": 0.2},
    {"yaw_gain": 0.7, "goal_blend": 0.5,  "dv_h": 0.35, "dv_z": 0.15},
    {"yaw_gain": 0.5, "goal_blend": 0.45, "dv_h": 0.3,  "dv_z": 0.15},
]


def log(msg: str) -> None:
    line = f"{datetime.now():%H:%M:%S} {msg}"
    print(line, flush=True)
    with LOG.open("a", encoding="utf-8") as f:
        f.write(line + "\n")


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


def run_trial(i: int, p: dict) -> dict | None:
    tag = f"tune_{i:02d}"
    cmd = [PY, str(ROOT / "demo" / "aerialvla_demo.py"),
           "--tag", tag, "--max-s", "75",
           "--yaw-gain", str(p["yaw_gain"]), "--goal-blend", str(p["goal_blend"]),
           "--dv-h", str(p["dv_h"]), "--dv-z", str(p["dv_z"])]
    r = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True,
                       timeout=600)
    mfile = ROOT / "demo" / "out" / tag / "metrics.json"
    if r.returncode != 0 or not mfile.exists():
        tail = (r.stdout or "")[-400:] + (r.stderr or "")[-400:]
        log(f"trial {i}: CRASHED ({tail.strip()[-200:]!r})")
        return None
    return json.loads(mfile.read_text(encoding="utf-8"))


def score(m: dict) -> float:
    if not m["reached"] or m["nfz_s"] > 0:
        return 0.0
    return m["efficiency"]


def main() -> int:
    LOG.write_text(f"# AerialVLA adapter tuning — {datetime.now():%Y-%m-%d %H:%M}\n\n",
                   encoding="utf-8")
    best, best_s = None, -1.0
    for i, p in enumerate(TRIALS[:MAX_TRIALS], 1):
        log(f"trial {i}/{len(TRIALS)}: {p}")
        if not restart_sim():
            log("sim failed to start — abort")
            return 1
        try:
            m = run_trial(i, p)
        except subprocess.TimeoutExpired:
            log(f"trial {i}: TIMEOUT")
            continue
        if m is None:
            continue
        s = score(m)
        log(f"trial {i}: reached={m['reached']} eff={m['efficiency']} "
            f"nfz={m['nfz_s']} interv={m['interventions']} "
            f"path={m['path_len_m']}m -> score {s:.3f}")
        if s > best_s:
            best, best_s = m, s
            BEST.write_text(json.dumps(m, indent=1), encoding="utf-8")
            log(f"  ** new best (score {s:.3f}) -> {BEST.name}")
        if s >= EFF_GATE:
            log(f"GATE PASSED (eff {s:.3f} >= {EFF_GATE}) after {i} trials")
            break
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Get-Process -Name AirSimNH -ErrorAction SilentlyContinue "
                    "| Stop-Process -Force"], capture_output=True)
    if best is None:
        log("NO successful trial")
        return 1
    log(f"DONE. best: {best['params']} eff={best['efficiency']} "
        f"interv={best['interventions']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
