"""
Auto-tuning loop for PATH SMOOTHNESS on the edge-patrol NFZ scenario.

Flies the same scenario repeatedly, varying the pure-pursuit params
(lookahead, follow_blend), and scores how tightly the flown path hugs the plan
(mean deviation from the planned polyline + flown/plan length ratio). Keeps the
best; stops when the gate passes:  NFZ 0  AND  mean_dev <= 1.8 m  AND
len_ratio <= 1.20  AND  reached.

Sequential (one sim) — restarts the sim between flights.
Run:  python training/tune_path.py
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
UE = r"C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe"
UPROJ = str(ROOT / "PASBlocks" / "Blocks.uproject")
MAP = "/Game/JapaneseCity/Maps/Demo_day"
LOG = ROOT / "training" / "path_tune_log.md"
BEST = ROOT / "models" / "path_best.json"

ROUTE = "42,42; 42,-42; -42,-42; -42,42"
POLICY = str(ROOT / "policies" / "edge_patrol.yaml")
CITYMAP = str(ROOT / "demo" / "out" / "citymap" / "occ_day.npz")

DEV_GATE, RATIO_GATE = 1.8, 1.20
TRIALS = [
    {"lookahead": 8.0, "follow_blend": 0.85},
    {"lookahead": 6.0, "follow_blend": 0.90},
    {"lookahead": 5.0, "follow_blend": 0.92},
    {"lookahead": 4.0, "follow_blend": 0.92},
    {"lookahead": 5.0, "follow_blend": 0.95},
    {"lookahead": 6.0, "follow_blend": 0.95},
    {"lookahead": 4.0, "follow_blend": 0.88},
    {"lookahead": 7.0, "follow_blend": 0.90},
    {"lookahead": 3.0, "follow_blend": 0.90},
    {"lookahead": 5.0, "follow_blend": 0.88},
]


def log(msg):
    line = f"{datetime.now():%H:%M:%S} {msg}"
    print(line, flush=True)
    with LOG.open("a", encoding="utf-8") as f:
        f.write(line + "\n")


def port_open(p):
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", p)) == 0


def restart_sim():
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Get-Process -Name UnrealEditor,AirSimNH -ErrorAction "
                    "SilentlyContinue | Stop-Process -Force"], capture_output=True)
    time.sleep(4)
    subprocess.Popen([UE, UPROJ, MAP, "-game", "-windowed", "-ResX=1280", "-ResY=720"])
    for _ in range(60):
        time.sleep(6)
        if port_open(8989):
            time.sleep(6)
            return True
    return False


def run_trial(i, p):
    tag = f"pathtune_{i:02d}"
    cmd = [PY, str(ROOT / "demo" / "aerialvla_pas_demo.py"), "--best",
           "--adapter", "D:/models/aerialvla-ft/run2/epoch1",
           "--route", ROUTE, "--policy", POLICY, "--citymap", CITYMAP,
           "--command", "fly to (42, 42) at 6 m/s altitude 45",
           "--clearance", "4", "--tag", tag, "--map-label", f"pathtune {i}",
           "--lookahead", str(p["lookahead"]),
           "--follow-blend", str(p["follow_blend"])]
    try:
        subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True, timeout=700)
    except subprocess.TimeoutExpired:
        log(f"trial {i}: TIMEOUT")
        return None
    mfile = ROOT / "demo" / "out" / tag / "metrics.json"
    if not mfile.exists():
        log(f"trial {i}: no metrics (flight failed)")
        return None
    return json.loads(mfile.read_text(encoding="utf-8"))


def score(m):
    if not m["reached"] or m["nfz_s"] > 0:
        return 1e9
    return m["mean_dev"] + 3.0 * max(0.0, m["len_ratio"] - 1.0)


def main():
    LOG.write_text(f"# path-smoothness tuning {datetime.now():%Y-%m-%d %H:%M}\n\n",
                   encoding="utf-8")
    best, best_s = None, 1e18
    for i, p in enumerate(TRIALS, 1):
        log(f"trial {i}/{len(TRIALS)}: {p}")
        if not restart_sim():
            log("sim failed to start"); return 1
        m = run_trial(i, p)
        if m is None:
            continue
        s = score(m)
        log(f"trial {i}: reached={m['reached']} nfz={m['nfz_s']} "
            f"mean_dev={m['mean_dev']} max_dev={m['max_dev']} "
            f"len_ratio={m['len_ratio']} -> score {s:.2f}")
        if s < best_s:
            best, best_s = m, s
            BEST.write_text(json.dumps(m, indent=1), encoding="utf-8")
            log(f"  ** new best (score {s:.2f}) params {m['params']}")
        if (m["reached"] and m["nfz_s"] == 0 and m["mean_dev"] <= DEV_GATE
                and m["len_ratio"] <= RATIO_GATE):
            log(f"GATE PASSED after {i} trials: mean_dev {m['mean_dev']} "
                f"<= {DEV_GATE}, len_ratio {m['len_ratio']} <= {RATIO_GATE}")
            break
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Get-Process -Name UnrealEditor -ErrorAction SilentlyContinue "
                    "| Stop-Process -Force"], capture_output=True)
    if best is None:
        log("NO successful trial"); return 1
    log(f"DONE. best: {best['params']} mean_dev={best['mean_dev']} "
        f"len_ratio={best['len_ratio']} nfz={best['nfz_s']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
