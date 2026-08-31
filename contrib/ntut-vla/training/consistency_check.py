"""
Consistency check: fly the SAME edge-patrol scenario N times with the locked
defaults and confirm every run is clean & bounded — proves the path-quality fix
is robust run-to-run (the earlier variance was the ESCAPE-loop bug, now fixed).

PASS per run: NFZ 0, reached, mean_dev <= 1.2 m, len_ratio in [0.85, 1.35],
ticks <= 2500 (bounded — no infinite escape).

Run:  python training/consistency_check.py 3
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
LOG = ROOT / "training" / "consistency_log.md"


def log(m):
    line = f"{datetime.now():%H:%M:%S} {m}"
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


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    LOG.write_text(f"# consistency check {datetime.now():%Y-%m-%d %H:%M}\n\n",
                   encoding="utf-8")
    results = []
    for i in range(1, n + 1):
        log(f"run {i}/{n}: restarting sim")
        if not restart_sim():
            log("sim failed"); return 1
        tag = f"consist_{i:02d}"
        cmd = [PY, str(ROOT / "demo" / "aerialvla_pas_demo.py"), "--best",
               "--adapter", "D:/models/aerialvla-ft/run2/epoch1",
               "--route", "42,42; 42,-42; -42,-42; -42,42",
               "--policy", str(ROOT / "policies" / "edge_patrol.yaml"),
               "--citymap", str(ROOT / "demo" / "out" / "citymap" / "occ_day.npz"),
               "--command", "fly to (42, 42) at 6 m/s altitude 45",
               "--clearance", "4", "--tag", tag, "--map-label", f"consist {i}"]
        try:
            subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True, timeout=1100)
        except subprocess.TimeoutExpired:
            log(f"run {i}: TIMEOUT (>1100s) — NOT bounded!"); results.append(None); continue
        mf = ROOT / "demo" / "out" / tag / "metrics.json"
        if not mf.exists():
            log(f"run {i}: no metrics"); results.append(None); continue
        m = json.loads(mf.read_text(encoding="utf-8"))
        ok = (m["reached"] and m["nfz_s"] == 0 and m["mean_dev"] <= 1.2
              and 0.85 <= m["len_ratio"] <= 1.35 and m["ticks"] <= 2500)
        log(f"run {i}: reached={m['reached']} nfz={m['nfz_s']} "
            f"mean_dev={m['mean_dev']} len_ratio={m['len_ratio']} "
            f"ticks={m['ticks']} interv={m['interventions']} -> {'PASS' if ok else 'FAIL'}")
        results.append(ok)
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Get-Process -Name UnrealEditor -ErrorAction SilentlyContinue "
                    "| Stop-Process -Force"], capture_output=True)
    good = sum(1 for r in results if r)
    log(f"CONSISTENCY: {good}/{n} runs clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
