"""
Run the pre-registered condition matrix for the VLA/Guardrail coexistence
experiment, one flight at a time, restarting the simulator between every flight.

The matrix, the metric parameters and the pass thresholds all live in
experiments/conditions.yaml. This script only executes it. That separation is
what lets the analyzer assert it scored the same design that was flown.

Three properties matter more than speed here:

  * results append after EVERY flight, so a crash at flight 13 does not cost the
    first twelve;
  * the order is a seeded shuffle, never block-by-block, because GPU thermal
    state, sim memory and OS state all drift over a 70-minute batch and would
    otherwise be confounded with the arm;
  * a completed cell is skipped on re-run unless --force, so an interrupted
    batch resumes instead of restarting.

Run:
    python experiments/semantic_ab.py --dry-run     # print the plan, fly nothing
    python experiments/semantic_ab.py
    python experiments/semantic_ab.py --only A1_on_p35,A2_on_m35
"""
from __future__ import annotations

import argparse
import json
import random
import socket
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PY = r"C:\Users\natha\.conda\envs\vla-real\python.exe"
UE = r"C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe"
UPROJ = str(ROOT / "PASBlocks" / "Blocks.uproject")
MAP = "/Game/JapaneseCity/Maps/Demo_day"

OUT = ROOT / "experiments" / "out"
RESULTS = OUT / "results.jsonl"
LOG = OUT / "run_log.md"


def log(msg: str) -> None:
    line = f"{datetime.now():%H:%M:%S} {msg}"
    print(line, flush=True)
    LOG.parent.mkdir(parents=True, exist_ok=True)
    with LOG.open("a", encoding="utf-8") as fh:
        fh.write(line + "\n")


def port_open(p: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", p)) == 0


def kill_sim() -> None:
    """Stop ONLY our sim, matched by its .uproject on the command line.

    `Get-Process -Name UnrealEditor | Stop-Process` — the pattern the older
    scripts in this repo use — kills every Unreal Editor on the machine, which
    includes unrelated projects the user may have open with unsaved work. Match
    on the project path instead.
    """
    ps = (
        "Get-CimInstance Win32_Process -Filter \"Name='UnrealEditor.exe' or "
        "Name='AirSimNH.exe'\" | Where-Object { $_.CommandLine -like "
        "'*Blocks.uproject*' -or $_.Name -eq 'AirSimNH.exe' } | "
        "ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"
    )
    subprocess.run(["powershell", "-NoProfile", "-Command", ps], capture_output=True)


def restart_sim() -> bool:
    kill_sim()
    time.sleep(4)
    # 640x360 and a 30 fps cap, not 1280x720. The VLA and the renderer share one
    # GPU, and at 720p inference measured ~10 s per action against ~3 s
    # standalone. Every second of that latency is a second the aircraft flies on
    # a stale command, so render budget spent here is taken straight out of
    # control quality. The sensor cameras keep their own configured resolution.
    subprocess.Popen([UE, UPROJ, MAP, "-game", "-windowed",
                      "-ResX=640", "-ResY=360", "-nosound",
                      "-FrameRateCap=30", "-NoVSync"])
    for _ in range(60):
        time.sleep(6)
        if port_open(8989):
            time.sleep(6)
            return True
    return False


def load_conditions(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def build_cmd(cell: dict, cfg: dict) -> list[str]:
    d = cfg["defaults"]
    instr = cfg["instructions"][cell["instr"]]
    adapter = cfg["adapters"][cell["adapter"]]
    cmd = [PY, str(ROOT / "demo" / "semantic_seek.py"),
           "--object", instr,
           "--adapter", adapter,
           "--policy", str(ROOT / d["policy"]),
           "--citymap", str(ROOT / d["citymap"]),
           "--target-xy", d["target_xy"],
           "--cruise-alt", str(d["cruise_alt"]),
           "--max-s", str(d["max_s"]),
           "--yaw-gain", str(d["yaw_gain"]),
           "--dv-h", str(d["dv_h"]),
           "--dv-z", str(d["dv_z"]),
           "--start-beta-deg", str(cell["beta"]),
           "--hint-mode", str(cell.get("hint_mode", d.get("hint_mode", "none"))),
           "--tag", cell["tag"]]
    if d.get("hold_s") is not None:
        cmd += ["--hold-s", str(d["hold_s"])]
    if d.get("fade_s") is not None:
        cmd += ["--fade-s", str(d["fade_s"])]
    if cell.get("follow_car", d.get("follow_car")):
        cmd += ["--follow-car", "--car-speed",
                str(cell.get("car_speed", d.get("car_speed", 3.0)))]
    if cell["scene"] == "absent":
        cmd.append("--no-spawn-target")
    if str(cell["guard"]).lower() in ("off", "false", "0"):
        cmd.append("--no-shield")
    return cmd


def run_flight(cell: dict, cfg: dict, timeout_s: int = 600) -> dict | None:
    cmd = build_cmd(cell, cfg)
    t0 = time.time()
    try:
        proc = subprocess.run(cmd, cwd=str(ROOT), capture_output=True,
                              text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        log(f"  {cell['tag']}: TIMEOUT after {timeout_s}s")
        return None
    mf = ROOT / "demo" / "out" / cell["tag"] / "metrics.json"
    if not mf.exists():
        tail = (proc.stderr or proc.stdout or "")[-600:]
        log(f"  {cell['tag']}: no metrics.json\n{tail}")
        return None
    m = json.loads(mf.read_text(encoding="utf-8"))
    m["cell"] = cell
    m["wall_s"] = round(time.time() - t0, 1)
    return m


def already_done(cell: dict) -> bool:
    return (ROOT / "demo" / "out" / cell["tag"] / "metrics.json").exists()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--conditions", default=str(ROOT / "experiments" / "conditions.yaml"))
    ap.add_argument("--dry-run", action="store_true",
                    help="print every command in order and exit — always do this "
                         "once before committing 70 minutes of sim time")
    ap.add_argument("--force", action="store_true", help="re-fly completed cells")
    ap.add_argument("--only", default=None, help="comma-separated tags")
    ap.add_argument("--timeout", type=int, default=600)
    args = ap.parse_args()

    cfg = load_conditions(Path(args.conditions))
    cells = list(cfg["cells"])
    if args.only:
        want = {t.strip() for t in args.only.split(",")}
        cells = [c for c in cells if c["tag"] in want]

    # seeded interleave: the arm must not be confounded with position in the batch
    rng = random.Random(cfg["seed"])
    rng.shuffle(cells)

    OUT.mkdir(parents=True, exist_ok=True)
    order = [c["tag"] for c in cells]
    log(f"=== semantic A/B: {len(cells)} flights, seed {cfg['seed']} ===")
    log(f"order: {' '.join(order)}")

    if args.dry_run:
        for i, c in enumerate(cells, 1):
            print(f"\n[{i}/{len(cells)}] {c['tag']}  "
                  f"({c['adapter']}/{c['instr']}/{c['scene']}/guard={c['guard']}/"
                  f"beta={c['beta']})")
            print("   " + " ".join(build_cmd(c, cfg)))
        print(f"\n{len(cells)} flights, no sim was started.")
        return 0

    (OUT / "order.json").write_text(json.dumps(
        {"seed": cfg["seed"], "order": order}, indent=1), encoding="utf-8")

    done, failed = 0, []
    for i, cell in enumerate(cells, 1):
        if already_done(cell) and not args.force:
            log(f"[{i}/{len(cells)}] {cell['tag']}: already flown — skipping")
            continue
        log(f"[{i}/{len(cells)}] {cell['tag']}: restarting sim")
        if not restart_sim():
            log("sim did not come up; stopping (results so far are kept)")
            break
        m = run_flight(cell, cfg, args.timeout)
        if m is None:
            failed.append(cell["tag"])
            continue
        # append immediately — a later crash must not cost this flight
        with RESULTS.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(m) + "\n")
        done += 1
        log(f"  {cell['tag']}: ticks={m['ticks']} inf={m['n_inference']} "
            f"({m['inference_hz']} Hz) nfz_s={m['nfz_s']} "
            f"entered={m['nfz_entered']} interv={m['interventions']} "
            f"clamps={m['integrity_clamps']} [{m['wall_s']}s]")

    kill_sim()
    log(f"=== done: {done} flown, {len(failed)} failed {failed} ===")
    log(f"results -> {RESULTS}")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
