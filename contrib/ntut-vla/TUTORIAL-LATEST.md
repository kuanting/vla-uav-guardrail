> **SUPERSEDED.** See `TUTORIAL.md` in the project root, which is the single verified guide. This file is kept as history only and its commands, tags and numbers may be out of date.

# Tutorial — Running the LATEST Demo

The newest demo shows the **trained flight policy** (`--vla v3`) — the model the
auto-research loop produced — flying itself, with the guardrail barely needing
to step in. It replaces the old story where a dumb stub pilot needed the shield
to save it ~200 times per flight.

## One command (recommended)

```powershell
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
.\demo_latest.ps1
```
It runs three stages, restarts the AirSim sim between each, and pops each
trajectory plot. Press ENTER to advance. Options:
```powershell
.\demo_latest.ps1 -Stage 3            # only the trained model
.\demo_latest.ps1 -World mountains    # nh | blocks | mountains
```
(If PowerShell blocks scripts: `powershell -ExecutionPolicy Bypass -File demo_latest.ps1`, or double-click nothing — run from a terminal.)

## The three stages (the whole story)

| Stage | Command inside | What you see |
|---|---|---|
| 1 — villain | `--shield off --vla stub` | pilot cuts straight through the no-fly zone → **KPI FAIL** |
| 2 — old way | `--shield on --vla stub --dynamic` | reckless stub, but guardrail catches every violation (~200 saves) → **PASS** |
| 3 — **new** | `--shield on --vla v3 --dynamic` | **trained model flies itself**, ~near-zero saves → **PASS** |

Watch the shield-save count in each `[report]` line drop from ~200 (stage 2) to
a handful (stage 3). That drop **is** the result.

## Run one stage by hand (what the script does)

```powershell
# 1. start the world, wait for the drone to appear:
D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe

# 2. new terminal:
conda activate airsim
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
python demo\run_demo.py --shield on --vla v3 --dynamic `
  --policy policies\urban_demo_policy.yaml --command "fly to (40, 40) at 6 m/s altitude 20"
```
Restart the .exe before each run (the drone stays where it landed).

## What is DIFFERENT vs the earlier demo

| | Earlier demo | Latest demo |
|---|---|---|
| Pilot in the slot | hand-written stub only | **trained policy** (`--vla v3`, `models/vla_policy_v2.pt`) |
| Shield interventions | ~200 per flight (shield does all the work) | **near-zero** (the model already obeys the rules) |
| Zone geometry | single static zone | static + **mid-flight dynamic zone** (`--dynamic`) |
| Altitude behaviour | shield's AltitudeFix fired constantly | **altitude-aware decoder** — model stays in the band on its own |
| Smoothness | visible wobble at zone edge | **rate limiter** (`--dv-h/--dv-z`) → worst swing 4.8 → 0.5 m/s |
| Landing | abrupt drop (looked like falling) | **gentle controlled descent** |
| Worlds | Blocks only | Blocks / **AirSimNH (urban)** / mountains |
| Live API | none | optional `--api` (POST /nfz mid-flight) |

## New flags on `run_demo.py`

- `--vla stub|bc|v3` — who flies: dumb stub / early model / **gate-2 trained model**
- `--dynamic` — inject a second no-fly zone at t = 8 s
- `--api` — serve the hot-apply REST API during the flight (port 8071)
- `--dv-h 0.25 --dv-z 0.15` — smoothing strength (smaller = smoother)

## Outputs (per run, under `demo\out\<tag>\`)
`trajectory.png` (map) · `report.md` (KPI table) · `audit.jsonl` (every shield
save, hash-stamped) · `prompt.yaml` (compiled mission) · `frames\` (drone camera).

## Talking point for the demo
> "Same guardrail as before. What changed is the pilot: instead of a reckless
> stub the guardrail must constantly correct, we dropped in a policy we trained
> that already flies inside the rules. The shield-save count falls from ~200 to
> a handful — proof that pushing constraints upstream reduces the safety layer's
> workload. And the guardrail is still there, still enforcing zero violations."
