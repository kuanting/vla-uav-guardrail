> **SUPERSEDED.** See `TUTORIAL.md` in the project root, which is the single verified guide. This file is kept as history only and its commands, tags and numbers may be out of date.

# Tutorial — Flying the REAL VLA (OpenVLA-7B) Through the Guardrail

This demo puts a **genuine 7-billion-parameter Vision-Language-Action model**
(openvla/openvla-7b — the base model AeroVLA fine-tunes) in the guardrail slot.
It **sees the drone's camera** and **reads a natural-language instruction**, and
every action it emits passes through our Safety Shield before reaching the sim.

Proven result (2026-07-14): `273 ticks | shield interventions 0 | NFZ 0.0s -> PASS`

## One command

```powershell
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
.\demo_real_vla.ps1
```

Options:
```powershell
.\demo_real_vla.ps1 -Instruction "fly toward the buildings"
.\demo_real_vla.ps1 -World blocks        # nh | blocks | mountains
```
(Blocked script? `powershell -ExecutionPolicy Bypass -File demo_real_vla.ps1`)

## What happens, step by step

1. Script kills any running sim and starts a fresh AirSimNH window.
2. `demo\real_vla_demo.py` loads OpenVLA-7B **4-bit quantized** from
   `D:\models\openvla-7b` — takes **2–5 minutes**, uses ~4.7 GB VRAM. Be patient;
   the drone does nothing until you see `[vla] loaded in ...s`.
3. Drone takes off and climbs to cruise altitude.
4. Loop at 10 Hz for 60 s: every 12 ticks the VLA gets a fresh camera frame +
   the instruction and emits an action (~0.9 s inference); the Shield checks
   **every** tick; the filtered action goes to the sim.
5. Report prints shield interventions + seconds inside the no-fly zone.

## Run it by hand (what the script does)

```powershell
# 1. start the world, wait until the drone appears:
D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe

# 2. new terminal — NOTE the env is vla-real, NOT airsim:
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
C:\Users\natha\.conda\envs\vla-real\python.exe demo\real_vla_demo.py --instruction "fly forward and avoid restricted areas"
```

## Reading the output

```
[vla] loaded in 289s (VRAM 4.7 GB)          <- 7B model is on your GPU
  [vla] raw=[0.004 -0.007 0.006] -> act=(0.24,-0.39,0.19)   <- model's decision
  tick 20: pos=(0.2,-0.5) raw=(-0.5,-0.3) shield=-           <- shield check each tick
[report] ... shield interventions 0 | time inside NFZ 0.0s -> PASS
```
`shield=-` means the action was legal; `shield=HIT` means the guardrail repaired it.

## Honest framing (say this in the demo)

Base OpenVLA is trained on **robot-arm manipulation**, not UAVs — so its raw
actions are tiny and aimless; the drone drifts rather than navigates. That is
expected, and it is exactly the point:

> "The slot accepts a real 7B camera+language VLA with a ~40-line adapter and
> zero guardrail changes. And even with a model that was never trained to fly,
> the guardrail holds the KPI: zero no-fly-zone seconds. Making it fly *well*
> is just a weights swap — AeroVLA's UAV LoRA on the same base."

## Difference vs the other demos

| | `demo_latest.ps1` (stages 1–3) | `demo_real_vla.ps1` |
|---|---|---|
| Pilot | stub / our trained policy (state-based) | **real OpenVLA-7B** (camera + language) |
| Sees images? | no (reads state + geometry) | **yes — drone camera every inference** |
| Understands language? | no (mission from compiler) | **yes — free-text instruction** |
| Conda env | `airsim` | **`vla-real`** |
| Load time | instant | 2–5 min (7B weights) |
| Flight quality | goal-directed | drifting (arm-trained base — expected) |
| What it proves | shield saves reckless pilot; trained model needs ~no saves | **slot works with a real VLA; guardrail holds KPI regardless** |

## Troubleshooting

- **`TimeoutError: Request timed out`** — sim not running or busy. Restart the
  AirSim .exe, wait for the drone to appear, run again.
- **Stuck at `Loading checkpoint shards`** — normal, up to 5 min (15 GB from disk).
- **CUDA out of memory** — close other GPU apps (games, another sim instance).
- **`ModuleNotFoundError`** — wrong env; use `C:\Users\natha\.conda\envs\vla-real\python.exe`.
- **numpy errors after any `pip install` in vla-real** — something dragged numpy
  to 2.x; re-pin: `pip install --force-reinstall numpy==1.26.4`.

## Outputs

`demo\out\real_vla\audit.jsonl` — every tick's shield decision, hash-stamped.
