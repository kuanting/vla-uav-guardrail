# Progress Checkpoint — v1 (Presentation Lock)

**Locked:** 2026-07-22 · **Status:** presentation-ready, work continues after this point.

> This is a **versioned checkpoint**. Nothing here is deleted going forward —
> new work is added on top and tagged v2+. The deliverables below are the "v1"
> baseline for the mid-term presentation.

---

## 1. What this project delivers

A **safety Guardrail** for Vision-Language-Action (VLA) drones, plus — beyond the
original scope — a **real fine-tuned VLA pilot** flying inside it, on real urban
maps, with **global + reactive obstacle avoidance**.

Pipeline (every stage auditable; the Shield is always the final authority):

```
User command / GUI  →  Constraint Compiler  →  Fine-tuned VLA (7B)  →
Global Planner (A*)  →  Depth Avoidance  →  Rate Limiter  →  Safety Shield  →  Sim
```

---

## 2. v1 locked deliverables (do not delete)

| Area | File(s) | State |
|---|---|---|
| Guardrail core | `guardrail/` (shield, compiler, geometry, audit, models) | frozen v1 |
| Fine-tuned model | `D:\models\aerialvla-ft\run2\epoch1` + `models/aerialvla_deploy_manifest.json` | **SUPERSEDED** — comparison arm only. It raised coordinate path efficiency 0.942→0.996 but lowered object-slot sensitivity 0.454→0.321, so every demo default now points at the original adapter. See `docs/FINDING-what-drives-aerialvla.md`. |
| VLA flight (Project AirSim) | `demo/aerialvla_pas_demo.py` | v1 |
| VLA flight (classic AirSim) | `demo/aerialvla_demo.py` | v1 |
| Global planner | `demo/city_planner.py` + `demo/survey_city.py` + `demo/out/citymap/occ.npz` | v1 |
| Mission Control GUI | `demo/gui_mission_control.py` | v1 |
| One-click runners | `run_japanesecity.bat`, `demo_japanesecity.ps1`, `demo_real_vla.ps1` | v1 |
| Deck | `docs/VLA-Guardrail-FineTuned-Jul2026.pptx` (14 slides) | v1 |
| Tutorials | `TUTORIAL-ONECLICK.md`, `TUTORIAL-REAL-VLA.md`, `TUTORIAL-LATEST.md` | v1 |
| Training campaign log | `training/research_log.md`, `docs/aerialvla-ft-report.md` | v1 |

**Versioning rule going forward:** keep v1 files intact; put new experiments in
new files or clearly-tagged sections. The deck filename carries its date; the
next deck should be `*-<newdate>.pptx`, not an overwrite.

---

## 3. Headline results (measured)

| Metric | Value |
|---|---|
| **P0 violation escape rate (KPI)** | **0** across 40+ eval flights, 5 simulators |
| Fine-tuned path efficiency (deploy cfg) | **0.996** (vs 0.942 baseline) |
| Shield workload cut by fine-tuning | **−60 %** (108 → 43.6 interventions) |
| Model | openvla-7b + AerialVLA LoRA + our QLoRA, 4-bit, **5.6 GB VRAM** |
| Inference latency | **0.9–1.3 s / action** (control loop stays 10 Hz) |
| Urban best flight | JapaneseCity: REACHED, **0 interventions**, NFZ 0.0 s |
| City tour (5 waypoints) | 6/7 sub-waypoints via global planner, NFZ 0.0 s |

---

## 4. Known limits (stated honestly — these are the v2 backlog)

- **Pure-VLA navigation ~20 %** — 0.8 Hz inference = ~63° heading per decision.
  Deployed system uses 55 % mission-direction assist (global-planner + local
  policy split). Fix path: faster inference (TensorRT-LLM) or waypoint chunking.
- **Occupancy map is nadir-sampled + striped** — reliable but has holes; a target
  behind an unmapped or >55 m building can still trap the drone (it climbs to the
  ceiling, then safe-skips). v2: denser/full-footprint survey once the down-depth
  semantics are pinned down.
- **Jetson numbers are projections**, hardware validation pending.
- **NFZ-in-path routing** — fixed on 2026-07-22 (planner now stamps NFZs), see
  the progress log; verify in v2 sign-off.

---

## 5. How to run (v1)

```powershell
# one-click urban demo (fine-tuned VLA + guardrail + planner):
.\run_japanesecity.bat                       # or: .\demo_japanesecity.ps1 -Route "45,45; -40,50; 50,-55"

# point-and-click Mission Control GUI:
conda activate vla-real
python demo\gui_mission_control.py           # click waypoints, right-drag NFZs

# rebuild the city occupancy map (once per new map):
python demo\survey_city.py --alt 52 --half 70 --line-step 3
```

Environment: conda env **`vla-real`** (`C:\Users\natha\.conda\envs\vla-real\python.exe`).
