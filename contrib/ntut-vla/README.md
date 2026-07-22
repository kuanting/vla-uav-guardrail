# NTUT Contribution — Fine-Tuned VLA + Guardrail Extensions

This folder contributes the NTUT AIoT Lab's work on top of the grant's Guardrail
framework: a **real, fine-tuned Vision-Language-Action pilot** flying inside the
safety layer, plus **global + reactive obstacle avoidance**, a **Project AirSim
(UE5) rail**, and a **point-and-click Mission Control GUI**.

It is additive and namespaced under `contrib/ntut-vla/` — it does not modify the
trunk packages. Where it overlaps the trunk (the guardrail core), it is our
independent implementation that converged on the same contracts (frozen 4-D
action, body→NED single boundary, policy_hash audit, trend-aware shield).

> **Large assets are intentionally excluded** (3D city maps, UE5 project,
> datasets, model weights, flight-output images). They live outside git; paths
> are referenced below.

---

## Map to the grant work packages

| WP | Grant item | Here |
|---|---|---|
| WP1 | Policy DSL | `policies/*.yaml` (fences, altitude band, kinematic caps) |
| WP2 | Prefix Compiler | `guardrail/compiler.py` (natural-language command → mission + YAML prompt) |
| WP3 | Safety Shield | `guardrail/shield.py` (+ `geometry.py`, `audit.py`, `models.py`): 3 s lookahead, trend-aware, repair ops, P0 re-check, brake |
| WP4 | Stress harness / evaluation | `training/` (auto-research loop, closed-loop eval, fine-tuning), `demo/` (5 sim rails) |
| — | Dynamic NFZ hot-apply (REST) | `guardrail/api.py` (POST /nfz) |

## What's new here (beyond the baseline)

1. **Real VLA in the slot** — `demo/real_vla_demo.py` (OpenVLA-7B),
   `demo/aerialvla_demo.py` / `demo/aerialvla_pas_demo.py` (AerialVLA UAV LoRA).
   4-bit, ~5.6 GB VRAM, 0.9–1.3 s/action; a background-thread inference so the
   10 Hz control loop never blocks.
2. **Weight fine-tuning campaign** — `training/finetune_aerialvla.py`,
   `collect_aerialvla_data.py`, `eval_aerialvla.py`, `tune_aerialvla.py`.
   QLoRA on self-collected flight data. Deploy config: path efficiency
   0.942 → **0.996**, shield workload **−60 %**. See `training/research_log.md`
   and `docs/aerialvla-ft-report.md`.
3. **Global path planner** — `demo/city_planner.py` (8-connected A* +
   supercover line-of-sight simplification) over a surveyed building occupancy
   grid (`demo/survey_city.py`). Routes waypoints around buildings AND drawn
   no-fly-zones. Reactive depth-camera avoidance is the safety net.
4. **Project AirSim (UE5) rail** — `demo/aerialvla_pas_demo.py` + `demo/pas_config/`.
5. **Mission Control GUI** — `demo/gui_mission_control.py`: click waypoints,
   draw NFZs (compiled to live policy), live drone tracking, planned-path preview.
6. **A/B comparison** — `--no-shield` / `--no-planner` flags + `demo_compare_guardrail.ps1`
   to show the guardrail's value directly (guardrail OFF flies through the NFZ).

## Headline results

- **P0 violation escape rate = 0** across 40+ eval flights, 5 simulators.
- Fine-tuned pilot: 0.996 path efficiency, NFZ 0.0 s, 100 % target reach (deploy cfg).
- Best urban flight (JapaneseCity): 0 shield interventions, NFZ 0.0 s.

## Run it (Windows, conda env `vla-real`)

See `TUTORIAL-ONECLICK.md`, `TUTORIAL-REAL-VLA.md`, `TUTORIAL-COMPARISON.md`.
Quick start:

```powershell
# fine-tuned VLA + guardrail + planner on a UE5 city:
.\demo_japanesecity.ps1 -Route "45,45; -40,50; 50,-55"

# guardrail OFF vs ON, same route through an NFZ:
.\demo_compare_guardrail.ps1

# point-and-click missions:
python demo\gui_mission_control.py
```

## External assets (NOT in git — reference by path)

| Asset | Location | Note |
|---|---|---|
| OpenVLA-7B base | `D:\models\openvla-7b` | 15 GB |
| AerialVLA LoRA + our fine-tune | `D:\models\aerialvla-ft\run2\epoch1` | production model |
| City occupancy maps | `demo/out/citymap/occ_<map>.npz` | per-world; rebuild with `survey_city.py` |
| UE5 city project | `PASBlocks/` | ~26 GB |
| Training datasets | `dataset/` | ~1 GB |

## Deck

`docs/VLA-Guardrail-FineTuned-Jul2026.pptx` (14 slides) — the current progress
deck, with `docs/PRESENTATION-CHEATSHEET.md` (talking points + Q&A) alongside.
