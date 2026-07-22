# Repo Inventory & Cleanup (2026-07-22)

Categorizes every top-level item: **CURRENT** (keep, active), **SUPERSEDED**
(older version, kept for history — not deleted per the v1 lock rule),
**LARGE / EXCLUDED** (never committed to git — too big), **JUNK** (safe to
remove / archived).

---

## CURRENT — active, part of v1

| Path | What |
|---|---|
| `guardrail/` | Core guardrail package (shield, compiler=WP2, geometry, audit, api=REST, models, vla_backends) |
| `demo/aerialvla_demo.py` | AerialVLA in classic AirSim |
| `demo/aerialvla_pas_demo.py` | AerialVLA on Project AirSim (planner + avoidance + `--no-shield`) |
| `demo/real_vla_demo.py` | Base OpenVLA-7B in the slot |
| `demo/gui_mission_control.py` | Mission Control GUI |
| `demo/city_planner.py`, `demo/survey_city.py` | Global A* planner + occupancy survey |
| `demo/run_demo.py` | Multi-rail demo driver (`--vla`, `--shield`, `--dynamic`) |
| `training/` | Fine-tuning + auto-research pipeline, logs, eval jsons, report |
| `policies/` | Policy YAMLs (urban_demo, gui_high_test, …) |
| `models/` | Trained BC policy (`vla_policy_v2.pt/.onnx`) + manifests |
| `docs/` | Reports, architecture, checkpoint, cheatsheet, **v1 deck** `VLA-Guardrail-FineTuned-Jul2026.pptx` |
| `run_japanesecity.bat`, `demo_japanesecity.ps1`, `demo_real_vla.ps1`, `demo_compare_guardrail.ps1` | One-click runners |
| `TUTORIAL-ONECLICK.md`, `TUTORIAL-REAL-VLA.md`, `TUTORIAL-COMPARISON.md` | Current tutorials |
| `*.pdf` (7 grant PDFs) | Prof's grant design docs (reference) |
| `meeting notes/` | Meeting notes (one English .md each) |

## SUPERSEDED — older version, kept (do not delete)

| Path | Replaced by |
|---|---|
| `demo_latest.ps1`, `TUTORIAL-LATEST.md`, `DEMO-SCRIPT.md` | `demo_japanesecity.ps1` + `TUTORIAL-ONECLICK.md` |
| `fly.ps1`, `fly.bat` | `demo_japanesecity.ps1` |
| `demo/live_demo.ps1`, `demo/_run3_auto.ps1` | `demo_japanesecity.ps1` / `demo_compare_guardrail.ps1` |
| `docs/VLA-Guardrail-Comprehensive-Jul2026.pptx` | `VLA-Guardrail-FineTuned-Jul2026.pptx` |
| `RUNBOOK.md`, `LEARNING-ROADMAP.md`, `docs/initial-report-draft.md` | historical, kept |

## LARGE / EXCLUDED — never commit to git (too big)

| Path | Size | Why excluded |
|---|---|---|
| `PASBlocks/` | ~26 GB | UE5 city project (3D maps/models) |
| `dataset/` | ~1 GB | training images/samples |
| `demo/out/` | large | flight outputs (trajectory PNG, frames, audit, `citymap/*.npz`) |
| `source/` | ~152 MB | cloned reference repos (uav-vla, AeroVLA, …) |
| `kuanting-vla-uav-guardrail/` | ~265 MB | Prof's repo clone (separate git) |
| `models/*.pt/.onnx` | ~12 MB | trained weights — reference by path, don't commit |
| external: `D:\models\*`, `D:\AirSim\*`, `D:\ProjectAirSim\*` | 10s of GB | model weights + sim binaries |

## JUNK — safe to remove

| Path | Action |
|---|---|
| `OneDrive_1_6-9-2026.zip` (2.5 GB) | **delete manually** (stale backup zip) |
| `2` (0 KB stray file) | archived to `_archive/` |
| `projectairsim_client.log`, `demo/projectairsim_server.log` | archived / regenerated each run |

---

## What gets uploaded to the Prof repo (small, code + key docs only)

`guardrail/`, `training/*.py` + logs + report, `demo/*.py` + runner `.ps1`,
`policies/*.yaml`, `docs/*.md` + `VLA-Guardrail-FineTuned-Jul2026.pptx` +
`docs/img/*.png`, the tutorials. **NOT** uploaded: `PASBlocks/`, `dataset/`,
`demo/out/`, `source/`, model weights, the 17 MB `*-Progress-*.pptx`.
