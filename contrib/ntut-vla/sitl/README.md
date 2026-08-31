# Fase 4 — ArduPilot SITL rail

Same Guardrail, real autopilot. Two rails now exist (the grant's split):

| Rail | Sim | Purpose |
|---|---|---|
| Functional (this folder) | **ArduPilot SITL** in WSL | real flight code, MAVLink path, regression/KPI runs |
| Perception (`demo/`) | **AirSim** on Windows | camera frames for the VLA, photoreal later |

## One-time setup (WSL Ubuntu 24.04)

```bash
# prereqs installed via apt (done); then:
bash setup_sitl.sh        # clones ardupilot, venv ~/venv-ap, builds SITL (~15 min)
```

## Run

Terminal 1 (WSL) — the autopilot:
```bash
bash start_sitl.sh                    # listens on tcp:127.0.0.1:5760
```

Terminal 2 (WSL) — the Guardrail mission:
```bash
cd "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone"
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield off     # A
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield on      # B
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield on --dynamic
```

No sim restart needed between runs (SITL relaunch is cheap; kill + rerun
start_sitl.sh if the vehicle state gets weird).

Outputs land in `demo/out/sitl_*/` — same artifacts as the AirSim demo
(trajectory plot, report, audit JSONL, prompt). No camera frames: SITL has no
renderer; perception stays on the AirSim rail.

## What this proves (talking point)

The Guardrail package did not change one line between AirSim and ArduPilot —
only the bottom adapter did (AirSim velocity API -> MAVLink
`SET_POSITION_TARGET_LOCAL_NED` in GUIDED mode). Adapter-isolation is the
grant's architecture rule, demonstrated.

## Notes

- SITL boot -> EKF ready takes ~20-40 s; the runner retries arming until ACK.
- `LOCAL_POSITION_NED` is meters from the EKF origin (= home), z down; the
  adapter flips to up-positive at the boundary, same rule as everywhere else.
- MAVROS 2 / ROS 2 node packaging is the NEXT step; pymavlink direct is the
  minimal honest MAVLink path and keeps WSL free of the full ROS install for now.
