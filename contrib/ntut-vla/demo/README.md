# VLA Drone Guardrail — Demo Guide

Working prototype of the meeting architecture:

```
User Command -> Constraint Compiler -> YAML Prompt -> VLA (stub)
             -> Safety Shield -> velocity cmd (10 Hz) -> AirSim drone
```

The core claim it demonstrates: **an untrusted, rule-ignorant VLA can be made
safe by a Guardrail layer — without touching the flight stack.**

## Run it (5 minutes)

1. Start the sim: `D:\AirSim\Blocks\WindowsNoEditor\Blocks.exe` (wait for the drone
   to appear; no dialog should show — settings force Multirotor mode).
2. In a terminal:
   ```
   conda activate airsim
   cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
   python demo/run_demo.py --shield off     # A: watch the NFZ get violated
   ```
3. **Restart the sim exe** (fresh spawn at origin), then:
   ```
   python demo/run_demo.py --shield on      # B: same VLA, now guarded
   ```
4. Restart again, then the showstopper — a no-fly zone appears MID-FLIGHT:
   ```
   python demo/run_demo.py --shield on --dynamic
   ```
   At t=8 s a second NFZ (purple) is hot-applied on the drone's detour path;
   the policy generation bumps, the audit hash changes, and the drone reroutes
   around BOTH zones. This is the grant's `dynamic_nfz` hot-apply in miniature.
5. Show `demo/out/*/trajectory.png` side by side. `frames/` in each run holds
   the drone-camera images (what a real VLA would see).

> Restart the sim between runs — the drone stays where it landed otherwise.
> Do NOT call `client.reset()` (this AirSim build goes command-deaf after it).

## Talking points (for the presentation)

1. **Scenario:** operator says *"fly to the northeast pad at 6 m/s"*. Straight
   line to the pad crosses a no-fly zone; 6 m/s exceeds the 4 m/s policy cap.
2. **Compiler (before-VLA):** ambiguous text -> structured mission + YAML prompt
   carrying the constraint summary (`demo/out/*/prompt.yaml`). Grant term: CSP.
3. **VLA slot:** today a stub P-controller, deliberately reckless. The interface
   (state -> 4-D action at 10 Hz) is the grant's locked contract, so a real
   VLA (CognitiveDrone, OpenVLA, ...) drops in without changing the Guardrail.
4. **Shield (after-VLA):** predicts 3 s ahead, catches violations, repairs
   (speed clamp / altitude fix / geofence slide), brakes only as last resort.
   Emitted action is re-checked: **P0 escape rate 0 by construction**.
5. **A/B evidence:** OFF = drone cuts through the zone (KPI FAIL, 3.5 s inside).
   ON = same VLA, drone slides around the edge (KPI PASS, 0 s inside), still
   reaches the target. Audit log (`audit.jsonl`) carries `policy_hash` on every
   record — the reproducibility anchor from the grant.

## What is real vs. stubbed

| Piece | Status |
|---|---|
| Policy DSL (YAML -> Pydantic, hash) | real, mini (3 of 9 constraint classes) |
| Constraint Compiler | real pipeline, toy NL parsing (regex + place registry) |
| VLA | stub (P-controller); swappable by design |
| Safety Shield (check/repair/brake) | real, 15 unit tests (`tests/test_shield.py`) |
| Audit log with policy_hash | real |
| KPI measurement (NFZ seconds) | real, computed from flown trajectory |
| Dynamic NFZ hot-apply (generation bump + hash change) | real (`--dynamic`) |
| Camera capture (VLA's-eye view) | real (`frames/`, every ~3 s) |
| MAVLink/ArduPilot | not yet — AirSim velocity API stands in (Fase 4) |

## Known limits (say them before someone asks)

- Shield only enforces *policy* constraints; physical obstacle avoidance is the
  other team's layer (drone once parked itself against a Blocks cube at 4 m alt —
  that is why the demo flies at 15 m).
- Straight-line 3 s forecast, constant velocity — no wind/dynamics model.
- Geofence slide can jitter along the edge (visible as orange dots); mission
  still completes. Grant's full PathRepair operator would smooth this.
