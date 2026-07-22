# Initial Report (Draft) — Guardrail Framework for Constraint-Aware VLA UAV Control
NTUT AIoT Lab · for mid-year review · draft v1, 2026-07-03

> Mid-year deliverable per 2026-06-23 agreement: architecture diagram + initial
> report + Guardrail concept. Full demo not required — but a working prototype
> already exists and is summarized in §4.

## 1. Problem

Vision-Language-Action models can pilot drones from camera images and natural-
language commands, but they are probabilistic: they sometimes emit commands that
violate airspace rules (no-fly zones, altitude limits, speed caps). Flight
safety cannot rest on a model that is sometimes wrong.

## 2. Approach — Guardrail, not a new autopilot

We wrap the VLA in two protective layers and change nothing underneath:

```
User Command → Constraint Compiler → YAML Prompt → VLA
            → Safety Shield (hard validation) → MAVLink → ArduPilot/PX4 → Drone
```

- **Before the VLA — Constraint Compiler.** Ambiguous human text ("fly around
  the mountain") becomes a structured YAML mission carrying an explicit summary
  of active constraints. The VLA receives clear, bounded instructions, so it
  violates less often.
- **After the VLA — Safety Shield.** Every 4-D action (vx, vy, vz, yaw_rate;
  10 Hz) is checked against a 3-second trajectory forecast. Violations are
  repaired minimally (speed clamp, altitude fix, slide along zone edge), or the
  vehicle recovers/brakes. The emitted action is re-checked: an action that
  still violates never leaves the Shield — **P0 escape rate = 0 by construction**.
- **Policy DSL.** All rules live in one versioned YAML policy with a content
  hash; mid-flight rule injection (dynamic NFZ) bumps a generation counter, so
  every logged action is traceable to the exact rule set active at that moment.

The flight stack (ROS, MAVLink, ArduPilot/PX4, built-in GeoFence) is untouched;
ArduPilot's own GeoFence remains the last-line backstop behind our Shield.

## 3. Division of work

| Our team | Prof. Lai's team |
|---|---|
| Guardrail framework (Compiler, Policy DSL, Shield) | Flight, navigation, physical obstacle avoidance |
| Validation / verification harness, KPI reporting | GCS, computer vision, tracking |
| VLA integration interfaces | Dynamic no-fly-zone sourcing |

## 4. Status — working prototype (simulation, AirSim)

All Guardrail components exist as running Python code (16/16 unit tests). A/B
evidence with an intentionally reckless VLA stand-in:

| Run | Time inside NFZ | P0 KPI |
|---|---|---|
| Shield OFF | 3.5 s | FAIL |
| Shield ON | 0.0 s (mission still completes) | **PASS** |
| Shield ON + NFZ injected mid-flight (t=8 s) | 0.0 s (reroutes around both zones) | **PASS** |

Each run produces a trajectory plot, KPI report, audit log (policy-hash on every
record), compiled YAML prompt, and drone-camera frames. The VLA slot is a stub
controller today; its interface (observation → 4-D action @ 10 Hz) matches
current drone-VLA practice, so a real model drops in without Guardrail changes.

## 5. Next phase

1. Interface agreement with Prof. Lai's team (CV detections, dynamic NFZ events — proposal drafted).
2. Replace the simulator adapter with ArduPilot SITL + MAVROS 2 (Shield becomes a ROS 2 node).
3. Real VLA backend; richer scenario library (time windows, corridors).
4. H2: photoreal virtual-terrain demonstration per kickoff plan.

## Appendix

- Architecture diagram: `architecture-v2.svg` · Planning detail: `architecture-v2.md`
- Reproduction guide: `demo/README.md` · Evidence: `demo/out/*/`
