# Architecture v2 — Planning Document
**Constraint-Aware VLA Guardrail for ArduPilot UAVs**
NTUT AIoT Lab · Guardrail work package · 2026-07-03
Diagram: [`architecture-v2.svg`](architecture-v2.svg) (color-coded: teal = new/ours, blue = Prof. Lai team, gray = existing untouched, dashed gold = interface to define)

---

## 1. Scope recap (from 2026-06-23 kickoff)

Our team builds the **Guardrail framework** that keeps an AI pilot (LLM/VLM/VLA)
inside safety and regulatory limits. Prof. Lai's team owns the application layer:
flight, navigation, physical obstacle avoidance, GCS, computer vision, object
tracking, dynamic no-fly-zone sourcing.

**Locked principle: the core flight stack is NOT modified.** ROS, MAVLink,
built-in GeoFence, ArduPilot, PX4 stay as-is. We only add layers in front of and
behind the VLA:

```
User Command → Constraint Compiler → YAML Prompt → VLA
            → Hard Validation (Safety Shield) → MAVLink → ArduPilot/PX4 → Drone
```

## 2. What changed since v1

v1 (kickoff) was a concept sketch. v2 reflects a **working prototype** — every
teal box in the diagram exists as running code with tests:

| Component | v1 status | v2 status |
|---|---|---|
| Policy DSL (YAML rules → validated bundle, hash + generation) | concept | **prototyped** (3 constraint classes: polygon fence, altitude envelope, kinematic envelope) |
| Constraint Compiler (text → structured mission + YAML prompt) | concept | **prototyped** (regex/place-registry parsing; LLM/map parsing later — interface stable) |
| Safety Shield (predict → repair → re-check → brake) | concept | **prototyped**, 16/16 unit tests; repair ops: speed clamp, altitude fix, geofence slide, geofence escape, brake |
| Dynamic NFZ hot-apply (mid-flight rule injection, generation bump) | not in v1 | **prototyped** |
| Verification harness (A/B runs, KPI, audit log, plots) | "biggest blocker" | **prototyped v0** in AirSim |
| VLA backend | — | stub (P-controller); slot contract locked: `state [+ camera] → (vx, vy, vz, yaw_rate) @ 10 Hz` |
| MAVROS 2 / ArduPilot SITL path | — | not yet (next phase); AirSim velocity API stands in |

## 3. Prototype evidence (AirSim Blocks, 2026-07-03)

Same deliberately-reckless VLA stub in all three runs (asks 6 m/s where policy
caps 4; straight line crosses a no-fly zone):

| Run | Result | P0 KPI (zero NFZ entry) |
|---|---|---|
| Shield **OFF** | cuts through NFZ, 3.5 s inside | **FAIL** |
| Shield **ON** | slides along edge, mission completes, 201 repairs / 0 brakes | **PASS** (0.0 s) |
| Shield ON + **dynamic NFZ** at t=8 s | policy generation 0→1, hash changes, reroutes around both zones | **PASS** (0.0 s) |

Artifacts per run (in `demo/out/<tag>/`): trajectory plot, KPI report, JSONL
audit log (every record carries `policy_hash`), drone-camera frames, compiled
YAML prompt. Reproduce: `demo/README.md`.

Design findings worth sharing (found by testing, not on paper):
1. **Freezing when already in violation = deadlock.** If the vehicle is already
   below the altitude floor / inside a zone, "brake" locks the violation in
   place. The Shield therefore uses trend-aware checks: *violating but actively
   correcting* passes, and dedicated recovery operators (climb-back, zone-escape)
   produce the correcting action.
2. **Head-on repair needs an anti-stall bias.** Cancelling the into-zone velocity
   component of a head-on approach leaves zero velocity → permanent stall. The
   slide operator adds a tangent component so the mission keeps moving along the
   zone edge.
3. **Guardrail ≠ obstacle avoidance.** The shield enforces *policy* geometry
   only; the drone once parked itself against a physical block that no rule
   covered. Physical avoidance stays with Prof. Lai's navigation stack — the
   split from the kickoff is correct and now demonstrated.

## 4. Interface proposal — CV & Dynamic NFZ (action item, due 5 Jul)

What we need from Prof. Lai's systems, as **events**, so the Guardrail can act on them:

```yaml
# dynamic NFZ event (GCS/NFZ service -> Guardrail hot-apply endpoint)
event: nfz_update
id: nfz-landslide-007
action: add            # add | remove | move
vertices: [{x: 23, y: 6}, {x: 31, y: 6}, {x: 31, y: 14}, {x: 23, y: 14}]
altitude_band_m: [0, 100]
valid_until: null      # or ISO timestamp
source: gcs-operator   # gcs-operator | cv-auto | authority
```

```yaml
# CV detection event (vision -> Guardrail; may auto-derive an NFZ)
event: detection
class: landslide       # landslide | victim | target-object | ...
confidence: 0.87
position: {x: 27, y: 10}
radius_m: 15
```

Proposed parameters (to confirm together):
- **Format:** YAML or JSON, same schema (we validate with Pydantic either way).
- **Update rate:** NFZ events ≤1 Hz (they are rare); detections up to ~5 Hz, we
  debounce before deriving policy changes.
- **Transport:** GCS → REST endpoint (prototype already has the hot-apply hook)
  or a ROS 2 topic once we're on MAVROS 2 — whichever fits your stack better.
- **Guarantee on our side:** every applied event bumps the policy generation and
  changes `policy_hash`, so the audit log shows exactly which rules were active
  at any instant (replayable).

**Questions for your team:** who authors NFZ events (operator on GCS? automatic
from CV?); does CV output come as pixel boxes or world coordinates; which
coordinate frame do you use (we currently use local meters, WGS84 planned).

## 5. Next steps

| When | What |
|---|---|
| now → mid-Jul | Interface agreement (this doc §4) + technical consultation w/ Prof. Dai & Prof. Lai (flight-stack "modifiable points", new components around VLA) |
| Jul–Aug | Fase 4: ArduPilot SITL + MAVROS 2 replaces the AirSim velocity adapter; Shield becomes a ROS 2 node (same code, thin wrapper) |
| Aug–Sep | Real VLA backend into the swappable slot; scenario library growth (time windows, corridor constraints) |
| H2 | Photoreal virtual-terrain demo (mountain scenario), per kickoff plan |

## 6. Open questions (for the consultation)

1. Which parts of the flight stack may we touch, if any (parameters only? GeoFence config?)
2. PX4 compatibility priority — when does it become real work vs. design constraint?
3. VLA choice/hosting — who runs the model, on what hardware (Orin? desktop GPU)?
4. AirSim variant for the shared sim environment: original 1.8.1 (archived), Colosseum (UE5 fork), or Project AirSim (commercial)? Prototype currently runs original 1.8.1 binaries.
