# Midterm Report

## Semantic-Spatial Translation and Safety-Constrained VLA for ArduPilot UAVs

**Institution** National Taipei University of Technology (NTUT), AIoT Laboratory
**Principal investigator** Prof. Kuan-Ting Lai
**Author** Nathanael Tjahyadi
**Funding body** ITRI
**Project period** February – November 2026
**Reporting period** February – August 2026
**Date** 20 August 2026

---

## 1. Deliverables accompanying this report

| Artefact | File |
|---|---|
| This report | `docs/MIDTERM-REPORT-Aug2026.pdf` (and `.docx`) |
| Slide deck | `docs/VLA-Guardrail-Midterm-Aug2026.pptx` (and `.pdf`) |
| Demonstration video, tracking | `docs/video/demo_follow.mp4` |
| Demonstration video, distractors | `docs/video/demo_traffic.mp4` |
| Demonstration video, no-fly zone | `docs/video/demo_nfz.mp4` |

All numerical claims in this report are read from `demo/out/<tag>/metrics.json`
and `demo/out/<tag>/flight_log.jsonl`. Section 11 indexes the source files.

---

## 2. Objectives and scope

The project addresses a specific failure mode. Vision-Language-Action (VLA)
models can pilot a UAV from camera images and natural-language instructions, but
they are probabilistic and occasionally emit commands that violate airspace
rules. Flight safety cannot rest on a component that is sometimes wrong.

The response is not a better model. It is a deterministic layer, the **Guardrail**,
placed between any action source and the autopilot, holding one contractual
acceptance criterion:

> **P0 violation escape rate = 0.** No action that violates a P0-priority rule
> reaches the vehicle.

Scope for this reporting period:

1. Establish the Guardrail as a component independent of the model above it.
2. Demonstrate language-commanded target following in a photorealistic city.
3. Demonstrate the Guardrail intervening under a rule conflict, without losing
   the mission.
4. Build the verification apparatus needed to make KPI claims traceable.

Explicitly out of scope this period: hardware flight, LiDAR or depth-based
obstacle avoidance beyond the existing occupancy map, and multi-agent operation.

---

## 3. System architecture

### 3.1 The action contract

Every component above the Guardrail communicates through a single type:

```
Action4D(vx, vy, vz_up, yaw_rate)      at 10 Hz
    vx      m/s, positive North
    vy      m/s, positive East
    vz_up   m/s, positive up
    yaw_rate rad/s, positive clockwise from above
```

Defined at `guardrail/models.py:31`. The Guardrail accepts nothing else. Any
action source that emits `Action4D` is admissible, which makes the model above a
replaceable component rather than a dependency.

### 3.2 Pipeline

```
operator text ─┐
front camera ──┴─> OWL-ViT ─> colour gate ─> state estimator ─> guidance
                                                                    │
                            policy + occupancy map ─> SAFETY SHIELD <┘
                                                          │
                                                          v
                                        autopilot (simple_flight / ArduPilot)
                                                          │
                                                          v
                                                      aircraft
```

The Shield is the last component before the autopilot and has final authority.

### 3.3 Why the Guardrail is the deliverable

The action stage in the tracking demonstration is a hand-written proportional
controller, not a learned policy. This is a deliberate consequence of the
architecture, and it is stated plainly because the distinction matters to how
the results should be read.

Over 108 controlled forward passes (`docs/FINDING-what-drives-aerialvla.md`),
the `{object}` prompt slot of AerialVLA was measured to be **inert**: a correct
colour word and an incorrect one produce indistinguishable actions. Only the
`{direction}` compass phrase drives its output. Language grounding therefore had
to come from elsewhere, and OWL-ViT supplies it.

The Guardrail has been exercised with five different occupants of the action
slot (Section 6.4). The Shield source code is identical in all five cases. That
invariance, not any single model's performance, is the project's claim.

---

## 4. Methods

### 4.1 Perception

**Detector.** `google/owlvit-base-patch32`, 153 M parameters, 0.61 GB VRAM,
Apache-2.0. Open-vocabulary: the target is specified at runtime as a text string
(`--object "a yellow car"`) with no fixed class list and no retraining.

**Colour verification.** OWL-ViT localises the noun; a fixed rule verifies the
adjective. Each candidate box is cropped and scored on the fraction of pixels
matching the requested hue. The final ranking is

```
score x (0.25 + 0.75 x colour_match)
```

The gate originally applied an HSV **saturation** floor (`s > 90`). Saturation is
chroma divided by brightness, so a vehicle entering direct sunlight loses
saturation without changing colour. Measured on the target's own pixels: median
saturation fell from 99 in shade to 69 in sunlight, and the fraction of pixels
passing the gate fell from 56.7 % to 8.1 %. In one flight, 25 of 25 detections
were rejected by the colour gate and none by the detector score, while OWL-ViT
scored the vehicle 0.17–0.30, its highest of that flight.

The gate now applies an **absolute chroma** floor, `s x v / 255 > 40`, which is
invariant to illumination. Detector hit rate rose from 0.73 to 1.000.

### 4.2 Target-state estimation

Detections arrive at roughly 4 Hz while the control loop runs at 10 Hz, and the
box width used as a range proxy varies **37.7 %** between consecutive frames. A
proportional controller acting directly on that measurement produces rough
commands.

A constant-velocity Kalman filter (`demo/target_state.py`) converts the polar
measurement to Cartesian, gates innovations, and predicts between detections.
Velocity feed-forward replaces an integral term. Effect on the forward channel:

| Metric | From box width | From the estimate |
|---|---|---|
| Command step per tick, 95th percentile | 0.716 m/s | 0.283 m/s |
| Ticks where the rate limiter engaged | 15.6 % | 3.0 % |

A derivative term was not added: it would amplify the same measurement noise. An
integral term was not added: it would wind up whenever the Shield overrides the
commanded action. PID control is present in the system, below this layer, inside
the autopilot.

### 4.3 Guidance

Proportional control on three channels: horizontal box offset to yaw rate,
estimated range error to forward speed, altitude error to climb rate. Output
passes through a rate limiter before reaching the Shield.

### 4.4 Safety Shield

Three stages, at `guardrail/shield.py`:

1. **Monitor.** Forward-simulate the proposed action for 3 s and test it against
   every active constraint, with trend awareness so a violation is caught before
   it occurs rather than after.
2. **Repair.** Apply the minimal correction that clears the violation, by
   fixed-point iteration. The emitted action is re-checked; an action that still
   violates never leaves the Shield.
3. **Escalate.** If repair cannot clear the violation, brake.

The Shield returns a `ShieldDecision` recording the raw action, the emitted
action, every violation with its rule identifier and predicted time, every
repair operator applied, and whether it braked. This record is written per tick
to `audit.jsonl`.

**Separation of authority.** No repair operator modifies `yaw_rate`. Heading
remains the controller's; the ground track is what the Shield bends. This makes
simultaneity observable: a tick in which `emitted.yaw_rate == raw.yaw_rate` while
`emitted.(vx, vy) != raw.(vx, vy)` is one in which both systems acted.

**Constraint taxonomy.** Five types are expressible: `PolygonFence`,
`AltitudeEnvelope`, `KinematicEnvelope`, `ObstacleClearance` and
`SubjectStandoff`.
Each carries a priority (P0/P1/P2) and a violation action (repair or brake).
Policies are YAML, validated into a Pydantic model, and hashed to `policy_hash`.

**Defect corrected this period.** `yaw_rate_max_dps` was specified in degrees per
second and compared against a value carried in radians per second at three sites,
making the effective cap 2578 °/s. The rule could never fire. Conversion now
occurs at the boundary. The worst commanded yaw rate measured across the demo
flights is 11.8 °/s against a 45 °/s cap, so no recorded flight changes behaviour
as a result of the fix.

---

## 5. Experimental setup

| Parameter | Value |
|---|---|
| Simulator | Project AirSim on Unreal Engine 5.7 |
| Scene | JapaneseCity, `Demo_day` map |
| GPU | NVIDIA RTX 4080, 16 GB |
| Detector input (`FrontCamera`) | 400 x 225, 90° HFOV, pitched 20° down |
| Recording camera (`Chase`) | 960 x 540 at 20 Hz |
| Control loop | 10 Hz nominal |
| Cruise altitude | 9 m |
| Target vehicle | glTF taxi, 2.5 m/s, two 8 s stops |
| Route | 93 m, straight leg then a left turn onto the cross street |
| Flight duration | 70 s per scenario |
| Policies | `policies/follow_car.yaml`, `policies/follow_car_nfz.yaml` |

`FrontCamera` resolution is held fixed at 400 x 225 across all reported work.
It is the detector's input, and every measurement in the repository is
conditioned on it.

Three scenarios were flown:

- **`demo_follow`** — tracking with no fence. Isolates the perception and
  control question.
- **`demo_traffic`** — three additional vehicles of different colours on the
  same street. Tests target discrimination.
- **`demo_nfz`** — a polygon fence spanning the corridor at x ∈ [26, 54],
  y ∈ [2, 16], which the target vehicle drives through and the aircraft may not
  enter. Tests the Shield under an unavoidable conflict.

The fence in `demo_nfz` spans the whole corridor by construction. An earlier
version fenced only one leg; the aircraft tracked the target on the other leg,
never approached the boundary, and demonstrated nothing.

---

## 6. Results

All figures are read from `demo/out/<tag>/metrics.json` and `demo/out/<tag>/flight_log.jsonl`. This section is generated from those files by `tools/build_report_results.py` rather than transcribed, so it cannot disagree with the artefacts or with the slide deck.

### 6.1 Target following

| Measure | Tracking | Distractors | No-fly zone |
|---|---|---|---|
| Detector hit rate | 1.000 | 1.000 | 0.662 |
| Ticks with the target held | 100.0 % | 100.0 % | 92.5 % |
| Detector rate | 4.07 Hz | 4.56 Hz | 4.80 Hz |
| Control loop rate | 8.78 Hz | 8.07 Hz | 7.83 Hz |
| Mean separation | 17.4 m | 19.0 m | 40.5 m |
| Minimum separation | 11.1 m | 11.0 m | 12.4 m |
| Time within 30 m | 100.0 % | 92.2 % | 35.9 % |
| Flight duration | 69.9 s | 69.9 s | 70.0 s |

The two tracking scenarios held the target on every control tick. The no-fly-zone scenario holds a larger separation by design: the fence spans the corridor, the target drives through it, and the aircraft is required not to follow. It was held at the boundary for 405 ticks.

With three additional vehicles of different colours on the same street, target jumping fell from 14.0 % of detections to 0.4 %.

### 6.2 Guardrail invariants

| Invariant | Tracking | Distractors | No-fly zone | Requirement |
|---|---|---|---|---|
| P0 violation escape rate | 0.000 | 0.000 | 0.000 | 0 |
| Time inside the no-fly zone | 0.0 s | 0.0 s | 0.0 s | 0.0 s |
| Altitude envelope escape | 0.0 s | 0.0 s | 0.0 s | 0.0 s |
| Shield interventions | 54 | 49 | 45 | not bounded |

The acceptance criterion is met on every flight. An escape is counted only when the Shield neither repaired nor braked and the emitted action still violated a P0 rule; scoring the raw action would credit the system for its own inputs.

The intervention counts distinguish the two situations. In Tracking (54), Distractors (49), No-fly zone (45) the guidance layer proposed actions that would have violated an active rule, and the Shield corrected them; time inside the zone remained 0.0 s, which is the property being claimed. Repair is the normal outcome, not an error condition.

### 6.3 Recording resolution against detector throughput

The recording camera was raised to 1280 x 720 to improve video quality. Acceptance thresholds were fixed before the runs: detector rate at least 4.0 Hz and control loop at least 9.5 Hz.

| Chase capture | Simulator window | Detector rate | Control loop | Median inference |
|---|---|---|---|---|
| 1280 x 720 | 1280 x 720 | 2.94 Hz | 8.69 Hz | not recorded |
| 960 x 540 | 1280 x 720 | 3.66 Hz | 8.79 Hz | 286 ms |
| 960 x 540 | 960 x 540 | 3.62 Hz | 8.85 Hz | 287 ms |

OWL-ViT inference held at 286-287 ms median across a 2.4x change in Chase pixels and a 1.8x change in window pixels. Neither capture resolution nor the simulator window is the binding consumer; both hypotheses were tested and rejected. See _inference_breakdown for what the cost actually is - the earlier reading of this invariance as 'fixed per-inference cost' was wrong, and is corrected there.

The invariance above was first read as a fixed per-inference cost. That reading was wrong, and the correction matters because it changes which remedies are worth trying. Timing the CPU preprocessing and the GPU forward pass separately, in flight and on an idle GPU at the detector's real 400 x 225 input:

| | Idle | In flight | Inflation |
|---|---|---|---|
| Preprocessing (CPU) | 26.7 ms | 44.0 ms | 1.6x |
| Forward pass (GPU) | 36.2 ms | 242.6 ms | 6.7x |
| Total | 66.4 ms | 286.8 ms | 4.3x |

The cost is not fixed: it is 66 ms on an idle GPU. The simulator starves the GPU specifically, and the invariance to capture resolution meant only that the binding consumer is something else.

Three candidate remedies followed. Lowering Unreal's scalability settings had no effect at all, and half precision bought 1.19x, which does not justify the accuracy risk. The recording is the consumer:

| Recording | Detector rate | Forward pass |
|---|---|---|
| 20 Hz | 3.48 Hz | 242.6 ms |
| 10 Hz | 3.76 Hz | 221.6 ms |
| off | 4.10 Hz | 198.5 ms |

Detector hit rate and ticks-held were 1.000 in all three, so this costs nothing in tracking quality. The recorder pulls camera frames over RPC at the record rate, forcing the simulator to render and serialise extra captures — which is why the **forward pass** moves with it, and why capture resolution never did.

**The threshold is met on every scenario reported here** — Tracking 4.07 Hz, Distractors 4.56 Hz, No-fly zone 4.80 Hz, against a 4.0 Hz gate fixed before the runs, and with the recorder attached rather than removed for the measurement. det_hit_rate 1.000 and frac_ticks_seen 1.000 on both tracking scenarios in every configuration. The threshold exists to protect tracking quality, and tracking quality was never degraded.

That was not true earlier in the period. It took the recording rate coming down off the critical path, and the obstacle map being rebuilt over the band the aircraft occupies, before the detector had enough of the GPU to clear it.

### 6.4 Independence from the action source

| Action source | Nature | Recorded outcome |
|---|---|---|
| OpenVLA-7B, 4-bit | Real 7 B camera and language VLA | 553 ticks at 10 Hz, 10 Shield interventions, NFZ 0.0 s |
| AerialVLA LoRA | UAV-tuned adapter on the same base | Target reached, NFZ 0, clean path around the zone |
| QLoRA fine-tunes (ours) | Trained on self-collected expert flights | 100 % reached, mean efficiency 0.996, goal assist off |
| Behaviour-cloning policy | Trained state and geometry policy | Flown, NFZ 0 |
| Proportional controller | Hand-written, no model | Reported in 6.1 and 6.2 |

The Shield source code is identical in all five cases; only the adapter above it differs. That invariance, rather than any single model's performance, is the result this project claims.

An eight-flight study of 150 s each established that the Shield and the action source act within the same control tick rather than alternating: on the passing flights the model commanded a direction closing on the target at cos 0.53 to 0.95 while the Shield bent the resulting ground track by 82 to 103 degrees, with the heading channel untouched throughout. The guardrail-disabled control flights scored zero such ticks and spent 32.4 s and 84.2 s outside a P0 rule respectively.

### 6.5 The same Guardrail over ArduPilot

The results above run on Project AirSim. The same Guardrail package also flies over **ArduPilot SITL** — real flight code, real MAVLink. Not one line of `guardrail/` differs between the rails; only the adapter beneath them does, which is the architecture rule this project claims.

#### The grant's canonical topology

`vla_stub → /vla/action_4d → shield node → MAVROS 2 → ArduPilot SITL`. This is the configuration the grant names for contractual KPI figures, and these are **the first KPI-grade runs this project has produced**.

| Configuration | P0 escape rate | Time in zone | Interventions | Outcome | KPI-grade |
|---|---|---|---|---|---|
| Guardrail disabled | 0.627 | 3.7 s | 0 | fail | yes |
| Guardrail enabled | 0.000 | 0.0 s | 219 | success | yes |
| Guardrail enabled, dynamic zone | 0.000 | 0.0 s | 307 | success | yes |

The disabled run is grade-eligible and **fails**, which is the point of a control: its numbers may be quoted, and what they say is that without the Guardrail 62.7 % of ticks flew a P0 violation and the aircraft spent 3.7 s inside the zone. With the Guardrail, on the same rail and the same policy, both are zero.

It earns that escape rate honestly. The Shield evaluates on every tick and only enforcement is conditional, so the violations it observes are recorded against an action that flew unaltered. Were it to run only when enforcing, the control would log no violations at all and score as perfectly clean.

Determinism manifest, all six fields resolved:

| Field | Value |
|---|---|
| `code_revision` | `c4249edef599` |
| `vla_model_hash` | `guardrail.vla_stub.StubVLA@src:6be6a7c1f655ad93` |
| `policy_hash` | `sha256:77d64d2e5e94ac39` |
| `random_seed` | `0` |
| `sim_speedup` | `1.0` |
| `topology` | `canonical-hil` |

`canonical-hil` is not a label the caller may simply assert. `build_manifest()` requires evidence that every link of the chain was live — a ROS 2 distribution, a MAVROS node on the graph, and a flight controller reporting connected — because MAVROS starts happily with nothing on the other end and publishes `connected: false` indefinitely, so a node can fly an entire mission into the void and look healthy. Recorded for these runs: `ros_distro=jazzy`, `mavros_node=/mavros`, `fcu_connected=True`.

#### Direct MAVLink, for comparison

The same missions driven through pymavlink rather than MAVROS. Identical Guardrail, one adapter lower, and deliberately **not** KPI-grade: the topology is honest about lacking MAVROS 2, so `is_kpi_grade()` refuses it.

| Configuration | P0 escape rate | Time in zone | Interventions |
|---|---|---|---|
| Guardrail disabled | 0.627 | 3.7 s | 0 |
| Guardrail enabled | 0.000 | 0.0 s | 217 |
| Guardrail enabled, dynamic zone | 0.000 | 0.0 s | 218 |

That the two rails agree to three decimal places on the escape rate, through different middleware, is itself the adapter-isolation claim being tested rather than asserted.

### 6.6 Demonstration recordings

| Scenario | Frames | Capture rate | Duration |
|---|---|---|---|
| Tracking | 1724 | 15.57 Hz (target 20.0) | 110.7 s |
| Distractors | 1725 | 15.59 Hz (target 20.0) | 110.7 s |
| No-fly zone | 1716 | 15.74 Hz (target 20.0) | 109.0 s |

The capture rate is measured by the recorder and written to `view/recorder.json`. It is not derived from the flight log: the recorder starts before the mission clock and stops after it, so frames divided by mission duration overstates the rate and would produce a video that plays faster than real time while being labelled real time.

---

## 7. Verification apparatus

### 7.1 Acceptance KPIs

Four KPIs are computed from each flight's artefacts by `guardrail/kpi.py`. The
binding one is the P0 violation escape rate. An escape is counted only when the
Shield neither repaired nor braked and the **emitted** action still violated a
P0 rule — scoring against the raw action would credit the system for its own
inputs.

### 7.2 Determinism manifest

Each flight emits a six-field manifest (`guardrail/manifest.py`):
`code_revision`, `vla_model_hash`, `policy_hash`, `random_seed`, `sim_speedup`,
`topology`. `sim_speedup` is derived from the scene file rather than asserted by
the caller.

`is_kpi_grade()` decides whether a run's numbers may be quoted as contractual
figures. It fails a run when the simulation is not real-time, when required
fields did not resolve, when the detector ran below 2 Hz, when the initial
heading was more than 10° off, or when the topology is not the grant's canonical
ArduPilot SITL configuration. Each check exists because the corresponding failure
has already occurred and produced a plausible-looking number.

### 7.3 Test suite

**201 tests across nine modules, all passing**, run on 20 August 2026.

| Module | Tests | Covers |
|---|---|---|
| `test_range_and_lock.py` | 42 | Range estimation, target lock, instance selection |
| `test_city_traffic.py` | 34 | Route generation, traffic circuits, heading rates |
| `test_vla_bridge.py` | 28 | Compass-phrase mapping, absence of coordinate leakage |
| `test_manifest.py` | 24 | Determinism manifest, code-revision provenance, KPI-grade gating |
| `test_guardrail_coverage.py` | 16 | Shield behaviour over randomly sampled states |
| `test_shield.py` | 16 | Monitor, repair and escalation logic |
| `test_fenceguard.py` | 14 | Polygon fence geometry and margins |
| `test_target_state.py` | 14 | Kalman filter, innovation gating, feed-forward, subject width |
| `test_clearance.py` | 13 | Obstacle clearance against the occupancy map |

`test_vla_bridge.py` includes a structural test rather than an assertion of
intent: it tokenises the bridge module, discards comments and docstrings, and
fails if any target-coordinate identifier survives in the code, then walks the
AST and fails if any function accepts an argument with such a name. The
no-coordinate-leak property is therefore checked, not claimed.

---

## 8. Limitations

Each limitation below is measured rather than anticipated.

1. **KPI-grade runs exist, but only on the waypoint mission.** The canonical
   topology now produces grade-eligible results (Section 6.5), so the earlier
   position that no run qualified no longer holds. The limit has moved rather
   than disappeared: those runs fly a stub pilot to a waypoint under ArduPilot,
   while the tracking results in 6.1 and 6.2 remain Project AirSim evidence and
   are not grade-eligible. Bringing the perception stack onto the canonical rail
   is the next boundary, and it is not a small one — that rail has no renderer.

2. **Resolved.** The occupancy map was described here as containing buildings
   only, following a 9 m flight that contacted street furniture at (48.3, −0.9)
   where the map reported 7.4 m of clearance. That was a description of the
   sampling band, not of the city: the map was built by collapsing voxels over
   15–55 m AGL while the demos cruise at 9 m, so nothing shorter than a building
   could appear. Rebuilt over 6–14 m the same point measures 1.8 m, and the
   Shield now performs real obstacle avoidance — 58 interventions on a demo that
   previously recorded 0 because there was nothing to avoid. Adopting it also
   required a street mask, since "on the road" had been inferred from the
   absence of obstacles. See
   `docs/FINDING-the-occupancy-map-was-looking-elsewhere.md`.

3. **Depth is quantised to one metre.** The depth stream arrives as `16UC1` at
   whole-metre granularity despite a float pixel request. It supports proximity
   detection but not a metric standoff requirement such as "hold 10 m".

4. **Resolved.** The requirement "hold 10 m from a pedestrian, with different
   policies per object class" was a command-line parameter and therefore not
   hashed, not audited and not enforced. `SubjectStandoff` is now a constraint
   type: `subject_class` selects which rule binds, the Shield checks it against
   a 3 s forecast, and a breach is repaired by removing the closing component of
   velocity so the aircraft may still circle at the held range. Demonstrated by
   commanding the pilot to hold 3 m against a 5 m rule — the aircraft spent 0 of
   501 ticks inside the ring, closest approach 9.7 m, P0 escape rate 0.0.

5. **Range from apparent width assumes a car.** `implied_range_from_width()`
   uses `object_width_m = 4.0`. Applied unchanged to a pedestrian of roughly
   0.5 m width, it would report the subject at approximately eight times the true
   distance.

6. **The learned VLA path cannot track a moving vehicle in real time.** Measured
   inference cost is 225 ms per token over 12 tokens, giving 2.7 s per decision
   out-of-process and 0.37 Hz. A target at 2 m/s travels 5.4 m within one
   inference. Section 6.4 quantifies this.

---

## 9. Work package status

| WP | Component | Status |
|---|---|---|
| WP1 | Policy DSL | In use. Policies validated and hashed. Five constraint types, including the per-object stand-off requested at the 19 August review. |
| WP2 | Prefix compiler | Reduced version in use on the VLA path. |
| WP3 | Safety Shield | Implemented, tested, and exercised under conflict. P0 escape rate 0 on every flight recorded. |
| WP4 | Stress harness and determinism | Manifest and KPI computation implemented, unit-tested, and emitted by every rail. The canonical ArduPilot SITL + MAVROS 2 topology produces **KPI-grade runs** (Section 6.5). Scenario sweep harness not built. |

---

## 10. Plan for the next period

Ordered by contribution to the acceptance criteria rather than by effort.

**10.1 Bring the perception stack onto the canonical rail.** The topology
question is settled: MAVROS 2 is in place and `is_kpi_grade()` passes runs on
it. What is not settled is scope. Those runs carry a stub pilot on a waypoint
mission, because ArduPilot SITL has no renderer and therefore no camera, while
the tracking results that make up most of this report still come from Project
AirSim and remain outside the gate. Closing that means feeding AirSim imagery
to a Guardrail driven over MAVROS — the two simulators cooperating rather than
substituting — and it is the largest remaining piece of work.

**10.2 Detector throughput and the recording trade.** Section 6.3 shows the
threshold is reachable today with recording off, and that the recorder is the
consumer. Formalise the split: a measurement mode that records nothing, and a
demonstration mode that does and is labelled as such.

**10.3 Extend the scene with pedestrians and additional vehicles.** Requested at
the 19 August review, and now unblocked: the per-object stand-off exists as a
policy rule and `object_width_m` is derived per class, which were the two things
that would have made a pedestrian demonstration meaningless. It needs a
pedestrian asset added to the simulator scene. `policies/follow_pedestrian.yaml`
already carries the requested rules — 10 m from a person, 5 m from anything
else, on a lower altitude band because the camera's blind spot at 9 m is 7.7 m
and therefore inside the stand-off itself.

**10.4 Give every `FenceGuard` caller the street mask, then lengthen the detour
search.** The gap policy's detour is 15.0 m against a 14 m search range, so it
reports no gap. Raising the range recovers it and simultaneously lets a
mask-less guard route around a corridor-spanning fence's end, so the two have to
move together. Both halves are pinned by a test.

**10.5 Populate the occupancy map beyond this one city.** The map is now built
over the band the aircraft flies in, but only for `Demo_day`. Every other scene
still carries a map sampled at 15–55 m and therefore has the same blind spot
this period's incident exposed.

**10.6 A faster detector, if throughput becomes binding again.** The candidate
is YOLO-World: open-vocabulary at 30–50 Hz, with text prompts compiled into
weights so there is no runtime language cost. Retraining a closed-vocabulary
detector to recognise colours is not recommended; it would remove the
open-vocabulary interface without addressing throughput.

---

## 11. Artefact index

| Content | Path |
|---|---|
| Per-flight metrics | `demo/out/<tag>/metrics.json` |
| Per-tick flight log | `demo/out/<tag>/flight_log.jsonl` |
| Shield audit trail | `demo/out/<tag>/audit.jsonl` |
| Safety Shield | `guardrail/shield.py` |
| Action and policy types | `guardrail/models.py` |
| KPI computation | `guardrail/kpi.py` |
| Determinism manifest | `guardrail/manifest.py` |
| Tracking mission | `demo/follow_vlm.py` |
| Target-state estimator | `demo/target_state.py` |
| ArduPilot SITL rail | `sitl/` |
| Colour gate analysis | `docs/FINDING-colour-gate-and-sunlight-aug15.md` |
| Estimator and control analysis | `docs/FINDING-target-estimator-and-control-aug17.md` |
| AerialVLA prompt-sensitivity study | `docs/FINDING-what-drives-aerialvla.md` |
| VLA and Shield simultaneity study | `docs/RESULT-vla-guardrail-simultaneity.md` |
| Fine-tuning report | `docs/aerialvla-ft-report.md` |
