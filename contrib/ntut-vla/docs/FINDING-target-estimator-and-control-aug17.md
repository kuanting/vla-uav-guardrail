# How the drone is controlled, and why the answer was not PID

**Date:** 2026-08-17
**Trigger:** "bagaimana cara kontrol dronenya? apakah menggunakan sistem PID?
apakah bisa menggunakan sistem PID agar lebih smooth? atau tidak disarankan
karena hanya simulasi?" — and "apakah masih ada bug atau error?"

---

## 1. What the control stack actually is

The outer loop is **pure proportional**, three channels, with no I and no D
anywhere in our code:

| channel | law | file |
|---|---|---|
| yaw | `yaw_rate = yaw_gain · bearing` | `demo/follow_vlm.py` `servo()` |
| forward | `fwd = ((want_w − w_frac)/want_w) · speed_max` | same |
| altitude | `vz_up = (cruise_alt − up) · alt_gain` | `fly()` |

then a slew-rate limiter (`RateLimiter`), then the Shield, then
`move_by_velocity_async`.

**PID already exists — below us.** `simple_flight` closes velocity → attitude →
rates inside the simulator, and ArduPilot does the same on real hardware. Our
layer is guidance, not stabilisation. "Should we use PID" was therefore the wrong
question: PID is already in the loop, and what we write is the outer law.

## 2. Is it rough? Only in one channel

Per-tick step in the emitted command, over three flights:

| channel | p95 | max |
|---|---|---|
| yaw_rate | 0.006 rad/s | 0.052 (**3 °/s per tick**) |
| vz_up | 0.003 m/s | 0.084 m/s |
| **forward** | **0.897 m/s** | **1.897 m/s in one 0.1 s tick** |

Yaw and altitude were already smooth. Only the forward channel was rough, and the
slew limiter was clipping it on **20% of ticks** — masking the noise while adding
phase lag.

## 3. Why PID was the wrong fix

The cause was **P acting on a noisy measurement**, not a missing term. The
detector's box width — which drove the forward servo — jitters **p95 37.7%, max
71%** between consecutive detections in traffic, and a 30% width jump is about
5.6 m/s of commanded change. The detector also runs at 3.8–5.6 Hz against a 10 Hz
loop, so **62% of ticks reused a stale box** and then jumped.

* **D** would differentiate exactly that noise.
* **I** would wind up every time the Shield overrides the actuator, which is the
  whole point of having a Shield.
* **Filtering had already been flown and lost**: `DESIGN-orbit-building-task.md`
  records a low-pass plus slew on the radial loop making radius hold clearly
  worse, 24.3 ± 10.1 m → 33.6 ± 25.2 m — *"the filter costs more phase than it
  buys in noise"*.

That document also named what had never been tried, and that is what was built.

## 4. The estimator

`demo/target_state.py` — a constant-velocity Kalman filter on the target's world
position, updated from detections, predicted between them. Polar measurements are
converted to Cartesian with a rotated covariance rather than linearised.

**Only measurements go in**: the box centre, the depth range, the aircraft's own
pose. No target ground truth, ever — `tgt_x`/`tgt_y` remain scoring-only, so the
demo's standing claim that the only steering input is the detector's box survives
intact.

**Six flights, arms interleaved**, `--no-target-estimator` as the control:

| | raw forward step p95 | max | limiter clipping | mean separation |
|---|---|---|---|---|
| **estimator ON** | **0.2831** | 1.2527 | **3.0%** | 17.3 m |
| box width (old) | 0.7161 | 2.1976 | 15.6% | 13.8 m |

Per-arm ranges do not overlap: ON [0.262, 0.305], OFF [0.532, 0.915]. Command
roughness down **2.5× at p95**, limiter saturation down **5.2×**.

Tracking quality is unchanged and perfect in both arms: hit rate 1.000, target
held on 100% of ticks, within 30 m 1.000, and **0 Shield interventions, NFZ
0.0 s, altitude 0.0 s** throughout.

**The cost, stated plainly.** The aircraft now sits at 17.3 m against a nominal
15.8 m stand-off, where the width servo sat at 13.8 m. Both miss nominal by a
similar margin in opposite directions, but the estimator holds *further back*,
and closest approach went 7.4 m → 11.8 m. On video that reads as less intimate
following.

## 5. The P-only lag, and why an integrator was still the wrong answer

The first flight held **23.4 m against a 15.8 m stand-off**. That is textbook
proportional lag: the car drives at 2 m/s, so the loop settles at the fixed point
`(r − 15.8)·0.25 = 2.0` → 23.8 m. Measured 23.4 m.

The textbook fix is an I term. It is still wrong here — it would wind up whenever
the Shield overrides the actuator, and it would drain slowly after the car stops,
pushing the aircraft in at exactly the wrong moment.

The estimator already knows the target's velocity, so it is fed **forward**
instead (`TargetState.range_rate`). No memory, no windup, and it goes to zero the
instant the car stops.

| | mean separation | min |
|---|---|---|
| P only | 23.4 m | 17.8 m |
| **P + velocity feedforward** | **17.9 m** | **11.9 m** |

Two tests pin it: one that it removes the lag, one that it reads ≈0 on a parked
car — the failure an integrator would have.

**Is any of this discouraged because it is "only simulation"?** No, the opposite.
The inner loops are PID in both the simulator and ArduPilot, so what has to
transfer to hardware is the Shield's contract, not this tuning. Building the
guidance law on an estimate rather than on a raw measurement is what makes it
transferable at all.

## 6. Bugs found and fixed

**(a) The yaw cap compared degrees against radians.** `yaw_rate_max_dps: 45.0`
was compared with `Action4D.yaw_rate` in rad/s at three sites in
`guardrail/shield.py`, so the cap effectively sat at 2578 °/s and this P1 rule
could never fire.

I had previously argued against fixing it, on the grounds that it would break
**F1** — "the Shield never edits yaw" — which the follow demo's headline rests on.
**That argument was wrong, and the measurement said so**: the worst yaw command
anywhere across all three demos is **11.8 °/s**, four times under the cap. Fixing
the unit leaves the follow demo untouched and clamps only the AerialVLA path
(±1.1 rad/s = 63 °/s), which is the rule finally doing its job. F1 is now asserted
directly in `tests/test_guardrail_coverage.py` rather than assumed.

**(b) Depth arrives quantised to 1 m although the config asks for float.** The
robot config sets `pixels-as-float: true`; the stream is `16UC1`, so every
`rng_m` is a whole metre. This had to be diagnosed twice — once from the orbit
results, once from the follow results — because nothing announced it. It now
prints once per run:

    [depth] encoding '16UC1', 2 bytes/px -> uint16, QUANTISED TO 1 m.

`object_mask_from_depth` also used a 0.5 m margin, below the resolution of the
signal it tests; raised to 1.0 m.

**(c) `det_hit_rate` flattered a stalled flight.** Fixed the day before: a low-Hz
warning plus `frac_ticks_seen` and `det_hz` in the summary table.

## Reproducing

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe experiments\replay_target_estimator.py demo\out\demo_traffic
C:\Users\natha\.conda\envs\vla-real\python.exe demo\follow_vlm.py --object "a yellow car" --tag ctrl --no-target-estimator ...
```
