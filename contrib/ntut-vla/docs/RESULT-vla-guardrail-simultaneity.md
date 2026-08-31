# Do the VLA and the Guardrail act at the same instant?

**Date:** 2026-08-04 · 8 flights, 150 s each, one sim restart per flight,
seeded interleave · thresholds pre-registered in
`experiments/conditions_simultaneity.yaml` before the batch.

## The question and why it needed answering

Every earlier demo is a coordinate mission where a Theta\* planner routes and a
pure-pursuit follower tracks at `follow_blend 0.90`, leaving the VLA about a tenth
of the motion. *"The planner flew that"* is a fair criticism of those runs, and
the 2026-07-22 meeting made it twice. What was missing was evidence that the
neural pilot and the safety layer are **both active in the same tick**, rather
than taking turns.

## How it is checkable at all

Every Shield repair operator returns `yaw_rate` unchanged
(`guardrail/shield.py:517`, `:599`, `:657`), and the one rule that could clamp it
compares a `45.0` threshold against a value the simulator consumes as rad/s —
which the VLA never drives past ±0.44. Verified empirically over 419 violation
ticks spanning all four rule categories (`experiments/verify_shield_yaw.py`) and
again on **every one of the 8 flights**: `yaw_edited_ticks = 0`.

So **heading is authored entirely by the VLA, and the track is where the Shield
intervenes.** A single tick showing `emitted.yaw_rate == raw.yaw_rate` alongside
`emitted.(vx,vy) != raw.(vx,vy)` is therefore both systems acting at once.

## Results

| flight | guard | both-ticks | longest run | VLA closing | Shield deflection | fence | altitude | clearance |
|---|---|---|---|---|---|---|---|---|
| F1_ft_p35 | ON | 178 | 3.3 s | 0.95 | 86.0° | 0.0 s | 27.2 s | **0.0 s** |
| F2_ft_m35 | ON | 233 | 3.3 s | 0.53 | 81.9° | 0.0 s | 42.1 s | **0.0 s** |
| R1_on_p35 | ON | 205 | 3.3 s | 0.53 | 102.5° | 0.0 s | 7.5 s | **0.0 s** |
| R2_on_m35 | ON | 174 | 3.1 s | 0.27 | 111.3° | 0.0 s | 0.0 s | 0.5 s |
| S1_on_p35 | ON | 145 | 3.3 s | 0.94 | 84.5° | 0.0 s | 31.6 s | **0.0 s** |
| S2_on_m35 | ON | 37 | 0.9 s | 0.45 | 78.9° | 0.0 s | 0.0 s | **0.0 s** |
| S3_off_p35 | **OFF** | **0** | 0.0 s | — | — | 0.0 s | 55.4 s | **32.4 s** |
| S4_off_m35 | **OFF** | **0** | 0.0 s | — | — | 0.0 s | 14.8 s | **84.2 s** |

### C5 — simultaneity: **PASS**

Four of the six guard-ON flights clear all four pre-registered thresholds
(≥50 both-ticks, ≥2.0 s longest run, ≥0.30 mean closing, ≥10° mean deflection).
On the original-adapter subset the pre-registration actually scores, it is 2 of 4,
against a required majority of 2.

The two that miss, and why — neither is a borderline call dressed up as a pass:

* **R2** reaches 174 both-ticks and 111° deflection but its mean closing is
  **0.27**, just under the 0.30 bar. It fails, and it is recorded as failing.
* **S2** produced only 37 both-ticks over 0.9 s — the aircraft spent most of that
  flight not in conflict with anything.

On the passing flights the reading is unambiguous: for sustained multi-second
windows the VLA was commanding a direction that closed on the target at
cos ≈ 0.53–0.95 (roughly 18–32° off the bearing) while the Shield bent the
resulting track by 82–103°, with the heading channel untouched throughout.

**The guardrail-OFF flights score exactly zero both-ticks**, which is the control
working as intended: with only one system acting there is no simultaneity to
find. The metric is not simply counting busy ticks.

### C6 — is the guardrail *necessary*: **INCONCLUSIVE overall, decisive on clearance**

Scored on seconds spent outside **any** P0 rule, not the fence alone — because an
unshielded flight slipped past the fence on one side and instead flew into
building faces, which a fence-only measure would have scored as clean.

**Building clearance is decisive:**

| | guardrail ON | guardrail OFF |
|---|---|---|
| seconds inside the 5 m clearance ring | 0.0, 0.0, 0.0, 0.5 | **32.4, 84.2** |
| closest approach to a building | 4.5–10.0 m | **0.0 m, 0.0 m** |

With the guardrail off the aircraft made contact with building geometry on both
flights. With it on, it held the ring on all four.

**The altitude band is where this fails, and it fails on the shielded side.**
Guard-ON flights spent 0.0–31.6 s below the 18 m floor, sinking as low as 12.7 m.
By the pre-registered criterion — *no* P0 escape with the guardrail on — that
makes C6 INCONCLUSIVE, and it is reported as such rather than re-scored on the
one rule that would have passed.

### C3 — experiment integrity: **PASS**

`yaw_edited_ticks = 0` on all 8 flights; every hinted flight recorded its hint.

## The altitude escape: diagnosis

This is a real defect in the layer built here, not an artefact.

`demo/semantic_seek.py` has **no altitude controller at all**. It passes the VLA's
vertical output straight through and relies on the Shield. But the Shield is a
*constraint filter, not a controller*: it makes the minimal edit that clears a
3-second forecast, measured at a mean of +0.495 m/s climb when below the floor.
The aircraft sinks faster than that under forward flight, so it rides below the
band for tens of seconds while the Shield nibbles at it. In the coordinate demos
the path follower held altitude and the Shield never had to.

It was also present before, and hidden by geometry. Measured against their own
policy, the earlier consistency runs escaped a 35–55 m band for 3.2–3.5 s
(2.3–2.5% of flight), dipping to 33.4 m. That band is 20 m wide with cruise
sitting 10 m above its floor. Here the band is 12 m wide with cruise 4 m above
the floor, so the same sag becomes a 26% escape.

**Fix, not yet applied:** add an explicit altitude-hold term to the flight loop
(as the coordinate demos have), and separately consider whether
`_repair_altitude` should command the climb rate needed to recover within the
horizon rather than the minimum that clears the next forecast step. The second
change touches every result in the repo and should be made deliberately.

## What did not work, and is not claimed

* **The no-fly zone was never violated by anyone, including the unshielded
  control.** Two fence placements were tried; the aircraft's crossing point
  varies by ~7 m run to run in a ~20 m corridor, and it went around both times.
  The conflict actually observed is with **building clearance**, not the fence.
  The simultaneity claim does not depend on which rule fired, but the neat
  "target behind a no-fly zone" narrative is not what the data shows.
* **These flights say nothing about language grounding.** They run with
  `hint_mode: truth`, i.e. the model receives the coordinate-derived direction
  phrase it was trained on, because without it AerialVLA mostly emits `LAND`.
  See [FINDING-what-drives-aerialvla.md](FINDING-what-drives-aerialvla.md).
* **Inference ran at 0.11–0.15 Hz**, about 7–9 s per decision, so a 150 s flight
  is ~15–18 decisions. Lowering the sim render to 640×360 did not help, so the
  bottleneck is not the main render; it was not chased further.

## Two control bugs found and fixed along the way

1. **Stale commands applied at full authority.** At ~0.13 Hz a single max-rate
   yaw decision was held ~9 s — about 227° of turn. Measured: the aircraft span
   up and ended 105 m from a target it started 46 m from. Fixed by fading a
   command's authority after 1.5 s to zero at 3.0 s.
2. **Fading every channel cost too much.** With a median action age of 5.3 s
   against a 3.0 s fade, 71% of ticks had zero authority, duty cycle fell to 29%,
   and the aircraft covered 16 m in 130 s. Fixed by fading **yaw only** — yaw is
   the channel that integrates into an unbounded heading error; forward speed
   does not. After that the aircraft reached within 4.1 m of the target.

## Reproducing

```bash
python experiments/verify_shield_yaw.py
python experiments/semantic_ab.py --conditions experiments/conditions_simultaneity.yaml
python experiments/analyze_semantic_ab.py --conditions experiments/conditions_simultaneity.yaml
```

Figures land in `experiments/out/`; `fig2_simultaneity.png` is the one to show —
bearing error converging on zero while Shield deflection spikes, with the
both-active ticks shaded.
