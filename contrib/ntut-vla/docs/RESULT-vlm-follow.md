# A drone that follows a named moving object, through the guardrail

**Date:** 2026-08-04 · `demo/follow_vlm.py` · JapaneseCity, car at 3 m/s, 120 s flights

## The goal, and why the previous approach could not reach it

The requirement was plain: a clearly visible object that moves, and a drone that
follows it because it was *told what to follow*, with the safety layer still in
charge.

AerialVLA cannot do this, and that is measured rather than assumed. Across 108
controlled forward passes per adapter and several flights: its object-description
slot does not steer (correct and wrong colour words give indistinguishable
actions), without a coordinate-derived bearing phrase it emits stop-and-land on
11 of 18 real frames, and flown against a real car mesh it never moved at all —
12 of 12 inferences returned the identical token triple `[0, 49, 49]`. Details in
[FINDING-what-drives-aerialvla.md](FINDING-what-drives-aerialvla.md).

So the model was replaced, not the goal. This is still vision-language-action —
words choose the target, the camera finds it, the controller acts — but with a
model whose language input actually reaches its output.

## What it is made of

| part | what it does |
|---|---|
| **OWL-ViT** (`google/owlvit-base-patch32`, 153 M params, Apache-2.0) | open-vocabulary detection: text in, boxes out. You type "an orange car" and it returns where that is in the image. |
| **Servo controller** (`servo()` in `demo/follow_vlm.py`) | box offset → yaw rate; box width vs a target width → forward speed; altitude error → climb rate |
| **The car** (`demo/moving_car.py`) | `SM_Offroad_Body` from `PASBlocks/Plugins/Rover` — a real vehicle mesh, 3.70 × 1.79 × 1.16 m — painted orange, driven at 3 m/s along a street verified clear for 100 m |
| **The Shield** (unchanged) | validates every action against the same policy as every other mission |

Nothing in the steering path receives the car's coordinates. The car's position is
used only to spawn it, to set the initial heading, and — offline — to score.

## Results

Two runs of the main condition, two controls, 120 s each:

| flight | words | car present | closest | mean separation | final | within 30 m |
|---|---|---|---|---|---|---|
| **main, run 1** | "an orange car" | yes | 3.2 m | **29.4 m** | 4.3 m | **48.8%** |
| **main, run 2** | "an orange car" | yes | **0.4 m** | **20.7 m** | 12.2 m | **75.7%** |
| control: wrong colour | "a blue car" | yes | 6.7 m | 31.4 m | 40.6 m | 48.9% |
| control: no car | "an orange car" | **no** | — | — | — | — |
| *baseline:* AerialVLA, no hint | "orange car" | yes | 12.4 m | 38.3 m | 47.8 m | 34.5% |
| *baseline:* AerialVLA + coordinates | "orange car" | yes | 7.8 m | 50.4 m | 30.7 m | 18% |

**The drone follows.** It closed to 0.4 m on the second run and spent three
quarters of that flight within 30 m of a target moving at 3 m/s. Both runs beat
both AerialVLA baselines on every measure, including the one that was handed the
car's true coordinates.

Note the AerialVLA no-hint baseline scores 12.4 m "closest" while never leaving
its spawn point — the car drove past it. That is why mean separation and
time-within-range are the metrics here and closest approach is not.

## Two things that were fixed on the way, and stayed fixed

**Latency.** AerialVLA ran at 0.11 Hz — about 9 s per decision, which cannot
close a loop on anything that moves. The detector runs at **3.2–4.9 Hz**, roughly
30× faster. Four hypotheses were tested before the real cause was found:

| tried | result |
|---|---|
| decode camera frames lazily instead of in the subscription callback | no change (still a correct fix; it removed genuinely wasted work) |
| remove the unused Chase camera (1280×720) and all depth streams | no change |
| throttle camera publishing to 2 Hz | no change |
| **run the model in a separate process from the control loop** | **1330 → 225 ms/token** |

The discriminating test: with a flight already airborne and two 7B models
resident on the same GPU at the same instant, the separate process ran at
225 ms/token while the in-flight one ran at ~1330. So it was never GPU
contention — it was the GIL, because `generate()` does per-token Python work and
a 10 Hz asyncio loop forces constant interpreter handoffs. That work lives in
`demo/vla_server.py` and remains useful for any large model; OWL-ViT is small
enough not to need it.

**The altitude escape.** Earlier flights sagged tens of seconds below the floor
even with the guardrail on — 31.6 s down to 13.0 m against an 18 m floor. The
cause was architectural: those scripts had no altitude controller and leaned on
the Shield, which is a *constraint filter* making minimal corrections
(measured +0.495 m/s mean climb), not a regulator. `follow_vlm.py` has an
explicit altitude-hold term. **Altitude escape is now 0.0 s on every flight.**

## What does not work, stated plainly

**The colour word does almost nothing.** "a blue car" against the same orange car
scored 48.9% within 30 m, indistinguishable from "an orange car" at 48.8% on the
matched run. The noun is doing the work; the adjective is close to inert. This
system grounds *"car"*, not *"the orange one"*.

**The detector fires on nothing.** With no car in the scene at all it still
reported a detection on 75% of frames — false positives on city clutter. The
continuity filter (reject boxes that jump more than 35% of the frame width)
suppresses the worst of it, but a confident wrong answer is still possible, and
with a general query like "a car" the drone chased city objects and ended 81 m
away. Specific queries matter, and so would a proper track-confirmation stage.

**Scores are low in absolute terms** — 0.03–0.07 for the true car at 22 m. The
box lands in the right place, which is what a servo loop needs, but this is not a
system you would trust to say *whether* the object is present.

**n = 2** on the main condition. The two runs differ substantially (29.4 vs
20.7 m mean separation), so treat the numbers as demonstrating capability, not as
a calibrated performance figure.

## The guardrail, throughout

Every flight: **NFZ 0.0 s, altitude escape 0.0 s.** The Shield ran on every
action exactly as in the coordinate missions — the pilot changed, the safety
layer did not, and it needed no modification to accept a completely different
kind of controller. That is the architectural claim this project has been making,
and it survived swapping the brain.

## Running it

See [TUTORIAL-FOLLOW-THE-CAR.md](../TUTORIAL-FOLLOW-THE-CAR.md), or:

```powershell
.\run_follow_vlm.ps1
```
