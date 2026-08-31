# Flying the glTF fleet: what got better, and what got worse

> **CAVEAT ADDED 2026-08-15, after these numbers were written.** Every figure
> below is ONE flight per arm, and a bug found later the same day makes that
> insufficient. The aircraft's takeoff heading was never actually commanded, and
> whether the subject fell inside the 45 deg half-FOV at t=0 swung the traffic
> hit rate between 0.740 and 0.331 on the SAME configuration. That is larger than
> any effect compared here. See
> `FINDING-lost-lock-at-the-intersection-aug15.md` section 3.
>
> The vehicle conclusions may well survive - the six-flight A/B run after the fix
> reproduces hit rates of 0.74-0.80 with the glTF fleet, consistent with the
> 0.740 quoted here and far above the buggy's 0.50 - but the evidence offered
> below is weaker than it appeared, and re-running this with n>=3 per arm is
> outstanding work.

**Date:** 2026-08-15
**Runs:** `demo/out/demo_follow`, `demo_traffic`, `demo_nfz` — `run_follow_vlm.ps1`,
three flights, simulator restarted before each.

The vehicle swap is a clear win on colour discrimination and on robustness in
traffic. It is a **loss on raw detector hit rate in the single-car case**, and
that is reported first rather than buried, because the previous version of this
document would have led with the wins.

---

## Choosing the baseline honestly

Three archived configurations could be quoted, and they give three different
stories:

| run | query | subject | hit | absent |
|---|---|---|---|---|
| `v7_traffic` | a white car | `SKM_SportsCar` + M_Orange | **1.000** | **0.83** |
| `v8_traffic` | an orange car | `SM_Offroad_Body` + M_Orange | 0.495 | 0.439 |
| `v9_traffic` | an orange car | `SM_Offroad_Body` + M_Orange | 0.506 | 0.301 |

**`v7` is not the baseline, despite the perfect hit rate.** Its query was "a
white car" against a city that is 8.5% white, and it called the target ABSENT on
83% of ticks. The 1.000 is the detector reliably finding *pale buildings*. That
run is the regression the user reported after the meeting, not the standard to
beat.

The honest baseline is **`v8`/`v9`** — the orange buggy, which is what was
actually shipping — and, for the single-car demos, `vlm_stopgo` and
`vlm_nfz_smooth`.

## Traffic: better, and not marginally

| | v8 (buggy) | v9 (buggy) | **glTF fleet** |
|---|---|---|---|
| detector hit rate | 0.495 | 0.506 | **0.740** |
| **verdict ABSENT** (target IS present; lower is better) | 0.439 | 0.301 | **0.034** |
| within 30 m | 0.687 | 1.000 | 0.946 |
| mean separation | 20.7 m | 15.2 m | 17.5 m |
| Shield / NFZ / altitude | 0 / 0.0 s / 0.0 s | 0 / 0.0 s / 0.0 s | **0 / 0.0 s / 0.0 s** |

The absent-rate is the number that matters most against the original complaint.
It fell from 0.30–0.44 to **0.034**: the system now almost never claims the car
has gone while it is in frame. That is the difference between a demo that looks
confused and one that looks like it is tracking.

## Single car: the hit rate went DOWN

| | `vlm_stopgo` (buggy) | **`demo_follow`** (taxi) |
|---|---|---|
| detector hit rate | **0.994** | 0.786 |
| within 30 m | 0.992 | **1.000** |
| mean separation | 16.2 m | **15.2 m** |
| verdict ABSENT | not recorded | 0.026 |

A 21% drop in hit rate is real and is not explained away. What it does **not**
do is degrade the following, and the logs say why:

* misses cluster at 20–29 m (24.3% of those ticks) and 10–19 m (10.2%);
* on **70 of 75** miss-ticks the presence monitor still says PRESENT — the
  system knows the car is there, it simply has no fresh box that tick and
  coasts on the last one;
* the colour gate is not the cause: colour p10 = 0.148 against a 0.10 gate, with
  only 6% of detections below 0.12.

So the aircraft spends more ticks coasting and still ends up *closer* to the car
for *more* of the flight. The buggy's 0.994 was bought with a silhouette OWL-ViT
finds easy in isolation, and it collapses to 0.50 the moment other vehicles are
present. The taxi holds 0.786 alone and 0.740 in traffic — worse at its best,
far better at its worst, which is the trade worth making for a demo that has to
survive a crowded street.

## Tracking stability

Computed from `detections.jsonl` by `experiments/compare_track_jumps.py` — a
"jump" is a box centre moving > 60 px between consecutive detections, which at
400×225 cannot be the same vehicle.

| run | jump rate | target switches |
|---|---|---|
| demo_follow | 1.0% | 0 |
| demo_traffic | 1.5% | 6 |
| demo_nfz | 1.0% | 0 |

**No before/after is offered.** The archived runs predate `detections.jsonl`, so
their jump rate cannot be recomputed, and the 14.0% → 0.4% figures in
`FINDING-traffic-rebuild-aug14.md` were measured live under a different target.
Quoting them against 1.5% would be comparing two things that were never measured
the same way.

The 6 switches in traffic looked like occlusion — at t = 38.6 s the box moves to
a distant yellow-ish object at p = 0.065 while a street tree stands in the frame.
**That reading was wrong and is retracted.** Projecting the car's ground truth
into the camera shows it dead centre (u = 201 of 400) and unobstructed when the
lock drops; what put the box on a distant object was the aircraft's own search
rotation carrying the real car out of frame. See
`FINDING-lost-lock-at-the-intersection-aug15.md`.

## The no-fly-zone flight

| | `vlm_nfz_smooth` (buggy) | **`demo_nfz`** (taxi) |
|---|---|---|
| detector hit rate | 0.689 | 0.552 |
| within 30 m | 0.284 | **0.355** |
| mean separation | 45.4 m | **39.6 m** |
| NFZ seconds | 0.0 | **0.0** |
| Shield interventions | 0 | **0** |

Its low numbers are the demo working, not failing: the fence holds the aircraft
outside while the car drives on through the zone, so separation grows to 60 m by
design and `frac_absent` rises to 0.331 because at 60 m the car is a few pixels.
`nfz_s = 0.0` with `interventions = 0` is the point — the fence is respected by
the controller before the Shield ever has to repair anything.

## The guardrail, across all three

**0 interventions, 0.0 s in the no-fly zone, 0.0 s outside the altitude band,
on every flight.** Unchanged, as it has been on every flight ever recorded here.
The vehicle swap touched nothing in `guardrail/`.

## Reproducing

```powershell
.\run_follow_vlm.ps1
C:\Users\natha\.conda\envs\vla-real\python.exe experiments\compare_track_jumps.py demo\out\demo_follow demo\out\demo_traffic demo\out\demo_nfz
```
