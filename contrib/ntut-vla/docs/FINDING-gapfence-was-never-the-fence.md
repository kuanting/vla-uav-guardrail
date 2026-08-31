# The gap-fence flight was never failing because of the fence

**Date:** 2026-08-11
**Resolves:** the negative result recorded in
`docs/RESULT-traffic-and-gapfence-aug2026.md`.
**Flights:** `vlm_gapfence`, `vlm_gapfence2`, `v2_gap`, `v2_gap2`, `v2_gap3`.

---

## The result

| flight | change | within 30 m | mean sep | Shield |
|---|---|---|---|---|
| `vlm_gapfence` | baseline | 0.261 | 33.5 m | 9 |
| `vlm_gapfence2` | anticipatory side choice | 0.284 | 31.5 m | 22 |
| `v2_gap` | + clearance-aware detour | 0.273 | 32.3 m | 4 |
| `v2_gap2` | + forecast-based gate | 0.262 | 30.6 m | 2 |
| **`v2_gap3`** | **`--want-width` 0.10 → 0.20** | **0.759** | **24.0 m** | 12 |

Three careful fence fixes moved the number by 0.001. One parameter moved it by
0.497.

## What was actually wrong

`servo()` sets forward speed from apparent box width:

```
err = (want_w_frac - w_frac) / want_w_frac
fwd = clip(err * speed_max, -0.4 * speed_max, speed_max)
```

That is a proportional controller on a *stand-off*, and `--want-width 0.10` means
"hold the target at 10% of frame width". At the gap policy's 13 m cruise that
works out to roughly a **30 m stand-off** — and the metric is "fraction of the
flight within 30 m". The aircraft was doing exactly what it was told, parked on
the threshold, and the metric read it as failure.

The instrumented log said so plainly once it was asked the right question:

| separation | ticks | raw command | emitted | box width |
|---|---|---|---|---|
| 15–25 m | 97 | 1.83 m/s | 1.81 | 21 px |
| **25–35 m** | **347** | **1.41 m/s** | **1.00** | **29 px** |
| 35–50 m | 99 | 1.85 m/s | 1.77 | 18 px |

At 29 px against a 40 px target the error is 27%, so the servo asks for about
1.1 m/s against a car doing 2.0. Not because it was throttled — the **raw**
command, before the fence gate touches it, was already below the car's speed.
That is the whole story, and it rules the fence out by construction.

## Why the earlier diagnoses were wrong

Both were real defects, both were fixed, and neither was the binding constraint.

**"It picks the wrong side."** True — the first flight slid west to x = 30.9, the
closed end. Fixed. But `v2_gap` then put 277 of 552 ticks at x > 43, inside the
gap, reaching x = 50.3, and the score did not move. **The aircraft was already
using the gap.**

**"The gate throttles it."** Also true — the gate braked for a *shrinking
distance* rather than a predicted incursion, so flying north up the gap at x = 47
was slowed by the fence corner at (43, 1) that the trajectory passes cleanly.
Fixed, and the gate scale went 0.62 → 0.85 with `hold` ticks 100 → 27. The score
still did not move, because the raw command was the limit, not the gate.

The honest lesson is procedural: **the first two fixes were chosen from a plausible
story, and the third from a measurement.** The story was wrong twice. Instrumenting
`raw` against `emitted` against separation is what settled it, and that
should have been the first step rather than the third.

## What is kept

All three fence changes stay. They are correct, they are pinned by 14 tests in
`tests/test_fenceguard.py`, and two of them fixed real safety-adjacent behaviour:

* the side choice stops the aircraft committing to the closed end of a fence;
* clearance-awareness stops the detour leaving the road — that one was a
  regression I introduced, and it cost 298 Shield interventions on the
  corridor-spanning fence before it was caught;
* the forecast gate stops the aircraft being throttled by fence geometry it is
  going to pass rather than enter.

They simply were not why the gap flight lost the car.

## Consequence for the other policies

`--want-width` is coupled to cruise altitude and nobody had written that down.
The unfenced flights cruise at 9 m and 0.10 is right for them —
`vlm_nofreeze` and `v2_track_r2` both score 1.000 within 30 m. The gap policy
forces 13 m (its altitude band is 10–17, set to clear street furniture the
obstacle map cannot see), and at 13 m the same angular stand-off is a much larger
ground distance.

**Any policy that changes cruise altitude has to revisit `--want-width`.** That
is now stated in `run_follow_vlm.ps1` rather than left to be rediscovered.
