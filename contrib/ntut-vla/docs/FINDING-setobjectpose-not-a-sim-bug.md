# "SetObjectPose ... not movable" was never a simulator bug

**Date:** 2026-08-11
**Status:** root cause found, fixed, verified by flight.

---

## What it looked like

```
[car] teleport failed (RuntimeError: ERROR code: 1.0, message: SetObjectPose
      failed. Unable to move object SemCar, check if object state is movable!)
[car] *** THE CAR IS NOT MOVING - respawning it ***
```

It appeared mid-flight after the actor had been teleporting successfully for
hundreds of ticks, the simulator stayed alive, and nothing appeared in the engine
log. The message says *"check if object state is movable"*, which reads like actor
corruption, and it was recorded as exactly that — a sim degradation that got worse
after many restarts, undiagnosed, and eventually the largest single threat to the
fidelity of every flight number we had.

## What it actually was

Across five flights the fault fired at **exactly (38.0, 58.0)** every single time.
That is the last point of `STRAIGHT_ROUTE`.

| flight | first freeze | position | stops | route time |
|---|---|---|---|---|
| `vlm_gapfence` | t = 34.4 s | (38.0, 58.0) | none | 33 s |
| `vlm_gapfence2` | t = 34.3 s | (38.0, 58.0) | none | 33 s |
| `vlm_traffic` | t = 38.6 s | (38.0, 58.0) | 2 × 6 s | — |
| `vlm_stopgo` | t = 48.8 s | (38.0, 58.0) | 2 × 6 s | 45 s |
| `vlm_sportscar` | t = 48.5 s | (38.0, 58.0) | 2 × 6 s | 45 s |

Same position every time, and the *timing* tracks route length plus stop dwell:
34 s with no stops, 48 s with two six-second stops.

With `one_shot=True` the car parks at the end of its route and `pose_at` clamps.
`update()` then asks the simulator to teleport the object **to where it already
is**, on every remaining tick. The sim refuses a no-op teleport and reports it
with a message about movability.

Nothing was corrupt. Nothing degraded. The car had simply arrived.

## Why it mattered so much more than it should have

The self-healing respawn. After 25 consecutive "failures" `MovingCar.update()`
destroyed the vehicle and respawned it — **at the start of its route**. So a
harmless no-op turned into the target teleporting 66 m backwards mid-flight, and
every tick after that was scoring separation against a car that had jumped. That
is why `vlm_sportscar` (9.6 m), `vlm_gapfence` (33.5 m) and `vlm_traffic` (16.4 m)
all had to be reported as "at least this good".

The recovery mechanism was doing all the damage, and it was recovering from
nothing.

## The fix

`update()` now compares against the last pose it actually sent and skips the RPC
when nothing has changed. A no-op teleport is pointless anyway — it was costing an
RPC per tick per parked vehicle. A genuine failure now means a genuine failure:
the car was asked to move somewhere new and could not.

## Verified by flight

Both runs 70 s, longer than any flight in which the fault previously appeared.
**Zero `SetObjectPose` errors, zero respawns.**

| | before | after |
|---|---|---|
| single car | `vlm_sportscar`: within 30 m **0.996**, froze at t = 48.5 s | `vlm_nofreeze`: within 30 m **1.000**, no freeze |
| with traffic | `vlm_traffic`: within 30 m **0.952**, hit rate 0.913 | `vlm_traffic2`: within 30 m **1.000**, hit rate 0.979 |
| Shield interventions | 0 | 0 |
| NFZ / altitude escape | 0.0 s / 0.0 s | 0.0 s / 0.0 s |

**Every separation number can now be quoted without the caveat.**

## Two more faults found while fixing it

**Background vehicles were parking too.** A `one_shot` distractor stops being a
distractor the moment it reaches the end of its route — 66 m at 1.5 m/s is 44 s,
so on a 90 s flight half the traffic was stationary scenery by the midpoint and
the selection problem quietly got easier exactly when it should not. Worse, phase
offsets were absolute seconds: `phase_s = 31 s` against a 32.7 s route left
`BgCar3` parked 1.7 s into the flight. Phase is now a **fraction of each
vehicle's own route**, and distractors drive a thin closed loop instead of a
one-shot run.

**Looping distractors were straying into the target's lane.** With the loop
direction tied to `reverse`, the lane-41 vehicle swung back to x = 39.4 — 1.4 m
from the target's lane, close enough to occlude the very vehicle the aircraft is
trying to pick out. The loop now always runs *away* from the target lane.

**And the fleet is three, not four.** Each looping vehicle needs its loop width
plus a clearance margin, and the legal corridor is only [32, 46] with the target
holding the middle. Four fitted when distractors drove straight and parked; once
they loop, lane 35's return leg reaches x = 33.8 and sits 1.3 m from a vehicle at
32.5. The band does not have the room, so `default_fleet` clamps to three and says
so.

All three are pinned by `tests/test_city_traffic.py` (17 tests), which caught
every one of them before any simulator time was spent.
