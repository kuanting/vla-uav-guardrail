# The controller could not see buildings

**Date:** 2026-08-25
**Status:** fixed, measured in flight, six regression tests added.

## The symptom

Chasing the car around the corner at NED (40, 21), the follow controller
emitted `v = (0.06, 4.00)` m/s - full speed North, straight at a building - for
about forty consecutive control ticks. The Shield repaired every one of them.
Safety held: P0 escape rate 0.0 throughout. But the outcome at that corner was
decided by sub-metre differences in approach:

| | violations | sep_end | det_hit_rate |
|---|---|---|---|
| one flight | 50 | 16.8 m | 1.000 |
| a near-identical flight | 163 | 58.1 m | 0.629 |

Same code, same policy, same route. The second one lost the car.

## The cause

`FenceGuard` exists precisely to prevent this. Its own docstring says so:

> Without this the servo commands "go to the car" every tick and the Shield
> refuses it every tick. Neither changes its mind, so the aircraft chatters
> against the boundary - measured at 659 corrections in 795 ticks.

But `fence_mode` was logged as `clear` on **597 of 597 ticks**. FenceGuard never
engaged, because both of its methods opened with a fence-shaped guard clause:

```python
def gate(...):
    if not self.polys:
        return 1.0, None, False
def slide(...):
    if not self.polys or speed < 1e-3:
        return 0.0, 0.0, float("inf")
```

The demo policy declares **no** no-fly zone. So on every tracking flight the
entire class was dead code, and building clearance was left purely reactive, in
the Shield alone.

The occupancy grid and the street mask were already passed in, already stored,
and already consulted by `_clear_of_obstacles` and `_on_street` further down.
Only the two guard clauses were fence-shaped.

## The geometry, which decides what a fix may claim

Flight-band occupancy (6-14 m AGL) at 2.0 m resolution. The car's route runs
straight North along x = 38. At **(38, 22) the map is BLOCKED** - a canopy: road
underneath, solid at cruise height.

Clearance reachable along that line, against clearance reachable if the
aircraft shifts in x:

| y | clearance at x = 38 | best clearance | at x |
|---|---|---|---|
| 18 | 4.00 m | 7.21 m | 44.0 |
| 20 | 2.00 m | 5.85 m | 43.5 |
| **22** | **0.00 m** | **5.00 m** | **43.0** |
| 24 | 2.00 m | 5.00 m | 43.0 |

So the 3.0 m ring **is** satisfiable at this corner - about five metres east of
the car's line. The corridor was never the problem. The controller simply could
not see that it had to move.

## The fix

Two changes, both in `demo/follow_vlm.py`.

**`slide()` no longer requires a fence.** Everything below its guard clause
already handled obstacles; the clause is now
`if speed < 1e-3 or (not self.polys and not self.occ)`, and the three
fence-distance tests inside became conditional on `self.polys`.

**`gate()` gained an obstacle half**, shaped like the fence half but keyed
differently, which is the part that matters:

> Urgency is how far ahead the incursion is, not how close the wall is.

The fence half can scale by current distance because a fence is a region you
approach and then leave. Buildings line the street continuously, so current
clearance sits at 4-5 m for the whole flight. Scaling by it throttled the
aircraft to **0.11 of commanded speed twelve metres before anything was in the
way** - which is exactly how the gap flight lost its car ("slower than the
target for 537 of 552 ticks"). Distance-to-incursion has neither problem:
flying parallel to a wall never enters the ring and is never braked.

One further condition was needed. An incursion counts only if the forecast is
**closing** - inside the ring *and* nearer than now. Without that, an aircraft
already inside the ring and flying **out** of it braked hardest exactly when it
was escaping: measured at scale 0.09 while retreating south from the canopy,
which would have pinned it against the obstacle it was leaving.

## Result in flight

Identical flags, same route, same 8 pedestrians:

| | before | after |
|---|---|---|
| Shield interventions | 56 | **0** |
| `fence_mode` | `clear` 597/597 | `skirt` 58, `near` 14 |
| x reached through the corner | 38.6 | **42.6** |
| `det_hz` | 4.04 | **4.35** |
| `det_hit_rate` / `frac_ticks_seen` | 1.000 / 1.000 | **1.000 / 1.000** |
| `sep_end` | 16.7 m | **16.9 m** |
| P0 escape / NFZ / altitude | 0.0 / 0.0 s / 0.0 s | **0.0 / 0.0 s / 0.0 s** |

The aircraft now steers into the free corridor by itself. The Shield did not
fire once - it is a backstop again rather than the only thing steering.

Checked on the fenced policy too, since `gate()` is shared. `follow_car_nfz.yaml`
still holds at the zone (473 hold ticks against 405) and its interventions fell
from 45 to 0, with tracking slightly better (hit rate 0.71 against 0.662).

Suite 214 -> **220**, six new tests in `tests/test_fenceguard.py` covering: the
canopy is seen without a fence; the detour goes east; the recommended corridor
is itself unbraked; flying parallel to a wall is not braked; escaping the ring
is not braked while pushing further in still is; and with no obstacle map the
old behaviour is exact.

## What this does not claim

The Shield's clearance rule is unchanged and still authoritative. This makes the
controller stop proposing commands the Shield would have to refuse - it does not
weaken what the Shield refuses. `min_clearance_m` for the guard is read from the
same policy constraint the Shield enforces, so the two cannot drift apart.
