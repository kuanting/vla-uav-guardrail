# Discrimination among identical cars, and a gap the drone could not use

**Date:** 2026-08-11
**Flights:** `vlm_traffic`, `vlm_gapfence`, both on the new `SKM_SportsCar`
target. Baseline `vlm_sportscar` (single car, no fence) for comparison.
**Raw:** `demo/out/<tag>/metrics.json`

| | `vlm_sportscar` | `vlm_traffic` | `vlm_gapfence` |
|---|---|---|---|
| scene | 1 car | **5 cars** | 1 car |
| policy | `follow_car` | `follow_car` | **`follow_car_gap`** |
| ticks | 506 | 505 | 578 |
| detector rate | 4.62 Hz | 5.23 Hz | 4.78 Hz |
| detector hit rate | 1.000 | **0.913** | 0.996 |
| mean separation | 9.6 m | 16.4 m | 33.5 m |
| min separation | 2.1 m | 1.7 m | 12.4 m |
| within 30 m | 0.996 | **0.952** | **0.261** |
| ticks held at fence | 0 | 0 | **147** |
| Shield interventions | 0 | 0 | 9 |
| NFZ time | 0.0 s | 0.0 s | **0.0 s** |
| altitude escape | 0.0 s | 0.0 s | 0.0 s |

---

## Result 1 — the colour test really does discriminate

This is the answer to the demo's own limitation number one, "the noun does most
of the work".

Four distractor vehicles were added, all **the same mesh as the target**
(`SKM_SportsCar`), unpainted so they render blue-grey, in lanes 3 m either side
of the target's, at four different speeds and four different phase offsets, two of
them driving the other way. The target is the only painted vehicle
(`M_Orange`, which renders white on this mesh).

Same mesh is the point. An earlier plan mixed a sports car with the old offroad
buggy, which would have confounded shape with colour — a correct lock could have
been the paint or could have been that the other vehicle looks like a roll cage.
Here silhouette, apparent size and detector affinity are held constant and colour
is the only free variable, so the noun *cannot* separate the candidates. Every
distractor is as much "a car" as the target is.

The drone tracked the target: **95.2% of the flight within 30 m, mean 16.4 m,
closest 1.7 m** — and separation is measured against the target's ground truth, so
a lock on a distractor would show up as a large and erratic separation. It does
not.

The detector hit rate fell from 1.000 to **0.913**, and that drop is the cost of
discrimination rather than a regression: more candidates are proposed and more are
rejected by the colour gate. Mean separation roughly doubled, 9.6 m to 16.4 m —
the task is genuinely harder and the tracking is genuinely looser.

The guardrail recorded 0 interventions, NFZ 0.0 s, altitude 0.0 s. Five moving
vehicles instead of one changed nothing about the safety layer.

## Result 2 — the gap-fence flight is a negative result

> **SUPERSEDED 2026-08-11.** The diagnosis below — that `FenceGuard` is
> reactive and picks the wrong side — identified two real defects, both since
> fixed, and neither was why this flight lost the car. The actual cause was
> `--want-width 0.10` at the policy's 13 m cruise, which commands a ~30 m
> stand-off and parks the aircraft exactly on the metric threshold. Corrected
> to 0.20, the same policy scores **0.759** within 30 m against the 0.261
> recorded here. See `docs/FINDING-gapfence-was-never-the-fence.md`.


`follow_car_gap.yaml` had never been flown. It fences x 26–42 of a 20 m corridor
and leaves 7 m of legal road at x 43–50, and the claim it was written to
demonstrate is the interesting one: *a rule that costs the aircraft its preferred
path without costing it the target.* That is what a usable safety layer looks
like, as opposed to `follow_car_nfz.yaml` which fences the whole corridor and
simply ends the mission.

**Half of that claim held and half did not.**

What held — the rule, absolutely. NFZ time 0.0 s, `nfz_entered` false, altitude
escape 0.0 s. The aircraft was held outside the zone for **147 ticks** and the
Shield intervened 9 times. Offline pre-flight checks had already confirmed the
policy was flyable (`test_gap_fence_leaves_a_flyable_gap_BEFORE_flying_it`), so a
failure here would have been the controller's, not the policy's.

What did not hold — the aircraft never exploited the gap. Mean separation 33.5 m
and only **26.1%** of the flight within 30 m, against 99.6% on the unfenced
flight. The rule cost the path *and* the target.

### Why, specifically

The car drives north up lane x = 38, straight through the zone. To use the gap the
aircraft has to be east of x = 43 *before* it reaches y = 2, which means starting
a 5 m lateral displacement while the target is still 10–15 m away and moving at
2 m/s.

`FenceGuard` is reactive: it brakes from 12 m out and only then probes
perpendicular for a gap (`slide()`). By the time the lateral search begins the
aircraft is already decelerating, and 5 m of sideways travel at a reduced speed
costs more ground than the target gives away. The trace shows it drifting *west*
to x = 30.9 at one point — the wrong side entirely, because the perpendicular
probe is symmetric and has no reason to prefer the side the gap is on.

Two concrete fixes, neither attempted here:

1. **Commit to a side before braking.** `slide()` should be consulted at the
   *brake* distance rather than at the stand-off, and should prefer the side with
   more legal road, which is knowable from the policy geometry alone.
2. **Bias the lateral probe by the fence's own extent.** The gap is at
   x 43–50 and the fence stops at x = 42; that asymmetry is in the polygon and is
   currently ignored.

Until one of those lands, `follow_car_nfz.yaml` remains the honest NFZ demo — it
shows the rule is absolute — and the gap variant should be presented as work in
progress, not as a capability.

## What did not change, on any flight

The guardrail. Not one line of `guardrail/shield.py` and not one policy value was
altered for the new mesh, the new fleet, or either flight. Every flight recorded
NFZ 0.0 s and altitude escape 0.0 s.

## Known faults that fired during these runs

`SetObjectPose ... not movable` appeared on both flights and **got worse with more
vehicles**, as expected — five actors mean five chances per tick. `MovingCar`'s
self-healing respawn absorbed it, but a respawn returns a vehicle to its start,
so any tick after the first respawn is measuring a target that teleported. On
`vlm_gapfence` this fired around tick 500 of 578; on `vlm_traffic` it fired for
several vehicles including the target. Both means should be read as "at least this
good". The root cause is still undiagnosed and is now the largest single threat to
the fidelity of these numbers.

## Reproducing

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe demo\follow_vlm.py `
  --object "a white car" --tag vlm_traffic --max-s 60 --det-thresh 0.008 `
  --car-speed 2.0 --car-stop-s 6 --traffic 4 `
  --policy policies\follow_car.yaml --straight
```

`--traffic N` adds N background vehicles (capped at 4, one per lane);
`--traffic-every` sets how many ticks between their teleports. The target always
updates every tick. Budget at the default: 30 RPC/s against 10 for a single car.
