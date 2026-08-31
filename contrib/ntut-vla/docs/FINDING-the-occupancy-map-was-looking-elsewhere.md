# The occupancy map was sampling a band the aircraft never flies in

**Date:** 2026-08-25
**Status:** ADOPTED 2026-08-25. The corrected map is now the default. This note
keeps the reasoning, the failed first attempt, and the one thing still open.

## The incident

A 9 m flight struck street furniture at **(48.3, −0.9)** while the Shield's
occupancy map reported **7.4 m of clearance** there, against a 5 m requirement.
The Shield was not wrong about the map; the map was wrong about the city.

## Cause

`demo/build_voxel_map.py` collapses the simulator's ground-truth voxel grid to a
2-D grid over an altitude band, and that band defaulted to **15–55 m AGL**. These
demos cruise at **9 m**. Everything shorter than 15 m — traffic lights, signs,
poles, tree canopies — was therefore structurally invisible.

"The occupancy map contains buildings only", repeated in several documents
including the midterm report, was a description of the sampling band rather than
of the world. Nothing was missing from the simulator's geometry.

Rebuilt over **6–14 m**, matching the altitude envelope in the policy that
consumes it:

| | 15–55 m band | 6–14 m band |
|---|---|---|
| Occupied fraction | 0.346 | 0.315 |
| Clearance at (48.3, −0.9) | **7.4 m** | **1.8 m** |
| Cells newly blocked | — | 564 |
| Cells freed | — | 461 |

The struck obstacle survives even an 8–14 m band, so it is tall: a genuine
hazard at cruise altitude, not kerb clutter. The 461 freed cells are the mirror
image — building tops that exist at 15–55 m and nothing at all at 9 m.

**Under the corrected map the Shield would have prevented the collision.**

## Why adopting it took three coordinated changes

Installing the corrected map alone made the demos worse, and the first flight
said so immediately: detector hit rate fell **1.000 → 0.48**, the target was held
on 68 % of ticks instead of 100 %, and mean separation went from 16 m to 56 m.
The aircraft was not avoiding obstacles, it was being pushed off the road.

With street furniture present a 5 m clearance ring is not satisfiable on these
streets — at y = 20 the widest free point across the road carries **5.2 m**, and
six of twenty-five route waypoints fell below the minimum. Lowering
`min_clearance_m` to 3.0 fixed it: hit rate back to 1.000, target held 100 % of
ticks, P0 escape rate 0.0, and **52 Shield interventions where the same demo
previously recorded 0** — because there had been nothing in the map to avoid.

That pairing works. What held adoption up was a third consequence, conceptual
rather than numeric, and it is resolved further down.

### The map is not a road map

Two tests fail under the corrected map, and both fail for the same reason.

- `test_the_turn_route_stays_on_mapped_road` flags the car's route at
  **(38.0, 22.1)**, where the corrected map is fully occupied. That cell is a
  canopy **over** the road. It blocks a drone at 9 m and does not inconvenience a
  car driving underneath it. The test treats an aerial obstacle map as a
  driveability map, which was harmless while the map held only buildings — roads
  were always free by construction — and is wrong now.

- `test_a_detour_must_stay_on_the_road_not_merely_outside_the_fence` fails from
  the opposite direction: the 461 freed cells open detours over low structures
  that the old map forbade, so "on the road" can no longer be inferred from
  free space either.

Both needed a **street mask** — a separate layer saying where a route may run —
rather than inferring roadness from the absence of obstacles. That is a design
change rather than a threshold, and it is what the rest of this note describes.

### And the gap policy's detour grew past the search range

`policies/follow_car_gap.yaml` describes a corridor at x 43–50. Under the
corrected map its eastern half is solid: **0.0 m at x 48–50, y 0**. The western
end (x 43–45) still carries 3.7–7.8 m, so the geometry is not unflyable — the
gap moved and the policy still describes the old one.

Worth recording: lowering `stand_off_m` does **not** recover it. Tried at 3.0,
2.0, 1.5 and 1.0 m, all report no gap. This is not a threshold that can be tuned
until the demo passes; it is an obstacle.

## What is in the repository

| File | Band | Role |
|---|---|---|
| `occ_day.npz` | 6–14 m | **Default.** What the aircraft can hit at cruise. |
| `street.npz` | — | Where the roads are. A different question; see below. |
| `occ_day_flightband_6to14.npz` | 6–14 m | Named copy of the default. |
| `occ_day_highband_15to55.npz` | 15–55 m | The old default, and now the building-footprint mask that `street.npz` is built from. |
| `ground_2to4.npz` | 2–4 m | Low clutter, the other input to the street mask. |

## The street mask

The two failures above have one cause: *road* was being inferred from *absence
of obstacle*, which holds only while the obstacle map contains nothing but
buildings. `demo/build_street_mask.py` asks the question directly, from three
bands that answer three different things:

```
2–4 m     low clutter: kerbs, walls, parked vehicles, tree trunks
6–14 m    what the aircraft hits at a 9 m cruise   -> the obstacle map
15–55 m   only buildings reach this                -> building footprints
```

A cell is **street** when it is free of buildings *and* free of low clutter:
3752 of 6400 cells. A cell is a **canopy** when it is street below and blocked
at cruise: **128 cells**, drivable by a car and closed to the aircraft. Those
128 are exactly what made the old inference wrong.

Buildings are what makes this work. They are the only structures tall enough to
appear in the 15–55 m band, so that band is a footprint mask rather than the
obstacle map it was being used as. A low-band test alone is not enough: the
voxel grid reports building interiors as empty at 2–4 m, because walls are
surfaces rather than solids, so it calls the inside of a city block a road.

## What changed to adopt it

1. `demo/build_street_mask.py` added; `FenceGuard` takes a `street_mask` and
   `slide()` now requires a detour to be outside the fence, clear of obstacles
   **and** on a road. The three were previously one check named `_on_road` that
   only measured distance from buildings.
2. Both road tests rewritten to ask the street mask instead of the obstacle map.
3. `min_clearance_m` 5.0 → 3.0 in all three follow policies.
4. `occ_day.npz` replaced. Re-flown: detector hit rate 1.000, target held on
   100 % of ticks, P0 escape rate 0.0, and **58 Shield interventions** where the
   same demo previously recorded 0.

## Still open: the gap policy's detour is one metre out of reach

`follow_car_gap.yaml`'s corridor at x 43–50 has a solid eastern end under the
corrected map, so the first viable column sits further east and the detour grows
to **15.0 m**. `slide()` searches 14 m, so it reports no gap at all.

The gap is real — at `reach_m=20` it is found, eastward, at cost 15.0. What
stops that being the fix is that a longer reach lets a guard with **no** street
mask route around a corridor-spanning fence's *end*, which is the off-road
regression the detour test exists to catch. **The reach cannot move until every
`FenceGuard` caller supplies the mask.**

Both halves are pinned by
`tests/test_fenceguard.py::test_the_gap_policy_gap_is_out_of_search_range_once_the_map_is_honest`,
so neither can drift unnoticed.

This one failed in the *safe* direction — no detour found falls through to
braking — which is the hardest kind of failure to see. The gap demo looked like
a policy problem and was a search-range one.
