# Pedestrians as scenery: what it cost, and what it did not

**Date:** 2026-08-25
**Status:** built, measured, merged. Off by default (`--pedestrians 0`).

The 2026-08-19 review asked for people so the virtual environment reads as a
real city. The scope is explicitly **decoration**: nothing detects them and no
rule refers to them yet. Rules come later, and the machinery is already in
place — `SubjectStandoff` is a constraint type, `subject_class` selects which
rule binds, and `policies/follow_pedestrian.yaml` already carries "10 m from a
person, 5 m from anything else".

## Result

Eight figures, three of them pacing, against a control flown with identical
flags and no figures:

| | control (0) | populated (8, 3 walking) | gate |
|---|---|---|---|
| `det_hz` | 3.43 | **4.04** | ≥ 4.0 |
| `det_hit_rate` | 1.000 | **1.000** | no fall |
| `frac_ticks_seen` | 1.000 | **1.000** | no fall |
| `sep_end_m` | 16.8 | **16.7** | — |
| interventions | 50 | 56 | — |
| P0 escape / NFZ / altitude | 0.0 / 0.0 s / 0.0 s | **0.0 / 0.0 s / 0.0 s** | 0 |

Suite unchanged at 214 passing.

Standing figures cost nothing after they spawn — no per-tick RPC at all. Three
walkers cost 540 pose updates across a 70 s flight, and `det_hz` still came out
*above* the empty control. The honest reading is that run-to-run variance
(measured spread 3.4–4.8 Hz) is larger than anything the figures contribute.

## Three failures worth keeping

**A plain dict is not a `Pose`.** Every spawn failed with `ERROR code: 2.0,
message: Unknown exception` while the same call spawned the car fleet happily.
That error reads like a rejected mesh, and two rounds were spent on the mesh
before the actual cause: `spawn_object_from_file` needs a `Pose` holding a
`Vector3` and a `Quaternion`, with `frame_id`. A dict of identical shape is
refused. `demo/moving_car.py:390` had it right all along.

**Editing a rigged GLB in place leaves too much behind.** The first baker
appended posed positions to the source file and stripped the rig from the JSON.
That is not wrong, but it is unreasonable: the skinning accessors stay in the
buffer, the 45-node armature stays in the scene graph, and — the real defect —
clearing the transform on a mesh node does nothing about the transforms on its
**ancestors**, so an armature scale would still be applied on top of
world-space baked vertices. The writer now emits a fresh minimal static GLB:
one node per primitive, no hierarchy, POSITION and NORMAL and indices only.
137 KB against 592 KB, same vertices, same bounding box.

Normals are recomputed from the posed triangles rather than copied. The source
NORMAL accessor describes the bind pose, so reusing it after skinning lights
the figure wrong.

**A one-off degradation that was not the pedestrians.** The first populated
flight came back at hit rate 0.629 and `sep_end` 58 m, which looks exactly like
a decorative figure stealing the tracker's lock. It was not: `switched=0`, and
the drone deviated at tick 287 *before* detections started failing at tick 112
of the detector's own clock. Both runs fight the same building corner at
(40, 21); the populated one entered it 0.5 m closer and logged 163 clearance
violations against 50. Repeating both conditions settled it — the repeat with
eight figures returned hit rate 1.000 and `sep_end` 16.7 m.

So the corner is **marginally unstable independently of this change**, and a
sub-metre difference in approach tips it. That is a real finding about the
demo, not about pedestrians, and it is not yet fixed.

## Placement

The street mask alone will not do — it marks roads and pavements alike, so
sampling it drops people in the middle of a carriageway. A cell that is street
**and adjacent to a building footprint** is a pavement, which is where people
stand. Figures are then banded 8–45 m from the route: closer and a figure
competes with the subject for the tracker, further and it is decoration nobody
ever sees, since the camera only looks near the route.

Walkers pace one straight verified stretch rather than following a route. A
wandering walker needs its whole path checked against the pavement, and every
metre of that is a chance to put a figure in a carriageway — the one placement
mistake that can cost the demo its lock.

## Assets

Quaternius "Animated Men Pack", CC0, four figures, fetched as GLB from
poly.pizza (1.98 MB). Not in the repository, per the standing rule on large
assets. `tools/bake_glb_poses.py` turns them into static meshes;
`demo/pedestrians.py` degrades quietly when the pack is absent, exactly as the
vehicle fleet does, so the repository stays runnable without third-party files.

Their bind pose turned out to be an A-pose, not the T-pose that was feared, so
baking mostly buys correct world-space geometry rather than rescuing the pose.

## Still open

- The (40, 21) building corner. A sub-metre approach difference decides between
  50 and 163 clearance violations. Worth understanding before it decides a
  recorded demo.
- Rules that refer to pedestrians — no-fly-over, standoff — once decoration is
  no longer the whole scope.
