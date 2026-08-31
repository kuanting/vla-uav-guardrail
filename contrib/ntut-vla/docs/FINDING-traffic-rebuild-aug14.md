# Why the tracking wandered, and why the motion looked wrong

**Date:** 2026-08-14
**Trigger:** review of `demo_follow` / `demo_traffic` / `demo_nfz` — "object
detection masih kacau dan tidak presisi, object tracking berpindah-pindah tidak
jelas", and "pergerakan mobil tidak smooth dan kasar".

Both were real, both had specific causes, and both are fixed. A third request —
give every car a different colour — turned out to be impossible in this build,
and that is reported rather than worked around.

---

## 1. The motion was rough because of a 1 metre U-turn

Each distractor had its own thin loop, 1.0 m wide with a 0.6 m corner radius, so
it could keep circulating without leaving its lane. Measured, that turned the
heading **10.9° per 0.1 s tick — 109°/s**. A real car manages about 30°/s. The
vehicles were snapping round, not turning.

Replaced with **one shared circuit**, 10 m × 82 m with a 4 m corner radius, driven
at different phases — cars circulating a block, which is what a street looks like.

| | before | after |
|---|---|---|
| peak heading rate | **109 °/s** | **28.7 °/s** |
| closest two vehicles ever came | **0.0 m** | **4.0 m** |
| closest a distractor came to the target | **0.0 m** | **4.0 m** |
| background update rate | 5 Hz (0.4 m steps) | **10 Hz** (0.2 m steps) |

Two further faults fell out of the rebuild:

**Vehicles were driving through each other.** Different speeds on a shared path
means a faster car eventually catches a slower one. Every distractor now runs at
one speed, so the phase gap is constant. The scene still does not read as a rigid
body, because they are spread round the lap and the two long legs run opposite
ways.

**The circuit crossed the target's lane.** Its cross-legs were at the same y as
the ends of the target's route, so a distractor swept through x = 38 exactly
where the aircraft was looking. The circuit now runs 8 m past each end.

## 2. The tracking wandered because every car looked the same

That was the design, and it was the right design for a *controlled experiment* —
hold the mesh constant so colour is the only variable. It is the wrong design for
a demo: with four identical vehicles the colour gate has nothing to separate and
the tracker hops between them.

| | same appearance | mesh + colour variety |
|---|---|---|
| **box-centre jumps > 60 px** | **14.0%** of detections | **0.4%** |
| detector hit rate | 0.979 | **1.000** |
| mean separation | 17.7 m | **10.3 m** |
| within 30 m | 1.000 | 1.000 |
| Shield / NFZ / altitude | 0 / 0.0 s / 0.0 s | 0 / 0.0 s / 0.0 s |

**A 35-fold reduction in target jumping**, and the tracking got tighter rather
than merely calmer.

Both configurations are kept, because they answer different questions:

* `--traffic-mode demo` — distractors differ in mesh *and* paint. Readable,
  stable, and what a viewer should see.
* `--traffic-mode experiment` — same mesh throughout, colour the only free
  variable. Weaker to watch, stronger as evidence, and the one to quote when
  someone asks whether the colour word is doing any work.

Mixing meshes confounds shape with colour: a correct lock could be the paint or
could be the silhouette. That trade is real and is why both modes exist.

## 3. Different colours per car are not possible in this build

Asked for, tried three ways, measured each time.

**`set_object_material` accepts exactly one material.** Only
`/Game/Geometry/Materials/M_Orange` is ever applied. `Blue.uasset`,
`Green.uasset` and `Yellow.uasset` provably exist under
`PASBlocks/Content/Geometry/Materials/` and are still refused — so this is not a
wrong-path problem, which was the earlier theory. Only materials cooked into the
packaged build can be bound at runtime.

**`set_object_texture_from_packaged_asset` accepts everything and changes
nothing.** Four textures applied cleanly, all returning `True`, and every one
rendered identically: hue 208, S 72, V 161 — the untouched mesh. *"Returns
`True`"* and *"renders differently"* are not the same claim, and only the second
one matters.

**What would work, and needs a decision.** `spawn_object_from_file` takes a glTF
from raw bytes, and Project AirSim's own sample GLBs all carry an embedded
`baseColorTexture` — the colour travels inside the file, so no `/Game/...`
material is involved at all. A free CC0 car model in GLB would give as many
genuinely different colours as we want. It requires downloading a model, which
needs explicit permission.

Until then, variety comes from the two vehicle meshes that exist. `M_Orange`
renders **white** on `SKM_SportsCar` and genuinely **orange** on
`SM_Offroad_Body` (0.118 orange coverage measured), so there are three
distinguishable appearances: white sports car, orange buggy, blue-grey vehicle.

## Reproducing

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe demo\follow_vlm.py --object "a white car" --tag traffic --max-s 70 --det-thresh 0.008 --car-speed 2.0 --car-stop-s 6 --traffic 3 --traffic-mode demo --lock-target --policy policies\follow_car.yaml --straight --save-view
```

`--lock-target` binds the controller to one instance rather than to whichever box
scores highest this tick. It is worth having here even though the appearance
difference does most of the work.
