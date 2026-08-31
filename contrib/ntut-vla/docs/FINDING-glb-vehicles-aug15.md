# Different-coloured vehicles, and three measurements that were not real

**Date:** 2026-08-15
**Trigger:** "buat warna object yang ada tersebut berbeda, dan jika ada asset
object mobil lain, gunakan object tersebut" — and, before that, six weeks of this
build refusing to bind any material except `M_Orange`.

The request is now satisfied: the street carries a yellow taxi, a white police
car, a red sedan and a blue van, and the colour arrives without any `/Game/...`
material being involved. What took the longest was not making it work. It was
noticing that the first three times I measured it, I was measuring nothing.

---

## 1. What was actually blocking colour

Three routes were tried before this one and all three are recorded in
`FINDING-vehicle-mesh-and-materials.md`:

* `set_object_material` binds exactly one material. `Blue.uasset`, `Green.uasset`
  and `Yellow.uasset` provably exist on disk and are still refused, so this is
  not a wrong-path problem. Only materials cooked into the packaged build load.
* `set_object_texture_from_packaged_asset` accepts everything and changes
  nothing — four textures applied cleanly, all returning `True`, every one
  rendering hue 208 / S 72 / V 161, the untouched mesh.
* Which left `spawn_object_from_file`, where the colour travels **inside** the
  mesh file and the sim never has to resolve a material at all.

Kenney's Car Kit (CC0) has 20 vehicles sharing one 512×512 atlas — measured
21.5% orange, 17.8% red, 15.9% black, 12.6% grey, 12.5% white, 8.8% blue, 6.2%
green, 5.7% purple, 3.2% yellow. Each model's UVs pick a different region, so one
texture yields a fleet of different colours.

One detail decides it. Kenney's GLBs reference the atlas **externally**:

```json
"images": [{"uri": "Textures/colormap.png", "name": "colormap"}]
```

`spawn_object_from_file` takes one byte array and has nowhere to resolve that
from. Project AirSim's own sample GLBs all carry the image as an embedded
`bufferView` instead. `tools/embed_glb_textures.py` rewrites the former into the
latter: 50 files, 0 already embedded, verified by re-reading every output.

## 2. Three measurements that were not measurements

Worth recording in full, because each produced a plausible number and each was
wrong in a different way. A plausible number is more dangerous than an error.

**Run 1 — the drone never took off.** Every model scored ~0.006. I read the
frame and it was bare asphalt: the camera sits at ground level pitched 20° down
and photographs the road in front of its own nose. Nothing was ever in shot.

**Run 2 — the subject was 90° out of frame.** After adding the takeoff, scores
rose to ~0.085 and looked like a real result. The frames were still bare street.
`x` is North and `y` is East, so a subject at the same `x` and +22 `y` sits due
**East** while a yaw-0 drone stares North — 90° outside a 90°-wide frame. The
0.085 was OWL-ViT finding the city's **own parked cars**.

That is the one that would have shipped. It had the shape of a finding: ten
models, a spread of scores, a clear winner, one that "beat the baseline". The
only thing wrong with it was that the subject was never present.

**What caught it** was a positive control, and nothing else would have. A
`1M_Cube` spawned through the ordinary packaged path at the same pose was
**equally invisible**. A known-good object failing identically says the fault is
in the harness, not the thing under test — and no amount of staring at candidate
scores would have said that.

**Run 3, valid.** Nose aimed at the subject, plus a **background floor**: the
empty frame is scored too, because this street has its own parked cars and
"detected the candidate" and "found the background" are otherwise the same
number.

## 3. The numbers

Background floor, empty frame, `a car`: **0.0303**.

| model | `a car` | over background | coloured query | colour gate |
|---|---|---|---|---|
| police.glb | **0.1639** | +0.1336 | — | — |
| van.glb | 0.1580 | +0.1276 | — | — |
| taxi.glb | 0.1500 | +0.1197 | `a yellow car` **0.2154** | 0.20 |
| garbage-truck.glb | 0.1026 | +0.0723 | — | — |
| sedan.glb | 0.0696 | +0.0393 | — | — |
| ambulance.glb | 0.0654 | +0.0351 | `a white car` 0.0729 | 0.23 |
| **firetruck.glb** | 0.0336 | **+0.0033** | `a red truck` 0.0271 | 0.14 |

Prior baselines: `SM_Offroad_Body` 0.0395, `SKM_SportsCar` 0.1272.

Firetruck is reported because it is the honest failure: +0.0033 over background
means it was **not detected at all**, and a table listing only the winners would
imply the models are uniformly good. They are not.

### The fair fight

The baselines above came from a different script on a different day. So the
incumbent was re-measured **at the same pose, in the same run**:

| subject | query | score | colour gate |
|---|---|---|---|
| `SM_Offroad_Body` + `M_Orange` | `an orange car` | 0.047 | 0.119 |
| `taxi.glb` | `a yellow car` | **0.108** | **0.317** |
| `police.glb` | `a white car` | 0.095 | 0.255 |
| `sedan.glb` | `a red car` | 0.097 | 0.237 |

Better on both halves of the test, and — the actual point — **four colours where
there was one**.

## 4. Fitness for the job, which is harder than standing still

Detecting well is not enough; `city_traffic.py` drives the fleet by
`set_object_pose` at 10 Hz.

| test | result |
|---|---|
| movable | 20/20 teleports, max readback error **0.0 m**, 0 failures |
| four at once | all four spawn and coexist |

The movability test was not optional. This build has already produced one
`SetObjectPose ... not movable` fault (`FINDING-setobjectpose-not-a-sim-bug.md`),
and a vehicle that spawns beautifully then refuses to move would have been
discovered mid-flight instead of in a 40-second probe.

## 5. The orientation, measured rather than reasoned

glTF is Y-up / −Z-forward; the sim is Z-up NED with heading 0 = +x/North. The
offset between them is silent: the pose call succeeds either way and the whole
fleet simply drives broadside.

Measured by sweeping yaw and taking the widest detection box — a car shows its
full flank when side-on:

| yaw | 0 | 45 | 90 | 135 | 180 | 225 | 270 | 315 |
|---|---|---|---|---|---|---|---|---|
| aspect | 0.93 | 1.12 | 1.45 | 1.47 | 0.93 | 1.10 | **1.61** | 1.58 |

Flank at 90 and 270, narrow at 0 and 180, exactly as a car should behave. Front
from back was settled by enlarging both flank views: the bonnet points image-left
at 270, and image-left is North (fixed independently by the fleet frame, where
the taxi at x=32 is nearest-right and the van at x=41 is farthest-left).

**`quaternion_yaw = heading + 270°`**, reproducible via
`experiments/probe_glb_yaw.py`.

## 6. What changed in the repo

* `moving_car.py` — `CarSpec.glb_path` / `glb_scale` / `glb_yaw_offset_deg`;
  `spawn()` branches to `spawn_object_from_file`; no material is applied to a
  model, and `material_used=None` is no longer treated as a paint failure.
* `city_traffic.py` — `glb_dir()`, `glb_fleet()`, `GLB_TARGET`, `GLB_PALETTE`.
* `follow_vlm.py` — `--glb-dir`; the single-car path uses a model too.
* `run_follow_vlm.ps1` — default object is now `a yellow car`.

**Discovery is the caller's job.** `default_fleet()` does not go looking for
models: a fleet that silently changes shape depending on what is installed on
the machine is unreproducible, and a result measured on one machine would not be
a result. `follow_vlm.py` resolves the directory and passes it in; the tests pass
a synthetic one.

**`--traffic-mode experiment` deliberately refuses the models.** It exists to
hold the mesh constant so colour is the only free variable, and five different
silhouettes would destroy the one property it is for. The demo mode mixes meshes
and therefore confounds shape with colour — that trade is unchanged, and
experiment mode is still the arm to quote when the question is whether the colour
word does the work.

## 7. A guard against the mistake that started this

The regression the user reported — "detection worse than before the meeting" —
was mine: I switched the target to `SKM_SportsCar` for its better noun score,
`M_Orange` renders **white** on that mesh, and the query still said "orange".
Nothing failed and nothing warned.

`follow_vlm.py` now compares the colour word in `--object` against the colour the
spawned subject actually claims, and says so loudly when they disagree. The class
of bug is silent by nature; the check is not.

## Flown, and the result is mixed

Three flights, in `RESULT-glb-fleet-flights-aug15.md`. Summarised:

* traffic detector hit rate **0.50 -> 0.740**, and the false-ABSENT rate
  **0.30-0.44 -> 0.034**, against the orange buggy that was actually shipping;
* single-car hit rate went the other way, **0.994 -> 0.786** — the buggy is
  easier to detect in isolation and collapses to 0.50 in traffic, the taxi
  holds 0.74-0.79 in both;
* following quality unchanged to slightly better (within 30 m 1.000, mean
  separation 15.2 m against 16.2 m);
* guardrail untouched: 0 interventions, NFZ 0.0 s, altitude 0.0 s on all three.

## Installing the models (not in this repo)

They are a 4.8 MB third-party pack and must never be committed.

```bash
python tools/embed_glb_textures.py --zip <kenney_car-kit_3.1.zip> --out D:/models/kenney_car-kit/glb
```

Source: https://opengameart.org/content/car-kit — Kenney, CC0. Then set
`VLA_GLB_DIR` or pass `--glb-dir`. Without them everything still runs, on the
packaged meshes, with one colour, and says so once at startup.

## Reproducing

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe experiments\probe_glb_vehicles.py
C:\Users\natha\.conda\envs\vla-real\python.exe experiments\probe_glb_traffic_fitness.py
C:\Users\natha\.conda\envs\vla-real\python.exe experiments\probe_glb_yaw.py
```
