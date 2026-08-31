# The target has never looked like a car

**Date:** 2026-08-11
**Measured by:** `experiments/survey_materials.py` against a live simulator,
JapaneseCity `Demo_day`, FrontCamera 400x225 pitched 20 deg down.
**Raw data:** `experiments/out/materials/` and `experiments/out/materials_sportscar/`
(`survey.json`, `VERDICT.md`, annotated frames).

---

## The finding

`SM_Offroad_Body` — the mesh every follow demo has tracked — is **an open
roll-cage buggy**, not a car body. The annotated frame shows a tube chassis with
a visible driver seat and daylight through most of it.

`SKM_SportsCar` is a solid car body, and it was in the asset registry the whole
time.

Measured side by side, same pose, same camera, one phrase per forward pass:

| mesh | OWL-ViT `"a car"` | IoU with true box | what it is |
|---|---|---|---|
| `SM_Offroad_Body` + M_Orange | **0.0395** | 0.59 | open tube buggy |
| `SKM_SportsCar` (mesh default) | **0.1272** | 0.65 | solid sports car |

**3.2x the detection score.** Against the decision rule derived independently in
`docs/RESULT-detector-baseline.md` (a candidate needs >= 0.0236 for the bare noun
at 40 px), the sports car clears it by 5.4x and the buggy by 1.7x.

This also explains two things we had recorded as puzzles rather than symptoms:

* **Why detector confidence was only 0.03-0.07.** We attributed it to range — the
  car being 20-30 px wide. Range is part of it, but the mesh is the rest: a
  roll-cage is mostly holes, and OWL-ViT is trained on photographs of cars.
* **Why `colour_match` was marginal.** The survey measures 0.099 orange coverage
  at 22 m against a 0.10 gate. A tube frame puts road, not paint, inside its own
  bounding box. The colour test was fighting the mesh.

## Materials: the registry does not expose them

`list_assets('.*')` returns **2109 names, none of them materials** — the only
material-shaped hit is `MaterialSphere`, which is a mesh. Material paths cannot
be discovered from the client; they have to be known a priori.

Every guessed path was refused by the sim:

| material | result |
|---|---|
| `/Game/Geometry/Materials/M_Orange` | **applied** (the only one) |
| `M_Red`, `M_Blue`, `M_Green`, `M_Yellow`, `M_White`, `M_Black`, `Orange`, `MI_Emissive_Red` | refused |

So the earlier note that "blue rendered invisible and Yellow rendered pale grey"
was measuring the *default* mesh appearance after a silently failed material
application, not a material that renders badly. The materials were never applied
at all.

### Materials are mesh-dependent

`M_Orange` on `SM_Offroad_Body` renders orange. The same material on
`SKM_SportsCar` renders **white** — and OWL-ViT agrees, scoring `"a white car"`
0.1178 on it against 0.0002 for `"an orange car"`. A material is not a colour;
it is a shader whose result depends on the mesh's UVs and its own slots.

### Colours available today, without a single new material

| vehicle | reads as | `"a car"` | colour query |
|---|---|---|---|
| `SM_Offroad_Body` + M_Orange | orange | 0.0395 | `"an orange car"` |
| `SKM_SportsCar` default | blue-grey (hue 206, S 74) | **0.1272** | `"a blue car"` 0.0412 |
| `SKM_SportsCar` + M_Orange | white | 0.0045 | `"a white car"` **0.1178** |

Two usable colours already exist. The multi-colour city is not blocked on finding
new materials — it is blocked on the fact that the two available colours sit on
meshes with different silhouettes, which would confound shape with colour in any
discrimination experiment.

## Flown, not just probed

`vlm_sportscar`, 55 s, `follow_car.yaml`, phrase `"a white car"`, against the
previous best flight `vlm_stopgo`:

| | `vlm_stopgo` (buggy, "an orange car") | `vlm_sportscar` (sports car, "a white car") |
|---|---|---|
| detector hit rate | 0.994 | **1.000** |
| target visible | — | **100% of ticks** |
| detector rate | 3.24 Hz | **4.62 Hz** |
| mean separation | 16.2 m | **9.6 m** |
| within 30 m | 0.992 | **0.996** |
| NFZ time / altitude escape | 0.0 s / 0.0 s | 0.0 s / 0.0 s |
| Shield interventions | 0 | 0 |

Mean separation nearly halved and the detector never lost the target. The
guardrail result is unchanged, which is the point: the pilot got better and the
safety layer did not have to.

One caveat on that mean. The known `SetObjectPose ... not movable` fault fired at
about tick 420 and the self-healing respawn put the car back at its start, so the
last ~80 ticks are measuring a target that teleported away. 9.6 m includes those
ticks; the honest reading is "at least this good".

## What this changes

**Immediate:** the follow demo should switch to `SKM_SportsCar`. It looks like a
car, it detects 3.2x better, and it makes "follow the car" an honest description
of the task rather than a generous one.

**The wrong-colour control needs re-reading, carefully.** That control
(`"a blue car"` against the orange target, 20.7% vs 100%) is not invalidated —
the HSV gate did the work and still would. But part of the margin came from the
buggy being a poor car in the first place. Re-run it on the new mesh before
quoting the number again.

**A caution about the previous baseline.** `docs/RESULT-detector-baseline.md`
established its numbers from saved FPV frames, and those frames are **annotated**
— `follow_vlm.annotate()` draws a green box, a centre line, a crosshair and HUD
text before the jpg is written, so the live detector saw something the analysis
did not. That report reconstructs clean frames by inpainting and reports both
variants; its numbers stand, but any future analysis of `view/fpv/*.jpg` must
account for the overlay.

## Reproducing

The `/Game/...` argument must not pass through Git Bash. It rewrites the path to
`C:/Program Files/Git/Game/...` — visible in the survey output as a spurious
"material REFUSED" row. Run from PowerShell:

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe experiments\survey_materials.py `
  --asset SKM_SportsCar --out experiments\out\materials_sportscar
```

## Still open

* Whether `SKM_SportsCar` accepts any material other than `M_Orange`, and what
  each renders as. The material registry cannot be enumerated, so this is guess
  and test.
* `SKM_Car_Template` was found in the registry and has not been probed at all.
* `set_object_texture_from_packaged_asset` is untried and is the remaining route
  to colour variety on one silhouette, which is what a clean discrimination
  experiment needs.
