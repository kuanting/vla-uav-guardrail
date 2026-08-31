# OWL-ViT detection baseline for the current car mesh

Measured 2026-08-11 by `experiments/probe_detector_realism.py`. No simulator involved: recorded flight frames only.

## Why

A downloaded glTF car can be spawned from raw bytes (`spawn_object_from_file`, format `gltf`), which makes swapping the mesh cheap. What is not cheap is discovering after the swap that OWL-ViT no longer grounds the word on it. The current mesh is `SM_Offroad_Body` + `/Game/Geometry/Materials/M_Orange` - the only material verified to render correctly - so it is the reference. This is how detectable it actually is.

## Method

* Model `google/owlvit-base-patch32`, CUDA, `local_files_only` (no download).
* 120 frames, evenly spread across two flights, 1200 forward passes.
* **One phrase per forward pass** (`text=[[phrase]]`), as the live `Grounder` does. Batching phrases changes the scores.
* Selection replicates `Grounder._worker`: top 12 by score, stop below `--det-thresh 0.008`, drop a box whose named colour covers < 10% of it, rank survivors by `score * (0.25 + 0.75 * colour_fraction)`. The live temporal jump filter is NOT replicated (it needs the previous accepted box), so these numbers are marginally more permissive than flight.

### Two data problems that had to be fixed first

**The saved FPV frames are annotated.** `follow_vlm.annotate()` draws a pure-green box, a full-height green centre line, a white crosshair and five lines of HUD text, and *that* is what is written to `view/fpv/*.jpg`. The live detector ran on the raw camera image. Every frame here is therefore reconstructed - overlay colours masked, Telea inpaint - and both variants are scored so the size of the contamination is on the record.

**`vlm_stopgo`**: 337 jpgs in `view/fpv`, in write sessions of [160, 1, 13, 163] frames. Only the last session was written by the run that produced `metrics.json`; 0 of its frames had no flight-log row. **163 usable.**
**`vlm_nfz_smooth`**: 184 jpgs in `view/fpv`, in write sessions of [184] frames. Only the last session was written by the run that produced `metrics.json`; 0 of its frames had no flight-log row. **184 usable.**

## Result 1 - per query, clean frames

`hit` = fraction of frames where the live selection procedure returned a box. `score` is the raw detector score of the selected box.

| query | hit | score min | p10 | median | p90 | max | box width px median | top-1 raw median |
|---|---|---|---|---|---|---|---|---|
| `an orange car` | 0.333 | 0.00859 | 0.00991 | **0.02181** | 0.03599 | 0.04217 | 29.62284 | 0.0152 |
| `a car` | 1.000 | 0.01362 | 0.02028 | **0.03822** | 0.08809 | 0.2048 | 21.30238 | 0.03822 |
| `a vehicle` | 1.000 | 0.03095 | 0.03952 | **0.10204** | 0.28776 | 0.40163 | 21.67738 | 0.10204 |
| `a truck` | 1.000 | 0.00822 | 0.01044 | **0.015** | 0.05582 | 0.21389 | 81.27651 | 0.015 |
| `a blue car` | 1.000 | 0.00956 | 0.01467 | **0.03207** | 0.06451 | 0.14951 | 19.13244 | 0.03583 |

### The annotation contamination, quantified

| query | median score, clean | median score, annotated | inflation |
|---|---|---|---|
| `an orange car` | 0.02181 | 0.02435 | x1.12 |
| `a car` | 0.03822 | 0.02403 | x0.63 |
| `a vehicle` | 0.10204 | 0.05769 | x0.57 |
| `a truck` | 0.015 | 0.01242 | x0.83 |
| `a blue car` | 0.03207 | 0.01769 | x0.55 |

## Result 2 - score against apparent width (the range proxy)

`a car` - Spearman(width, score) = **0.42** over 120 boxes

| box width px | n | score median | p10 | min |
|---|---|---|---|---|
| 0-15 | 32 | 0.03099 | 0.02043 | 0.01362 |
| 15-25 | 45 | 0.03354 | 0.01984 | 0.01398 |
| 25-40 | 29 | 0.05134 | 0.02284 | 0.01569 |
| 40-60 | 13 | 0.09203 | 0.06083 | 0.03267 |
| 60-400 | 1 | 0.04319 | 0.04319 | 0.04319 |

`an orange car` - Spearman(width, score) = **0.412** over 40 boxes

| box width px | n | score median | p10 | min |
|---|---|---|---|---|
| 0-15 | 0 | None | None | None |
| 15-25 | 14 | 0.0167 | 0.00911 | 0.00859 |
| 25-40 | 16 | 0.02705 | 0.01269 | 0.00868 |
| 40-60 | 10 | 0.02619 | 0.01395 | 0.0128 |
| 60-400 | 0 | None | None | None |

### and against true range, from the flight log

| separation m | n frames | n with a box | score median | width median px |
|---|---|---|---|---|
| 0-10 | 6 | 5 | 0.03379 | 41.9 |
| 10-15 | 21 | 16 | 0.02196 | 34.1 |
| 15-20 | 28 | 15 | 0.0215 | 26.1 |
| 20-30 | 21 | 4 | 0.01348 | 18.5 |
| 30-45 | 7 | 0 | None | None |
| 45-200 | 37 | 0 | None | None |

## Result 3 - do these numbers agree with the live flight?

`detections.jsonl` holds the score the live detector assigned. The detector ran asynchronously at 3-4 Hz, so the record current at a frame was computed on a frame grabbed up to a third of a second earlier; the tight rows below restrict the comparison to frames where that staleness was under 0.15 s.

| subset | variant | n | live median | offline median | median rel. err | median abs rel. err | Spearman | box centre dist px | within 25 px |
|---|---|---|---|---|---|---|---|---|---|
| all | clean | 40 | 0.0421 | 0.02181 | -42.5% | 44.8% | 0.207 | 4.4 | 0.975 |
| all | annotated | 15 | 0.053 | 0.02435 | -46.8% | 46.8% | 0.207 | 1.8 | 0.933 |
| det_age<=0.15s | clean | 21 | 0.0455 | 0.02439 | -40.4% | 40.4% | 0.078 | 3.2 | 1.000 |
| det_age<=0.15s | annotated | 10 | 0.05685 | 0.03047 | -50.0% | 50.0% | -0.164 | 1.7 | 1.000 |

## Result 4 - THE DECISION RULE

The servo law holds `want_width = 0.1` of a 400 px frame, so **40 px is the apparent width the aircraft spends the flight trying to maintain**. That is the width any threshold has to be quoted at; a score measured on a 90 px box says nothing about whether the lock survives at standoff.

Measured on the current mesh in the 30-50 px band:

| query | n | score median | p10 | min | median / 0.008 | p10 / 0.008 |
|---|---|---|---|---|---|---|
| `a car` | 25 | 0.06954 | 0.03928 | 0.01903 | x8.69 | x4.91 |
| `an orange car` | 19 | 0.02798 | 0.01382 | 0.00968 | x3.5 | x1.73 |

> **Rule.** A candidate mesh is acceptable if OWL-ViT scores it at least **0.0236** for the bare noun phrase **`a car`** on a box of apparent width **40 px** (30-50 px band) in a 400x225 frame from the FrontCamera geometry, on at least 90% of sampled frames.

Where the number comes from, and why not simply 0.008:

* `--det-thresh 0.008` is the *floor the code will accept*, not a quality bar. At 40 px the current mesh sits at x8.69 that floor at the median and x4.91 at the 10th percentile. A candidate that merely clears 0.008 would be running with essentially no margin, and the p10 is what decides whether the lock survives the bad frames rather than the good ones.
* The rule takes 60% of the current mesh's p10 (0.03928), floored at 2x the flight threshold. 60% is a deliberate concession: the point of a new mesh is a car that looks more like a real car, and we would accept somewhat worse detectability for that - but not a mesh that has to be carried by the threshold.
* Quote `a car`, not `an orange car`, because OWL-ViT does not discriminate colour (measured: `a car` and `a blue car` return an identical box on a real frame). The colour word's contribution is the HSV check, which is a separate and independent test of the candidate's material. **A candidate that passes this rule can still fail the flight** if its material does not give colour_match >= 10%; check both.

### How to test a candidate

Render or photograph the candidate at the geometry the FrontCamera gives - 400x225, 90 deg horizontal FOV, pitched 20 deg down - framed so the car's box is about 40 px wide, then run exactly the query path this script uses: one phrase per forward pass, `post_process_object_detection(threshold=0.0)`, top 12, `score * (0.25 + 0.75 * colour)`. Do not batch the phrases and do not score an annotated frame.

## Caveats

* The inpaint is imperfect and the error has a direction: the green box is drawn ON the car's silhouette, so removing it blurs the car's own edge. The clean-frame scores are a floor, not a point estimate. Result 3 is what bounds the error.
* The live temporal jump filter is not replicated, so offline hit rates for the loose queries (`a car`, `a vehicle`, `a truck`) overstate what flight would accept - in flight a box far from the last one is rejected.
* Both flights are the same map, the same time of day, one car, one material. This is a baseline for *that* mesh under *those* conditions, which is all the swap decision needs.

Raw per-frame records: `experiments/out/detector_baseline/samples.jsonl`, aggregates `summary.json`.
