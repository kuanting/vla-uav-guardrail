> **SUPERSEDED.** See `TUTORIAL.md` in the project root, which is the single verified guide. This file is kept as history only and its commands, tags and numbers may be out of date.

# Running the "follow the car" experiment

> **Start here if you just want the drone to follow the car.**
>
> ```powershell
> .\run_follow_vlm.ps1
> ```
>
> That flies `demo/follow_vlm.py`, which **works**: measured closest approach
> 0.4 m, 75.7% of the flight within 30 m of a car moving at 3 m/s, NFZ 0.0 s,
> altitude escape 0.0 s. It uses an open-vocabulary detector for the language
> grounding and a servo loop for control, with the same guardrail underneath.
> Written up in [docs/RESULT-vlm-follow.md](docs/RESULT-vlm-follow.md).
>
> The rest of this file covers the **AerialVLA** version, which does *not*
> follow — 12 of 12 inferences return stop-and-land. It is kept because the
> negative result is the reason the working version looks the way it does, and
> because reproducing it is how you check the finding rather than taking it on
> trust.

---


Step by step, from a cold machine. Every command here was run to produce the
results in [docs/FINDING-what-drives-aerialvla.md](docs/FINDING-what-drives-aerialvla.md).

> **Environment for everything below:** conda env `vla-real`
> (`C:\Users\natha\.conda\envs\vla-real\python.exe`), run from the project root
> `D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone`.

---

## What you will see, before you start

So the output is not a surprise:

* **Without a direction hint the drone does not move.** All 12 inferences return
  the same token triple `[0, 49, 49]` — stop and land. This is the headline
  result, not a bug in the setup.
* **With the hint it moves but does not follow.** It ends further from the car
  than a drone that never took off.

If your run shows something different, that is genuinely new and worth writing
down. If it shows the same, you have reproduced the finding.

---

## 0. Start the simulator

**Use PowerShell, not Git Bash.** Git Bash rewrites the `/Game/...` map argument
into a Windows path (`C:/Program Files/Git/Game/...`), the map is not found, and
the engine crashes on the fallback. This costs ten minutes to diagnose every
time.

```powershell
Start-Process "C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe" -ArgumentList '"D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone\PASBlocks\Blocks.uproject"','/Game/JapaneseCity/Maps/Demo_day','-game','-windowed','-ResX=1280','-ResY=720'
```

Wait until port 8989 answers — usually 20–60 s, longer on the first load of a map:

```powershell
while (-not (Test-NetConnection 127.0.0.1 -Port 8989 -WarningAction SilentlyContinue).TcpTestSucceeded) { Start-Sleep 5 }; "sim ready"
```

**If you have another Unreal project open**, leave it alone — the batch runner
now kills only processes whose command line contains `Blocks.uproject`. Older
scripts in this repo (`training/consistency_check.py`) still kill *every*
`UnrealEditor`, which will take unsaved work with it.

---

## 1. Look at the car first

Never trust a number that depends on an object you have not seen.

```powershell
python experiments\probe_moving_car.py
```

This spawns the car, checks that teleporting actually moves it (it reports the
pose read back from the sim), then photographs it from 9 m altitude at 15, 22 and
30 m range. Then **open the images**:

```
experiments\out\car_probe\rng_22_front.png
experiments\out\car_probe\zoom_22.png
```

You should see an orange **car** on a road with lane markings — a vehicle
silhouette, not a block — about 29 px wide at 22 m. If it is grey or invisible,
the material did not apply and the colour word in the instruction no longer
matches the pixels; fix that before flying.

The target is `SM_Offroad_Body` from `PASBlocks/Plugins/Rover`, a real car body
mesh that ships with Project AirSim. The sim reports it at 3.70 × 1.79 × 1.16 m.
`world.list_assets(".*Car.*")` also lists `SKM_SportsCar` and `SKM_Car_Template`
if you want to try others — note those are *skeletal* meshes and spawn with a
zero bounding box, so they need different handling.

**Expected console output:**

```
[car] spawned 'SemCar' 3.7x1.8x1.2 m at (38.0, -8.0)
[car] material -> /Game/Geometry/Materials/M_Orange
[probe] sim reports the car at (38.0, 7.0) -> teleport CONFIRMED
[probe] range  22.0 m  car ~29.2 px wide  depression 22.2 deg (front cam half-FOV 29.2)
```

A `teleport failed ... check if object state is movable` line for the *first*
update is normal — the object is not movable for a moment after spawning. Any
later failure is not normal.

---

## 2. Fly the mission as asked: camera and language only

```powershell
python demo\semantic_seek.py --follow-car --object "orange car" --adapter D:/models/aerialvla-lora/aero_vla --hint-mode none --policy policies\follow_car.yaml --cruise-alt 9 --start-beta-deg 0 --max-s 120 --tag follow_none
```

Takes about 5 minutes: ~2 min to load the 7B model at 4-bit, then 120 s of
flight. Watch for:

```
[semantic] direction hint: NONE (pure semantic)
  tick 250: pos=(  35.0, -20.0, 8.7) psi=+130.7 fwd=0.00 yaw=+0.00 shield=-
[report] ticks 1112 | inferences 12 (0.11 Hz) | ... | NFZ 0.0s entered=False
```

`fwd=0.00` on every tick is the result. Confirm it came from the model rather
than from a plumbing fault:

```powershell
python -c "import json; rows=[json.loads(l) for l in open('demo/out/follow_none/inference.jsonl',encoding='utf-8')]; print(repr(rows[0]['prompt'])); print([r['bins'] for r in rows])"
```

Expect the prompt `'<image>\nFly and find the target. orange car\nAction: '` with
no direction phrase, and `[0, 49, 49]` twelve times.

---

## 3. Fly the same mission with the hint the model was trained on

```powershell
python demo\semantic_seek.py --follow-car --object "orange car" --adapter D:/models/aerialvla-lora/aero_vla --hint-mode truth --policy policies\follow_car.yaml --cruise-alt 9 --start-beta-deg 0 --max-s 120 --tag follow_truth
```

`--hint-mode truth` adds the seven-way bearing phrase computed from the car's
real position, updated as the car moves. The drone will fly. It will not follow.

---

## 4. Score both

```powershell
python experiments\analyze_semantic_ab.py --conditions experiments\conditions_follow_car.yaml
```

Reads the pre-registered thresholds from the conditions file, writes
`experiments\out\comparison_table.md` / `.csv` and four figures. The numbers that
matter for a moving target:

| column | means |
|---|---|
| `sep_mean_m` | average distance to the car — **the following metric** |
| `frac_within_30m` | fraction of the flight spent within 30 m |
| `sep_end_m` | distance at the end |
| `sep_min_m` | closest approach — **misleading on its own**, see below |
| `n_both` | ticks where VLA and Shield were both demonstrably acting |
| `viol_any_s` | seconds outside any P0 rule |

Reproduced result, against the real car mesh:

| flight | hint | sep_min | sep_mean | sep_end | within 30 m |
|---|---|---|---|---|---|
| `follow_none` | none | 12.4 m | 38.3 m | 47.8 m | 34.5% |
| `follow_truth` | truth | 7.8 m | 50.4 m | 30.7 m | 18% |

The same mission against a car-sized *box* gave 38.2 m / 48.0 m / 35% with no
hint — identical. Replacing the box with a real vehicle mesh changed the no-hint
result by nothing at all, which is what rules out "the model could not tell it
was a car".

**Do not read `sep_min` as success.** The stationary drone scores 12.4 m because
the car drove past it. That is why the pass criterion in
`experiments/conditions_follow_car.yaml` is mean separation and time-within-range,
not closest approach.

---

## 5. Optional: the whole 4-cell matrix instead of two flights by hand

Adds a wrong-colour control and our fine-tune. About 25 minutes, one sim restart
per flight, resumable.

```powershell
python experiments\semantic_ab.py --conditions experiments\conditions_follow_car.yaml --dry-run
```

Always dry-run first and read the commands. Then:

```powershell
python experiments\semantic_ab.py --conditions experiments\conditions_follow_car.yaml
```

Results append to `experiments\out\results.jsonl` after every flight, so a crash
does not cost the earlier ones. Re-running skips cells that already have a
`metrics.json` unless you pass `--force`.

---

## 6. The cheaper experiment: why it fails, without flying

The flight tells you *that* it does not work. This tells you *why*, with better
control and no flight-to-flight variance — 108 forward passes over every
combination of (camera frame × direction hint × object phrase).

```powershell
python experiments\capture_frames.py
python experiments\ablate_image_vs_hint.py --label orig
python experiments\ablate_image_vs_hint.py --adapter D:/models/aerialvla-ft/run2/epoch1 --label ft
```

`capture_frames.py` needs the sim; the two ablation runs do not. Each takes about
8 minutes. Expect:

```
Mean commanded yaw per hint:
  (none)           yaw=+0.182  fwd=0.56  LAND 11/18
  to your left     yaw=-0.466  fwd=4.24  LAND 0/18
  to your right    yaw=+0.161  fwd=4.11  LAND 0/18

Same pixels (target present), correct vs wrong colour word:
  orange barrier   yaw=+0.050 fwd=3.90 LAND 1/18
  blue barrier     yaw=+0.076 fwd=3.53 LAND 1/18
```

The direction phrase orders the yaw command monotonically; the object phrase does
nothing. That is the whole finding in two tables.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Engine window opens then dies immediately | you launched from Git Bash — the `/Game/...` argument was rewritten. Use PowerShell (§0). |
| `pynng ConnectionReset` mid-flight | transient sim hiccup; restart the sim and re-run that flight |
| Car invisible in the probe images | material did not apply. `Yellow` and `MI_Emissive_Red` both render pale — only `M_Orange` is verified. Check `[car] material ->` in the log. |
| Car never moves | a `SetObjectPose ... not movable` error on *every* update, not just the first. Increase the settle delay after spawn in `semantic_seek.py`. |
| Drone sinks below the altitude band | known open defect. `semantic_seek.py` has no altitude-hold term and the Shield is a constraint filter, not a controller — it makes minimal corrections (~+0.5 m/s), which forward flight outpaces. Tracked as a task. |
| Inference at 0.1 Hz instead of 1–3 Hz | expected with the sim rendering on the same GPU. Lowering the render resolution did **not** help, so it is not the main render. Unresolved. |
| `numpy` errors after any `pip install` | re-pin: `pip install --force-reinstall numpy==1.26.4` |

---

## The working version: `demo/follow_vlm.py`

One click, main flight plus both controls (~25 min):

```powershell
.\run_follow_vlm.ps1
```

Main flight only:

```powershell
.\run_follow_vlm.ps1 -MainOnly
```

By hand:

```powershell
python demo\follow_vlm.py --object "an orange car" --tag vlmfollow --max-s 120 --det-thresh 0.008
```

Useful flags:

| flag | meaning |
|---|---|
| `--object` | what to follow, in words. **Be specific** — "a car" makes it chase city clutter and end 81 m away; "an orange car" ends 4 m away. |
| `--no-car` | control condition: identical mission with nothing to follow |
| `--want-width` | target apparent width as a fraction of the frame; sets standoff distance |
| `--det-thresh` | detection score floor. True scores are 0.03–0.07 for a small distant car, so this is low by necessity. |
| `--yaw-gain`, `--speed-max`, `--alt-gain` | servo tuning |

Expected, from two runs:

```
[report] ticks 1042 | detector 4.74 Hz, hit rate 0.613 | target visible on 100% of ticks
[report] separation: min 0.4 m, mean 20.7 m, end 12.2 m, within 30 m 0.757
[report] guardrail: NFZ 0.0s, altitude escape 0.0s, interventions 0
```

The first run of the model downloads `google/owlvit-base-patch32` (~600 MB,
Apache-2.0) into the HuggingFace cache; after that it is offline.

Honest limits, before you quote any of this: the colour adjective contributes
almost nothing ("a blue car" scores the same as "an orange car" on the same
orange car), the detector reports a hit on 75% of frames even with no car in the
scene, and n = 2. It follows; it does not discriminate.

## What would be needed to make AerialVLA itself work

None of this is tuning. The obstruction is structural:

1. ~~A real car mesh.~~ **Done.** `SM_Offroad_Body` from `PASBlocks/Plugins/Rover`
   is a genuine vehicle mesh and is now the target. It changed nothing: the
   no-hint flight produced the identical null. So this was never the blocker.
2. **A model that emits actions from pixels**, not from a seven-way bearing
   phrase. This is the real obstruction — the measured evidence is that
   AerialVLA's object-description slot does not steer at all.
3. **Inference at 5 Hz or better.** A ±22° bearing phrase refreshed every ~8 s
   cannot track a 3 m/s target regardless of how good the model is. Currently
   0.11–0.14 Hz; lowering the sim render resolution did not help, so the
   bottleneck is elsewhere and remains undiagnosed.

Item 3 is the cheapest to attack and would improve every mission in the repo, not
just this one. Item 2 means a different checkpoint or a different model family,
not more fine-tuning of this one — our own fine-tune moved the object slot
*further* toward inert (yaw response 0.454 → 0.321).
