# How to run this

**This file supersedes every other TUTORIAL-*.md in the repo.** Those are kept as
history; if they disagree with this one, this one is right.

Last verified end to end: 2026-08-14.

---

## 0. The one command

Open **PowerShell** — not Git Bash, see §6 — in the project folder and run:

```powershell
.\run_follow_vlm.ps1
```

That is the whole demo. About 18 minutes. It starts the simulator, flies three
missions, restarts the simulator between each, builds three side-by-side videos,
and prints a summary table.

You do not need to do anything else. The rest of this file is for when you want
to run one piece, change something, or explain what you are watching.

---

## 1. What you will see

Three flights, each with the same spoken instruction — **"a white car"** — and
nothing else steering the aircraft.

| | what happens | what to point at |
|---|---|---|
| **DEMO 1** | The car drives, stops for 6 s, drives, stops, drives again. The drone follows and **stops when the car stops**. | A follower has to stop. That is what separates following from flying down the same street. |
| **DEMO 2** | Three more cars appear, **the same model, only the colour differs**. The drone still picks the right one. | The noun cannot separate them — every one of them is "a car". Only the colour test can. |
| **DEMO 3** | A no-fly zone spans the whole road. The car drives through it. The drone **stops at the edge and does not**. | Losing the car here is the *correct* outcome. Entering the zone would not be. |

Measured on those three, re-flown 2026-08-11:

| | detector hit | within 30 m | mean sep | Shield | NFZ / altitude |
|---|---|---|---|---|---|
| DEMO 1 tracking | 99.3% | **100%** | 13.4 m | 0 | **0.0 s / 0.0 s** |
| DEMO 2 + 3 distractors | 97.9% | **100%** | 17.7 m | 0 | **0.0 s / 0.0 s** |
| DEMO 3 no-fly zone | 100% | 34.9% | 38.8 m | 0 | **0.0 s / 0.0 s** |

### The slide that makes it a result

```powershell
.\run_follow_vlm.ps1 -Controls
```

Two extra flights. The important one changes **one word** — "a white car" to
**"a red car"** — with the same white car in the same scene. The drone does not
follow badly; it **fails to acquire at all**: detector hit rate collapses from
99.3% to **22.1%**, and time within 30 m from 100% to **24.1%**.

Without that control, "the drone ended up near the car" could just be drift.

---

## 2. Running one flight by hand

Everything the script does is one Python command. This is DEMO 1:

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe demo\follow_vlm.py --object "a yellow car" --tag my_run --max-s 70 --det-thresh 0.008 --car-speed 2.0 --car-stop-s 6 --policy policies\follow_car.yaml --straight
```

The simulator must already be running. Start it with:

```powershell
& "C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe" "PASBlocks\Blocks.uproject" /Game/JapaneseCity/Maps/Demo_day -game -windowed -ResX=1280 -ResY=720
```

Wait until port 8989 answers — about 25 seconds. Then fly.

### The arguments worth knowing

| flag | what it does |
|---|---|
| `--object` | **what to follow, in words.** This is the whole interface. |
| `--traffic N` | add N distractor cars. With the glTF models installed each is a different colour; without them they fall back to two meshes |
| `--glb-dir` | where the glTF vehicles live. Defaults to `$VLA_GLB_DIR` then `D:/models/kenney_car-kit/glb`. Not in the repo - see `docs/FINDING-glb-vehicles-aug15.md` to install |
| `--policy` | which guardrail rules apply |
| `--max-s` | how long to fly |
| `--save-view` | record frames for the video |
| `--no-car` | fly with no target at all (a control) |

### Change what it follows

The vocabulary is open — the detector was never trained on a fixed class list:

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe demo\follow_vlm.py --object "a tree" --tag tree_test --max-s 40 --no-car --policy policies\follow_car.yaml
```

`"a tree"`, `"a building"`, `"a road"`, `"a traffic light"`, `"a lamp post"` all
work. **The colour half is narrower**: nine words only — red, orange, yellow,
green, blue, purple, white, black, grey. Anything else passes unchecked.

Avoid `"a blue car"`. The asphalt in this map reads blue, so the query has a
background-coloured escape hatch and the control looks weaker than it is.

---

## 3. Checking it without the simulator

135 tests, no sim needed, about 4 minutes:

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe tests\test_shield.py; C:\Users\natha\.conda\envs\vla-real\python.exe tests\test_clearance.py; C:\Users\natha\.conda\envs\vla-real\python.exe tests\test_guardrail_coverage.py; C:\Users\natha\.conda\envs\vla-real\python.exe tests\test_city_traffic.py; C:\Users\natha\.conda\envs\vla-real\python.exe tests\test_fenceguard.py; C:\Users\natha\.conda\envs\vla-real\python.exe tests\test_vla_bridge.py; C:\Users\natha\.conda\envs\vla-real\python.exe tests\test_range_and_lock.py
```

Every one should end `N/N passed`. `test_guardrail_coverage.py` is the important
one — it fires 20 000 random state/action pairs at the safety layer and checks
that nothing it emits ever violates a P0 rule.

---

## 4. Reading the output

Each flight writes to `demo\out\<tag>\`:

| file | what it holds |
|---|---|
| `metrics.json` | the headline numbers — start here |
| `flight_log.jsonl` | one record per 10 Hz tick: position, detection, the action before and after the Shield |
| `detections.jsonl` | one record per detector inference |
| `<tag>_demo.mp4` | the side-by-side video, if `--save-view` was used |

The three numbers that matter in `metrics.json`:

- **`frac_within_30m`** — how much of the flight it stayed with the target
- **`nfz_s`** and **`alt_violation_s`** — the safety KPI. **Must be 0.0.**
- **`interventions`** — how often the Shield had to correct the pilot

---

## 5. The other missions

```powershell
# Orbit a parked car — it circles more than a full lap
C:\Users\natha\.conda\envs\vla-real\python.exe demo\follow_vlm.py --object "a yellow car" --tag orbit --max-s 100 --cruise-alt 18 --park-at "40,-40" --orbit-speed 2.5 --orbit-radius 16 --lock-target --policy policies\orbit_building.yaml
```

```powershell
# No-fly zone with a 7 m gap left open, so the rule costs the path not the target
C:\Users\natha\.conda\envs\vla-real\python.exe demo\follow_vlm.py --object "a yellow car" --tag gap --max-s 70 --cruise-alt 13 --want-width 0.20 --car-speed 2.0 --policy policies\follow_car_gap.yaml --straight
```

**`--want-width` is tied to `--cruise-alt`.** It sets an *angular* stand-off, so
the same value is a much larger ground distance from higher up. 0.10 suits a 9 m
cruise; the gap policy flies at 13 m and needs about 0.20. Getting this wrong
cost two wrong diagnoses before it was noticed.

---

## 6. When it goes wrong

| what you see | what to do |
|---|---|
| `map not found`, engine crashes on start | **You used Git Bash.** It rewrites `/Game/...` into `C:/Program Files/Git/Game/...`. Use PowerShell. |
| `ticks 0`, or the flight connects and dies | The simulator refuses a second connection after a flight disconnects. Restart it. The script already does. |
| Drone sweeps at the start | That is SEARCHING. It has not seen the target yet and will not sit still waiting. |
| Drone stops moving | The car stopped. Watch it start again when the car does — that is DEMO 1's point. |
| It locks onto something wrong | Limitation 1. A general noun catches city clutter; watch the colour score in the HUD. |
| `SetObjectPose ... not movable` | Should no longer appear. If it does, it is a genuine failure now — the no-op case was fixed. |

**Never kill every `UnrealEditor.exe`.** The scripts match on `Blocks.uproject`
specifically, because a blanket kill would take unrelated projects and unsaved
work with them.

---

## 7. Where to read more

| question | file |
|---|---|
| What to say in the presentation | `docs\CHEATSHEET-Follow-Aug2026.md` |
| The slides | `docs\VLA-Guardrail-Follow-Aug2026.pptx` |
| Why we stopped using AerialVLA | `docs\FINDING-what-drives-aerialvla.md` |
| The target was never really a car | `docs\FINDING-vehicle-mesh-and-materials.md` |
| Two defects a flight could not have found | `docs\FINDING-guardrail-coverage-aug2026.md` |
| The teleport fault that was not a sim bug | `docs\FINDING-setobjectpose-not-a-sim-bug.md` |
| Three fence fixes that were not the problem | `docs\FINDING-gapfence-was-never-the-fence.md` |
| Can it say the object is absent? | `docs\RESULT-absence-detection.md` |
| The orbit task | `docs\DESIGN-orbit-building-task.md` |
| For the team: COCO and the frame contract | `docs\TEAM-NOTE-coco-and-frame-contract.md` |

---

## 8. What to say if someone asks how it works

Four stages, and it is worth being precise about which is a neural network and
which is not:

1. **Detect** — OWL-ViT takes your phrase and the camera frame, returns scored
   boxes. This is the neural network, and it is open-vocabulary: no fixed class
   list, no training on this scene. *153 M parameters, 3.9–5.4 Hz.*
2. **Verify** — a fixed HSV rule checks the pixels inside the box really are the
   named colour. **Not** a network. This is what makes the colour word mean
   something.
3. **Act** — a hand-written control law turns the box into motion: horizontal
   offset becomes yaw, box width becomes forward speed. **Not** a network either.
4. **Guard** — the Safety Shield checks every action against the policy before it
   flies. Nothing reaches the simulator unchecked.

Be honest about stage 3: the action is engineered, not learned. A 7 B model that
ignored its instruction was replaced by a 153 M detector that obeys it, and the
control law was written by hand. That is a real trade and it is better to say it
than to have it found.

And the claim that actually matters: **the guardrail was never modified.** Not
one line of `guardrail\shield.py`, not one policy value, through a complete
change of pilot, a new fleet of vehicles, and every new mission. NFZ time and
altitude escape are 0.0 s on every flight ever recorded.
