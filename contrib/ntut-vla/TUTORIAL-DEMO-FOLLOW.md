> **SUPERSEDED.** See `TUTORIAL.md` in the project root, which is the single verified guide. This file is kept as history only and its commands, tags and numbers may be out of date.

# Demo: a drone follows a car it was told about, in words

Step by step, from a cold machine. Everything here was run to produce the numbers
below.

> **Environment:** conda env `vla-real`
> (`C:\Users\natha\.conda\envs\vla-real\python.exe`), from the project root
> `D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone`.

---

## The 60-second version

```powershell
.\run_follow_vlm.ps1
```

Flies **two** demos, about 15 minutes:

1. **`vlm_demo`** — pure tracking. The car drives a circuit; the drone follows it
   because you named it.
2. **`vlm_nfz`** — the same mission with a no-fly zone across the route. The car
   drives through it. The drone may not, and does not.

Then open:

```
demo\out\vlm_demo\vlm_demo_demo.mp4
demo\out\vlm_nfz\vlm_nfz_demo.mp4
```

Left half is the drone's own camera with the detection box and live telemetry
drawn on it; right half is the third-person chase view. In the second video the
HUD flips to a red **GUARDRAIL: correcting** while **TARGET LOCKED** stays up —
the drone can see where it wants to go and is being stopped from going there.

With the two control flights as well (~25 min):

```powershell
.\run_follow_vlm.ps1 -Controls
```

---

## What is actually happening

You type words. A detector finds that thing in the camera. A servo loop chases
the box. The Shield checks every action. **No coordinates reach the controller** —
the car's position is used only to spawn it and, offline, to score.

| part | file | role |
|---|---|---|
| detector | `google/owlvit-base-patch32` (153 M params, Apache-2.0) | open-vocabulary: text in, boxes out |
| colour check | `colour_match()` in `demo/follow_vlm.py` | verifies the box really is the named colour |
| controller | `servo()` in `demo/follow_vlm.py` | box offset → yaw, box width → speed, altitude error → climb |
| the car | `demo/moving_car.py` | `SM_Offroad_Body` from `PASBlocks/Plugins/Rover`, painted orange, driving a closed circuit at 2.5 m/s |
| guardrail | `guardrail/shield.py` | unchanged from every other mission |

---

## Step by step

### 1. Start the simulator

**PowerShell, not Git Bash.** Git Bash rewrites the `/Game/...` map argument into
a Windows path, the map is not found, and the engine crashes on the fallback.

```powershell
Start-Process "C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe" -ArgumentList '"D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone\PASBlocks\Blocks.uproject"','/Game/JapaneseCity/Maps/Demo_day','-game','-windowed','-ResX=1280','-ResY=720'
```

Wait for it:

```powershell
while (-not (Test-NetConnection 127.0.0.1 -Port 8989 -WarningAction SilentlyContinue).TcpTestSucceeded) { Start-Sleep 5 }; "sim ready"
```

If another Unreal project is open, it is safe — every script here kills only
processes whose command line contains `Blocks.uproject`.

### How the car drives

It used to ping-pong along a straight line: reach the end, flip its heading 180°
in a single frame, drive back. That looked wrong on video and actively broke
tracking — the continuity filter rejects a target that teleports its orientation.

It now drives a **closed 111 m circuit** with rounded corners and a speed
profile. Measured: heading changes at most **3.1° per 0.1 s** (a 31°/s cornering
rate, what a real car does) instead of 180° instantly, speed varies by at most
0.06 m/s per tick, and the whole route stays ≥ 10 m from any building.

One detail that matters: the car does **not** start next to the drone. A first
attempt put it 4 m away, which at 9 m altitude is a 66° depression angle — under
the camera entirely. The detector saw it zero times in 132 s. `phase_s` starts it
17.9 m ahead instead, at a 27° depression, comfortably in frame.

### 2. Fly the mission

Pure tracking:

```powershell
python demo\follow_vlm.py --object "an orange car" --tag vlm_demo --policy policies\follow_car.yaml --max-s 150 --det-thresh 0.008 --car-speed 2.5 --save-view
```

With the no-fly zone across the route:

```powershell
python demo\follow_vlm.py --object "an orange car" --tag vlm_nfz --policy policies\follow_car_nfz.yaml --max-s 150 --det-thresh 0.008 --car-speed 2.5 --save-view
```

**Restart the simulator between flights.** It reliably refuses the next
connection once a flight has disconnected; reusing it costs a run and the failure
looks like `ticks 0` rather than an error. `run_follow_vlm.ps1` does this for you.

Expect:

```
[grounder] colour prior: 'orange' (a box must be >=10% that colour to qualify)
[view] third-person + annotated frames -> ...\demo\out\vlm_demo\view
[flight] cruise 9 m - following 'an orange car'
  tick 300: pos=(  33.2,  -6.3, 9.0) TRACK  brg= -4.3 fwd= 0.2 sep= 15.5m shield=-
[report] ticks 1210 | detector 5.41 Hz, hit rate 0.318 | target visible on 90% of ticks
[report] separation: min 0.5 m, mean 19.6 m, end 14.3 m, within 30 m 0.821
[report] guardrail: NFZ 0.0s, altitude escape 0.0s, interventions 18
```

The word after the position is the mode — `TRACK`, `COAST`, `SEARCH` or `SCAN`
(see below). `brg` is how far off-centre the box was; `sep` is the true distance
to the car, which the controller never sees.

### 3. Build the video

```powershell
python tools\make_demo_video.py --tag vlm_demo --fps 10
```

### 4. Run the controls, if you want the claim to hold up

```powershell
python demo\follow_vlm.py --object "a blue car" --tag vlm_wrongcolour --max-s 120 --det-thresh 0.008
```

Same car, wrong colour word. It should **fail** to follow — that is what shows
the words are doing the work rather than the drone drifting into the car.

---

## Results

| condition | detector hit rate | mean separation | within 30 m | shield ticks |
|---|---|---|---|---|
| **straight route** | **99.8%** | **13.3 m** | **100%** | 0 |
| **straight + no-fly zone** | 60.5% | 51.4 m | 17.5% | **659** |
| *earlier: closed circuit* | 20–44% | 19–27 m | 65–90% | 18–55 |
| *control — "a blue car"* | — | 68.5 m | 21% | — |

Guardrail on every one: **NFZ 0.0 s, altitude escape 0.0 s.**

**The straight route is the tracking demo.** The detector holds the car on
essentially every frame and the drone never falls outside 30 m. The circuit rows
are kept for honesty: a loop looks more natural but its turns swing the target
through the aircraft's blind spot, and tracking was measurably worse for it.

**The no-fly-zone row is the safety demo, and it is a deliberate failure.** The
fence spans the whole corridor with no way around it. The car drives through; the
drone is held at the boundary for 659 ticks and the car escapes. Tracking
collapses to 17.5% — and the zone is still never entered. The guardrail cancelled
the mission rather than bending, which is the point.

**The control is what makes tracking a result rather than a video.** Same scene,
same orange car, one word changed to "a blue car", and following collapses to
21%.

### Losing the target, and what it does about it

The drone used to freeze whenever detection dropped, which happened on about a
quarter of ticks. It now has four modes, shown live in the HUD:

| mode | when | what it does |
|---|---|---|
| `TARGET LOCKED` | detection is fresh | servo on the box |
| `COASTING` | lost < 2 s | keeps turning the way the target was moving, decaying |
| `SEARCHING` | lost < 7 s, or never acquired yet | sweeps toward the side it last saw |
| `SCANNING` | lost longer | keeps sweeping slowly — the car is on a circuit and will come back |

Two bugs were found here, both worth knowing about.

**Before the first detection it used to give up immediately.** `last_seen = 0`
made the "lost" timer read as infinite, so it skipped coast and search and held
its launch heading for the whole flight while the car drove a lap behind it.
Never-yet-acquired now means search.

**Detection age was measured against the wrong clock.** The detector stamped
`_t` on every *inference*, hit or miss, and never cleared the last box. So a
stale detection always looked fresh: the aircraft kept chasing a box that was no
longer there and the HUD kept reporting TARGET LOCKED after the car had left the
frame. `_t_det` now records when it last actually *found* something, and the box
is dropped after 8 s.

That second bug means any "target held on N% of ticks" figure from before this
fix was measuring nothing — it was true by construction. The numbers in the table
above are from after it.

---

## Reading the numbers honestly

**`sep_min` is not success.** A drone that never moves still records ~12 m,
because the car drives past it. Judge by **`sep_mean`** and
**`frac_within_30m`**.

**`hit_rate` around 0.3 is correct, not broken.** It is the fraction of detector
frames that survive the colour check. Before the colour check it was 0.82 — and
that flight was *worse*, because the extra detections were city clutter. One of
them held the aircraft on station 60 m from the car.

**Detector scores are low in absolute terms** (0.03–0.07 for the true car at
22 m). The box lands in the right place, which is all a servo loop needs, but do
not read the score as confidence that the object is present.

**n = 4.** Consistent — 16.4 to 19.2 m mean separation — but four flights.

---

## Tuning, if you need to

| flag | default | effect |
|---|---|---|
| `--object` | "an orange car" | **be specific.** "a car" alone scored 51 m mean separation against 16 m for "an orange car". |
| `--colour-min` | 0.10 | fraction of the box that must really be the named colour. 0 disables the check. |
| `--det-thresh` | 0.02 (use 0.008) | detector score floor. Low by necessity — real scores are 0.03–0.07. |
| `--want-width` | 0.10 | target apparent width as a fraction of the frame; sets the standoff distance |
| `--yaw-gain` | 1.2 | how hard it turns to centre the target |
| `--speed-max` | 4.0 | chase speed cap. Must exceed the car's 3 m/s or it can never catch up. |
| `--cruise-alt` | 9.0 | see the camera geometry note below |

**Why 9 m and not higher.** The front camera is pitched 20° down with a 29.4°
vertical half-FOV, so it sees ground from 0.86 × altitude outward. Horizontal — as
it was originally — that figure is 1.79 × altitude, which meant the car left the
frame exactly when the drone got close enough to matter. Pitching it down was one
of the two changes that made tracking work.

---

## Troubleshooting

| symptom | cause / fix |
|---|---|
| Engine window opens then dies | launched from Git Bash; use PowerShell (step 1) |
| `pynng ConnectionRefused` | the sim is not up, or it crashed; restart it |
| Drone flies off and holds station far from the car | it locked onto city clutter. Use a more specific `--object`, and check `--colour-min` is not 0. |
| `hit_rate` near 1.0 and poor following | the colour check is not engaging — is there a colour word in `--object`? |
| No video produced | the flight needs `--save-view`; check `demo\out\<tag>\view\` has frames |
| `numpy` errors after any `pip install` | re-pin: `pip install --force-reinstall numpy==1.26.4` |

---

## What this does not do

The system grounds **"car"** well and **"orange"** usefully. It does not
understand the scene: it has no notion of which car, no memory of the target
across occlusions beyond a simple continuity filter, and no way to tell you
whether the object is present at all — with no car in the scene the raw detector
still fires on most frames, and only the colour check suppresses it.

It follows. It does not reason.
