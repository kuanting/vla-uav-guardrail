> **SUPERSEDED.** See `TUTORIAL.md` in the project root, which is the single verified guide. This file is kept as history only and its commands, tags and numbers may be out of date.

# Complete Operating Guide — VLA Drone + Guardrail

Everything you need to run the system, in order. Written 2026-08-04, after the
obstacle-clearance / Theta* / RTH / polygon-NFZ / semantic-mission work.

> **Environment for everything below:** conda env `vla-real`
> (`C:\Users\natha\.conda\envs\vla-real\python.exe`). All commands are run from
> the project root `D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone`.

---

## 0. The 30-second version

```powershell
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
conda activate vla-real
python demo\gui_mission_control.py       # click waypoints, draw NFZs, press Fly
```

That is the whole demo. Everything else in this file is detail.

---

## 1. What each piece does (so the commands make sense)

| Layer | File | Job |
|---|---|---|
| Occupancy map | `demo/build_voxel_map.py` | asks the sim for its real building geometry → `demo/out/citymap/occ_<map>.npz` |
| Global planner | `demo/city_planner.py` | Theta* route around buildings + no-fly zones |
| Path follower | inside `demo/aerialvla_pas_demo.py` | arc-length pure-pursuit — flies the planned line tightly |
| VLA pilot | `D:\models\aerialvla-lora\aero_vla` (ORIGINAL adapter) | camera + language → local flight commands |
| Reactive avoidance | inside the flight script | front depth camera: brake < 15 m, side-step < 8 m |
| **Guardrail (Shield)** | `guardrail/shield.py` | validates EVERY action: NFZ, altitude band, speed caps, building clearance |
| Mission GUI | `demo/gui_mission_control.py` | draw the mission, watch it fly |

Order of control each tick:
`VLA → goal/path following → depth avoidance → rate limiter → **Shield** → sim`

---

## 2. First-time setup per world (do once per map)

The planner needs that world's building map. **The sim for that world must be
running first** (see §3 step 1), then:

```powershell
python demo\build_voxel_map.py --out occ_day       # JapaneseCity day
```

Output: `demo/out/citymap/occ_day.npz` (+ `occ.npz` and a preview PNG).
For other worlds use `--out occ_night`, `occ_airport`, `occ_military`.
Without this file the planner is off and only the reactive layer protects you.

---

## 3. The GUI (recommended way to run everything)

```powershell
python demo\gui_mission_control.py
```

**Step by step:**

1. **Map** — pick `day` / `night` / `airport` / `military` / `blocks`.
2. **Altitude** — `High 35-55 m (over roofs)` for city flights,
   `Low 15-25 m (street canyon)` for tight flying.
3. **1. Start Sim** — kills any old sim and launches this world.
   *Wait for `sim: READY` — the first city load takes ~2 minutes.*
   (If you change the map, press **Start Sim again**.)
4. **Draw the mission on the map:**
   - **Left-click** = add a numbered waypoint.
     A click inside a building is snapped to the nearest open spot.
   - **Right-button drag** = rectangular no-fly zone.
   - **Polygon NFZ** (checkbox) → every left-click drops a **vertex**;
     press **Close polygon** to finish. Untick to go back to waypoints.
   - **Return to home** (checkbox) → the drone flies back to the spawn at the
     end; the return leg is planned and guarded like any other.
   - **Undo waypoint / Undo NFZ / Clear all** as needed.
   - The **dashed teal line** is the planned route — it already goes around the
     buildings and your NFZs. If a leg is impossible you get
     `a leg was unreachable`, which is the planner being honest.
5. **2. Fly** — the drone takes off, flies the route, and the log streams live.
6. When it lands, the trajectory plot pops up. Everything is saved under
   `demo\out\gui_flight\`.

**Reading the live log:**

| Line | Meaning |
|---|---|
| `[planner] N user waypoints -> M sub-waypoints` | the route was expanded around obstacles |
| `[shield] obstacle map ... -> obstacle_clearance ARMED` | building-distance rule is active |
| `[avoid] slow / climb+steer` | depth camera sees something close |
| `shield=HIT` | the guardrail corrected that action |
| `[flight] no progress 16s ... skipping waypoint` | unreachable point, skipped safely |
| `NFZ 0.0s -> PASS` | **the KPI: no critical violation** |

---

## 4. Command line (for scripted / repeatable demos)

### 4.1 One-click city demo

```powershell
.\run_japanesecity.bat                                   # double-click also works
.\demo_japanesecity.ps1 -Route "45,45; -40,50; 50,-55"   # your own route
.\demo_japanesecity.ps1 -Map night                       # night / airport / military / blocks
```

### 4.2 Full flight command (all options)

```powershell
python demo\aerialvla_pas_demo.py `
  --best --adapter D:/models/aerialvla-lora/aero_vla `
  --route "42,42; 42,-42; -42,-42" `
  --policy policies\urban_clearance.yaml `
  --citymap demo\out\citymap\occ_day.npz `
  --command "fly to (42, 42) at 6 m/s altitude 45" `
  --clearance 5 --rth --tag mydemo --map-label "my demo"
```

| Flag | What it does |
|---|---|
| `--route "x,y; x,y"` | waypoints (North, East) |
| `--policy` | the rule set (NFZ + altitude + speed + clearance) |
| `--citymap` | the world's building map (planner + clearance rule) |
| `--clearance N` | how far the planner keeps from buildings (auto-raised to the policy rule) |
| `--rth` | return to spawn at the end |
| `--best` | use the tuned follower parameters |
| `--no-planner` | reactive only (no global route) |
| `--no-shield` | **guardrail OFF** — comparison baseline only |
| `--lookahead / --follow-blend` | path-following tuning |

### 4.3 Guardrail OFF vs ON (the money slide)

```powershell
.\demo_compare_guardrail.ps1
```
Flies the same route twice — without the guardrail it cuts through the NFZ
(`NFZ 5.3 s VIOLATED`), with it the same pilot routes around (`NFZ 0.0 s PASS`).

### 4.4 Semantic mission (no coordinates at all)

```powershell
python demo\semantic_demo.py --instruction "fly along the open street and keep clear of the buildings"
```
No target, no planner, no path following — the VLA steers from camera + language
only, and the guardrail still holds. This is the demo that shows what a VLA is
*for*; a classical planner cannot run this mission at all.

> **Fixed 2026-08-04.** This script used to pass `target_xy=(0, 0)` meaning "no
> target", but `(0, 0)` is a real coordinate: the model was still being handed a
> bearing phrase pointing at the world origin (from the spawn point that reads
> `"to your right rear "`). Every semantic result recorded before this date was
> confounded. It now passes `None`, which really does produce no hint.

### 4.6 Proving the VLA and the guardrail run *side by side*

The demos above show the guardrail holding. They do not show the VLA and the
guardrail acting **at the same instant** — and on coordinate missions a sceptic
can fairly say the planner did the flying. This experiment settles it.

The idea rests on one property of the Shield: **every repair passes `yaw_rate`
through untouched**, and the only rule that could clamp it compares a `45.0`
threshold against a value the sim consumes as rad/s, which the VLA never drives
past ±0.44. So **heading is authored entirely by the VLA, and the track is where
the Shield intervenes.** Put a semantic target behind a no-fly zone and both show
up in one log line: the nose stays on the target while the track is bent away.

```powershell
python experiments\verify_shield_yaw.py                 # V3: no sim needed, run first
python demo\semantic_seek.py --dry-spawn --tag probe    # V1: then OPEN the PNG
python experiments\verify_sign_convention.py            # V2: yaw sign vs the sim
python experiments\semantic_ab.py --conditions experiments\conditions_simultaneity.yaml --dry-run
python experiments\semantic_ab.py --conditions experiments\conditions_simultaneity.yaml
python experiments\analyze_semantic_ab.py --conditions experiments\conditions_simultaneity.yaml
```

| File | Role |
|---|---|
| `demo/semantic_seek.py` | the flight: no planner, no follower, no goal blend |
| `demo/semantic_target.py` | spawns a tall, uniquely-coloured prop and reads its true pose back |
| `policies/semantic_conflict.yaml` | one NFZ across the corridor, altitude 18–30 m |
| `experiments/conditions_simultaneity.yaml` | the 8-cell matrix **and the pass thresholds, pre-registered** |
| `experiments/analyze_semantic_ab.py` | metrics, claim scoring, figures |

### 4.6b "Follow the car"

A moving car-sized target driven down a real street, with the VLA asked to follow
it in plain language. Full step-by-step in
[TUTORIAL-FOLLOW-THE-CAR.md](TUTORIAL-FOLLOW-THE-CAR.md); the short version:

```powershell
python experiments\probe_moving_car.py                 # spawn it and LOOK at it
python demo\semantic_seek.py --follow-car --object "orange car" --hint-mode none --policy policies\follow_car.yaml --cruise-alt 9 --start-beta-deg 0 --max-s 120 --tag follow_none
python experiments\analyze_semantic_ab.py --conditions experiments\conditions_follow_car.yaml
```

Result, reproduced: **the drone does not move.** All twelve inferences return
`[0, 49, 49]` — stop and land. Given the coordinate-derived hint instead
(`--hint-mode truth`) it flies, but ends 102.9 m from the car against a
stationary drone's 48.0 m. Read §4.7 for why.

Note there is no car in this simulator — the target is a 4.5 × 2.0 × 1.6 m orange
box, a saloon car's real footprint, moved by client-side teleport. That caveat
belongs with any conclusion drawn from these flights.

### 4.7 Read this before interpreting anything above

**The language slot does not steer this model.** A 108-pass controlled ablation
(`experiments/ablate_image_vs_hint.py`, written up in
[docs/FINDING-what-drives-aerialvla.md](docs/FINDING-what-drives-aerialvla.md))
found that AerialVLA's actions are driven by the `{direction}` phrase in its
prompt — a compass bearing computed from **ground-truth coordinates** — and not
by the object description. Correct and wrong colour words produce
indistinguishable actions. Given no direction phrase at all, the model emits
`LAND` on 11 of 18 real frames.

So a mission like *"follow the red car"* is not achievable with this model, and
the reason is not a missing asset. It is that the slot which steers is the one
derived from coordinates.

Two practical consequences:

* The simultaneity flights run with `--hint-mode truth`, i.e. the model gets the
  direction phrase it was trained on, because that is the only setting under
  which it flies at all. Those flights record `hint_mode: "truth"` and
  `hint_used: true`. **A hinted flight is not a semantic result** — do not quote
  one as evidence of language grounding.
* `--hint-mode none` remains available and is the honest semantic configuration.
  Expect the drone to stop.

**The simultaneity claim survives all of this**, and this is the point worth
carrying into the write-up: whether the pilot is *competent* has no bearing on
whether it is *active at the same instant* as the Shield. An incompetent pilot
still emits a direction the Shield must override. The safety guarantee does not
depend on the pilot's competence — which is a stronger claim for the grant than a
marginal navigation result would have been.

Other things worth knowing:

* **Cruise is 22 m here, not 45 m.** The front camera is horizontal with a 29.2°
  vertical half-FOV, so a ground object only enters the frame beyond 1.79 ×
  altitude. At 45 m the target is invisible until 80 m away.
* **Inference is ~3 s standalone, ~10 s with the sim rendering on the same GPU.**
  A 90 s flight therefore yields roughly nine decisions, not ninety.
* **Where a navigation number is reported, the headline is the *worse* of the two
  mirrored start headings.** A model with a constant yaw bias scores ~1.0 from
  one side and ~0.0 from the other; synthetic validation confirmed this correctly
  rejects a biased model that scored 0.602 from one side alone.

### 4.5 Stress scenarios

```powershell
python demo\gen_random_scenario.py --seed 8 --n 5 --nfz 16 --out policies\hard.yaml
# then fly with the printed ROUTE= and --policy policies\hard.yaml
```
Random waypoints with a no-fly zone on **every** leg, so the planner must detour
around each one.

---

## 5. Editing the rules (the policy YAML)

`policies/urban_clearance.yaml` is the current full example:

```yaml
constraints:
  - id: nfz-edge-1            # no-fly zone (any polygon, not just rectangles)
    type: polygon_fence
    vertices: [{x: 32, y: 5}, {x: 44, y: 5}, {x: 44, y: 17}, {x: 32, y: 17}]
    altitude_floor_m: 0
    altitude_ceiling_m: 60
    margin_m: 1.0             # extra buffer around the zone

  - id: bld-clearance         # minimum distance from BUILDINGS
    type: obstacle_clearance
    min_clearance_m: 5.0
    soft_margin_m: 2.0

  - id: alt-band              # altitude limits
    type: altitude_envelope
    alt_min_m: 35
    alt_max_m: 55

  - id: kin-caps              # speed / climb / turn limits
    type: kinematic_envelope
    speed_max_mps: 4.0
    climb_rate_max_mps: 2.0
    yaw_rate_max_dps: 45.0
```

Every one of these is enforced by the Shield on **every action**, and every
decision is written to `demo/out/<tag>/audit.jsonl` with the policy hash.

**Important interaction:** if `min_clearance_m` is larger than the planner's
`--clearance`, the flight raises the planner automatically — otherwise the plan
hugs a wall while the rule pushes off it and the drone stops making progress.

---

## 6. Outputs of every flight (`demo/out/<tag>/`)

| File | Contents |
|---|---|
| `trajectory.png` | map view + altitude plot |
| `trajectory.json` | every position sample |
| `planned.json` | the planned polyline |
| `metrics.json` | reached, NFZ seconds, interventions, path deviation, length ratio |
| `audit.jsonl` | every shield decision, hash-stamped |
| `live.json` | live position (used by the GUI) |

Path-quality numbers to look for: `mean_dev` ≈ 0.3 m (how tightly it flew the
plan) and `len_ratio` ≈ 1.0 (no wandering).

---

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Sim window opens but never READY | first city load compiles shaders — run again, second load is fast |
| `pynng ConnectionReset` / `Timeout` mid-flight | transient sim hiccup — restart the sim and re-run |
| Wrong world shown after changing the map | press **1. Start Sim** again |
| `a leg was unreachable` | your NFZ/waypoint has no safe route — shrink the NFZ or move the point |
| Drone skips a waypoint | it is behind a building taller than the ceiling, or boxed in — safe by design |
| Planner off / no buildings drawn | that world has no `occ_<map>.npz` yet — run `build_voxel_map.py` (§2) |
| numpy errors after any `pip install` | re-pin: `pip install --force-reinstall numpy==1.26.4` |

---

## 8. What changed recently (so you know what is new)

| Change | Why it matters |
|---|---|
| Ground-truth voxel occupancy map | the GUI map now matches the simulator 1:1 (was the cause of "it flew into a building that looked empty") |
| Theta* planner + clearance cost | routes keep more distance from buildings (+7 %) |
| Arc-length pure-pursuit | the flown path hugs the plan (deviation ~0.3 m, no corner loops) |
| `obstacle_clearance` rule | distance from buildings is now a **hard P0 rule**, not best-effort |
| Euclid inflation | fixed the square kernel's √2 diagonal over-inflation that made routes take the long way |
| Progress-based skip | no more infinite escape loops; every flight is bounded |
| Polygon NFZ | zones can be any shape, not only rectangles |
| Return-to-Home | the drone comes back, with the return leg planned and guarded |
| Semantic demo | shows what the VLA is actually for |
