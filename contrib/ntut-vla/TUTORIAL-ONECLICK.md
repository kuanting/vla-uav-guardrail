> **SUPERSEDED.** See `TUTORIAL.md` in the project root, which is the single verified guide. This file is kept as history only and its commands, tags and numbers may be out of date.

# One-Click Demo Guide — Fine-Tuned VLA + Guardrail

Everything below runs the FULL stack automatically — simulator, fine-tuned
7B VLA model, depth-camera obstacle avoidance, Safety Shield — and always ends
with a `trajectory.png` you can show.

## The one script

**Double-click `run_japanesecity.bat`** — that's it. Or from a terminal:

```powershell
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
.\demo_japanesecity.ps1                               # JapaneseCity, default target (40,40)
.\demo_japanesecity.ps1 -Target "60,20"               # pick the destination
.\demo_japanesecity.ps1 -Route "40,40; 45,-15; 60,-45"  # multi-waypoint patrol (verified route)
.\demo_japanesecity.ps1 -Map night                    # night city | airport | military | blocks
```

What it does, in order (fully automatic, ~6-8 min total):
1. Kills any old simulator, launches Project AirSim (UE 5.7) with the chosen
   map — **map load takes ~2 min**, the script waits for server port 8989.
2. Loads the ORIGINAL AerialVLA adapter (`D:\models\aerialvla-lora\aero_vla`, 4-bit,
   ~5.6 GB VRAM, ~25 s load).
3. Flies the route: VLA sees front+down cameras -> depth avoidance layer ->
   rate limiter -> Safety Shield -> sim. 10 Hz control.
4. Prints the KPI report line and **pops `trajectory.png`** (also saved under
   `demo\out\oneclick_<map>\` with `audit.jsonl` + `metrics`).

Expected report shape:
```
[report] AerialVLA on Project AirSim | ticks 586 | shield interventions 42 | NFZ 0.0s -> PASS | target REACHED
```

## The GUI (point-and-click missions)

```powershell
conda activate vla-real
python demo\gui_mission_control.py
```
- LEFT-CLICK map = add waypoint (numbered). RIGHT-DRAG = draw NFZ rectangle.
- Pick map -> "1. Start Sim" -> wait for READY -> "2. Fly".
- Drone position streams LIVE onto the map; log pane shows every event
  ([avoid], waypoint done, shield); result PNG pops at the end.
- Drawn NFZs become real guardrail policy (`policies\gui_policy.yaml`).

## During the flight — what the log lines mean

| Line | Meaning |
|---|---|
| `[avoid] slow: d_center=14.0m ...` | building < 15 m ahead — braking |
| `[avoid] climb+steer: ...` | < 8 m — strafing to the freer side + climbing |
| `[avoid] clear` | path open again |
| `[flight] waypoint 1/2 done` | route progress |
| `[flight] ESCAPE: stalled 6s` | wedged — reversing + climbing for 4 s |
| `... skipping waypoint N` | unreachable point — safe skip, mission continues |
| `shield=HIT` | guardrail repaired an action this tick |

## Known behaviors (say these in a demo, they are features)

- A target behind a building taller than the 25 m altitude ceiling is
  UNREACHABLE by design — the Shield refuses to climb higher; the system tries
  slow/steer/climb/escape, then skips safely. Rules beat wishes.
- `shield interventions` > 0 with `NFZ 0.0s` = the guardrail doing its job.
- The spawn is at (35, -20) on an open street (spawn at (0,0) is under a roof).

## Troubleshooting

- Sim window opens but script says port never appeared: first JapaneseCity load
  compiles shaders — run once more, second load is fast.
- `TimeoutError` / NNG errors: restart the sim (rerun the script — it kills and
  restarts automatically).
- Wrong env: everything must run with `C:\Users\natha\.conda\envs\vla-real\python.exe`
  (the .ps1/.bat and GUI already do).
