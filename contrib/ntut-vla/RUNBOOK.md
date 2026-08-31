# RUNBOOK — how to run every simulation yourself

## ⚡ Fastest way: `fly.ps1` (launches sim + flies, one command)

```powershell
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
.\fly.ps1                          # urban world, shield ON
.\fly.ps1 -Shield off              # watch the violation
.\fly.ps1 -Dynamic                 # NFZ appears mid-flight
.\fly.ps1 -World mountains         # other worlds: blocks | mountains | zhangjiajie
.\fly.ps1 -Command "fly to (10, 30) at 5 m/s altitude 18"
```
Or just double-click **`fly.bat`**. It starts/restarts the right AirSim world,
waits for it, runs the mission, and pops the trajectory plot when done.
No conda activate needed (uses the env's python directly).

Cheat-sheet. Four rails, all show the same thing: reckless VLA + Guardrail.
`--shield off` = watch it violate · `--shield on` = watch it get caught ·
`--dynamic` = no-fly zone appears mid-flight.

Results always land in `demo/out/<tag>/` → `trajectory.png`, `report.md`, `audit.jsonl`.

---

## Rail 1 — AirSim (Windows, has 3D graphics you can WATCH)

Terminal = normal PowerShell.

**Installed worlds** (pick ONE, they all serve the same RPC port):
- `D:\AirSim\Blocks\WindowsNoEditor\Blocks.exe` — empty dev world, fastest
- `D:\AirSim\LandscapeMountains\WindowsNoEditor\LandscapeMountains.exe` — snowy mountains (the H2 "virtual mountain" demo world)
- `D:\AirSim\ZhangJiajie\WindowsNoEditor\ZhangJiajie.exe` — karst pillar mountains
- `D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe` — neighborhood (urban demo; use `policies/urban_demo_policy.yaml`, alt band 15–25 m above the houses)

Urban run example:
```powershell
python demo\run_demo.py --shield on --policy policies\urban_demo_policy.yaml `
  --command "fly to (40, 40) at 6 m/s altitude 20" --tag urban_shield_on
```

More worlds: https://github.com/microsoft/AirSim/releases/tag/v1.8.1-windows
(AirSimNH = neighborhood 1.6 GB, Africa 679 MB, Coastline 884 MB, ...) — unzip
to `D:\AirSim\`, run the exe, done. Scripts work unchanged on any world; only
mind PHYSICAL terrain heights vs the policy altitude band.

```powershell
# 1. start the sim world (wait until you see the drone on the runway)
D:\AirSim\Blocks\WindowsNoEditor\Blocks.exe

# 2. in a second terminal:
conda activate airsim
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
python demo\run_demo.py --shield off
# RESTART Blocks.exe between runs (drone stays where it landed!)
python demo\run_demo.py --shield on
python demo\run_demo.py --shield on --dynamic
```

Watch the sim window while it flies. Camera keys: `M` free camera, `Backspace` reset view.
NEVER add `client.reset()` to scripts — sim goes deaf.

## Rail 2 — ArduPilot SITL direct (WSL, no graphics, real flight code)

Manual two-step (good for learning what the scripts automate):

```bash
# inside WSL (type `wsl` in PowerShell first), terminal 1:
bash "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/sitl/start_sitl.sh"

# terminal 2 (second `wsl` window):
cd "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone"
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield on
```
Restart terminal-1's SITL (Ctrl+C, rerun) between missions.

## Rail 3 — ROS 2 + MAVROS (WSL, the grant's final architecture)

Fully automated, one command per run:

```powershell
wsl bash "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/sitl/run_ros2_demo.sh" off
wsl bash "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/sitl/run_ros2_demo.sh" on
wsl bash "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/sitl/run_ros2_demo.sh" on --dynamic
```
It starts SITL + mavros + VLA node + Shield node, flies, reports, cleans up.

## Rail 4 — Gazebo Harmonic (WSL, real physics engine)

Also one command per run:

```powershell
wsl bash "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/sitl/run_gazebo_demo.sh" off
wsl bash "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/sitl/run_gazebo_demo.sh" on
wsl bash "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/sitl/run_gazebo_demo.sh" on --dynamic
```

## Unit tests (no simulator at all — 16 scenarios, 2 seconds)

```powershell
conda activate vla-drone
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
python tests\test_shield.py
```

---

## If something breaks

| Symptom | Fix |
|---|---|
| `TransportError: Retry connection over the limit` | sim not running — start Blocks.exe FIRST, wait for drone to appear |
| `ModuleNotFoundError: No module named 'airsim'` | wrong env — run `conda activate airsim` first (NOT vla-drone) |
| WSL run hangs at "connecting tcp:...5760" | wedged leftover from a failed run: `wsl pkill -9 -f "[a]rducopter"` (scripts now auto-clean via trap, but old leftovers need one manual kill) |
| AirSim: "Choose Vehicle" dialog | settings.json must be at `D:\OneDrive\Dokumen\AirSim\settings.json` |
| AirSim: connection refused | Blocks.exe not running / dialog blocking — restart exe |
| Drone reaches target instantly | it started where it last landed — restart the sim |
| WSL: connection refused 5760 | SITL not up: `wsl pkill -f "[a]rducopter"` then rerun script |
| WSL: arming takes forever | normal — EKF warms up 15–30 s, scripts retry automatically |
| Leftover processes in WSL | `wsl pkill -f "[a]rducopter"` · `wsl pkill -f "[m]avros_node"` · `wsl pkill -f "[g]z sim"` |

## Which rail to demo when

- **Show Prof. Lai team live:** Rail 1 (AirSim) — visible 3D drone.
- **Prove "real autopilot":** Rail 3 (ROS 2) — grant's exact topology, one command.
- **Quick smoke test after code change:** unit tests, then Rail 2.
