# Project AirSim — Setup & Status (verified working 2026-07-07)

Second AirSim rail: **Project AirSim v0.2.0** (IAMAI, UE 5.2) — the actively
maintained successor to the AirSim we use daily. Proven flying on this machine.

## What's installed where

| Piece | Location |
|---|---|
| Neighborhood world (UE5) | `D:\ProjectAirSim\Neighborhood\Neighborhood-Windows-UE5.2-PAS_v0.2.0\AirSimNH.exe` |
| Client repo (sparse: client dir only) | `D:\ProjectAirSim\repo` |
| Python client env | conda **`pas`** (py 3.10, `projectairsim` installed editable) |
| Server log | `<world dir>\AirSimNH\projectairsim_server.log` |

> Their `Blocks-Windows-UE5.2` release zip is MISLABELED (contains the Linux
> UE5.7 plugin). Use Neighborhood; report/re-check Blocks upstream later.

## How to run (verified sequence)

```powershell
# 1. start the world (UE5 — first launch compiles shaders, be patient)
D:\ProjectAirSim\Neighborhood\Neighborhood-Windows-UE5.2-PAS_v0.2.0\AirSimNH.exe

# 2. server is ready when ports 8989 (topics) + 8990 (services) listen:
Test-NetConnection 127.0.0.1 -Port 8990

# 3. fly the official example:
conda activate pas
cd D:\ProjectAirSim\repo\client\python\example_user_scripts
python hello_drone.py     # takeoff -> up -> down -> land
```

## Key differences vs our current AirSim rail

| | AirSim 1.8.1 (current) | Project AirSim v0.2.0 |
|---|---|---|
| Protocol | msgpack-RPC, port 41451 | **pynng (NNG)**, ports 8989/8990 |
| Client | `airsim` pip pkg | `projectairsim` (from repo, editable install) |
| World/robot config | `settings.json` (global) | **JSONC scene + robot configs, sent BY THE CLIENT** (`World(client, "scene_x.jsonc")`) |
| Engine | UE4 | UE 5.2 |
| Empty world at start | drone auto-spawns | **no vehicle until a client creates the world** — that's why no drone is visible on launch |
| Maintenance | archived 2022 | active (2026 commits) |

## Adapter plan (next step when we want the guardrail here)

The client API has direct equivalents for everything our adapter needs:
- state: pose topics / `drone.get_ground_truth_kinematics()`
- velocity control: `drone.move_by_velocity_async(v_north, v_east, v_down, duration)`
- camera: image topics (subscribed callbacks)

So `demo/run_demo.py`'s AirSim calls map ~1:1; guardrail core untouched (adapter
isolation, proven 5× if we count this). Estimated effort: one working session.

## Useful extras spotted in their examples

`client/python/example_user_scripts/` includes `ardupilot/` integration samples
and `autonomy/` demos — relevant later for unifying the SITL rail with UE5
visuals (SITL + Project AirSim rendering in one setup).
