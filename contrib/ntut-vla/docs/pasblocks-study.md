# PASBlocks — Repo Study

*Studied 2026-07-16. Location: `PASBlocks/` (25.3 GB).*

## What it is

**The Unreal Engine 5.7 source project for Project AirSim's "Blocks" world** —
the buildable sim ENVIRONMENT that pairs with the Project AirSim client
libraries we already downloaded. Where our current AirSim rail uses prebuilt
.exe worlds (AirSimNH, Blocks, LandscapeMountains on old UE 4.27), PASBlocks is
the modern UE5 equivalent you can open in the editor, modify, and rebuild.

```
PASBlocks/
├── Blocks.uproject            UE 5.7, plugin ProjectAirSim enabled
├── Source/Blocks/             thin game module (the world itself is content)
├── Binaries/Win64/            ALREADY BUILT: Blocks-Win64-DebugGame.exe
│                              + onnxruntime CUDA DLLs (in-sim ML inference)
├── Plugins/
│   ├── ProjectAirSim/         the sim core (CreatedBy: Microsoft)
│   │   └── SimLibs/           core_sim, multirotor_api, mavlinkcom (PX4/ArduPilot
│   │                          link), physics, rendering_scene, nng (transport)
│   ├── Drone/                 quadrotor vehicle assets
│   └── Rover/                 ground vehicle assets
├── Content/                   ← the big surprise: FOUR extra environments
│   ├── Airport/               Demo_day.umap, Demo_night.umap
│   ├── JapaneseCity/          demo.umap, showcase.umap  ← URBAN, dense
│   ├── MilitaryAirport/       Map_Airbase_Demo.umap
│   └── HelicopterAirStrike/   scenario content
└── Saved/Logs/Blocks.log      last run 2026-07-14 (it already launches)
```

## Why it matters for our project

1. **This is the UE5 rail's world.** Prof's reference repo
   (`kuanting-vla-uav-guardrail/demo/projectairsim_demo.py`) targets Project
   AirSim — PASBlocks is the server side that demo talks to. Running our
   guardrail against it = 5th rail, and the one Prof's repo treats as canonical.
2. **Better demo worlds than AirSimNH.** JapaneseCity (dense urban — ideal for
   NFZ stories), Airport day/night, MilitaryAirport. All UE5 lighting → much
   better-looking videos for the final report than UE 4.27 AirSimNH.
3. **ArduPilot-ready.** `mavlinkcom` SimLib means the same world can back a
   SITL/MAVLink rail — closer to the grant's "ArduPilot UAVs" wording than the
   plain AirSim RPC API we use today.
4. **onnxruntime CUDA ships in the binary** — the sim itself can run ONNX
   models; our exported `vla_policy_v2.onnx` could in principle run in-process.

## How to run it

Prebuilt binary (DebugGame config):
```powershell
& "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone\PASBlocks\Binaries\Win64\Blocks-Win64-DebugGame.exe"
```
Or open `Blocks.uproject` in UE 5.7 editor (needs Engine installed) and pick a
map: Blocks default, or `Content/JapaneseCity/Maps/demo.umap`, etc.

Client side: Project AirSim Python client (`projectairsim` package — downloaded
under task "Project AirSim binaries") connects over NNG, NOT the old
`airsim`/msgpack-rpc API. Adapter work = new `get_frame` + pose/velocity calls;
guardrail core unchanged (as always).

## Caveats

- 25.3 GB, sits inside OneDrive — consider excluding from sync (Intermediate/
  DerivedDataCache churn will hammer OneDrive).
- DebugGame build = slower than Shipping; fine for demos.
- UE 5.7 editor required only for modifying worlds; the prebuilt exe runs as-is.
- Old `airsim` Python package does NOT talk to it — different protocol.

## Suggested next step

Port `demo/real_vla_demo.py`'s adapter to the `projectairsim` client and fly
the guardrail in JapaneseCity — same story, canonical Prof-repo rail, far
better visuals.
