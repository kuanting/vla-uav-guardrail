# Scene motion belongs to the simulator, not to a client thread

**Date:** 2026-08-25
**Status:** cause established; a threaded fix tried and **abandoned on
evidence**; the simulator-side mechanism **built and proven working**, but held
back as opt-in for one concrete reason given at the end.

## The complaint

Car motion in the demo videos looks choppy and unnatural.

## The cause is the sampling rate, not the motion model

`demo/moving_car.py` already carries a proper vehicle model: rounded corners,
cornering speed capped by a lateral-acceleration limit, and a forward/backward
pass that brakes into corners and accelerates out of them under a longitudinal
limit. The speed it produces is continuous. Nothing about the path is wrong.

What was wrong is how often that continuous model got *sampled*. Measured on the
delivered `demo_follow` flight:

| | measured |
|---|---|
| Car pose updates while moving | **8.69 Hz**, median step **54 cm** |
| Recorder capture rate | **15.57 Hz** |
| Frames during motion repeating a position | **44 %** |
| Background traffic (`update_every = 2`) | **4.4 Hz**, ~72 % repeated |

The car jumped 54 cm and then held still for a frame. In the code the single car
was stepped only on **even** ticks (`tick % 2 == 0`), so it moved at roughly half
an already-slow control loop.

## What was tried, and why it was abandoned

The obvious fix, by analogy with `demo/recorder.py`, was to move scene updates
onto their own fixed-rate thread. In isolation it worked perfectly — 19.96 Hz
achieved against a 20 Hz target, zero late slots.

In flight it **aborted the mission**:

```
pynng.exceptions.ConnectionReset: Connection reset
[car] teleport failed (Canceled: Operation canceled)
[warn] flight aborted: BadState: Incorrect state
```

The Project AirSim client is **not thread-safe**. The driver thread's
`set_object_pose` calls collided with the control loop's RPCs on the same
connection and corrupted it. This is documented in this project's own history —
the real-VLA demo runs inference on a background thread with *its own* AirSim
client for exactly this reason.

Giving the driver its own connection does not work either. `World.__init__`
loads the scene, which would destroy everything already spawned; passing an empty
scene name skips the load (`if scene_config_name:`) but then hangs in
`import_ned_trajectory` against the default topic.

**A client thread is the wrong place for this.** The approach was reverted whole,
and a flight re-run to confirm the repository is back to working: 266 ticks,
hit rate 1.000, 47 Shield interventions, no abort.

## The right mechanism: env actors with trajectories

Project AirSim already moves scene objects itself, and the client API for it is
sitting unused:

```
World.import_ned_trajectory(name, time[], x[], y[], z[], roll[], pitch[], yaw[], ...)
EnvActor.set_trajectory(traj_name, to_loop=True, time_offset=..., x_offset=...)
```

The whole path is uploaded **once**. The simulator then interpolates and moves
the actor at its own render rate. That removes the judder by construction rather
than by out-running it, costs essentially no per-frame RPC, and therefore takes
nothing from the detector — which matters, because `det_hz` only just cleared its
4.0 Hz gate.

One trajectory can drive several actors with per-actor time and position offsets,
which is how a small fleet gets staggered without any client-side scheduling.

### And it is articulated, which is what the pedestrian needs

An env actor is defined by **links and joints**, not a single mesh:

```
EnvActor.set_link_rotation_angle(link_name, angle_deg)
EnvActor.set_link_rotation_angles({link: angle, ...})
EnvActor.set_link_rotation_rate(link_name, deg_per_sec)
```

The shipped quad-tiltrotor example rotates four shroud links to tilt its rotors.
The same mechanism applied to a figure with upper-leg, lower-leg and arm links is
a **real gait**, driven by joint angles, rather than the pose-swapping workaround
planned earlier.

That workaround — baking N static GLBs at N phases of a walk cycle and cycling
which one sits at the walking position — remains technically valid, because a
spawned glTF genuinely cannot animate (`AssimpToProcMesh` produces a procedural
mesh with no bones, and there is no animation API). But it is strictly worse than
articulating an env actor, and it was only chosen because env actors had not yet
been found.

## What this costs

Env actors are **declared in the scene configuration**, not spawned at runtime.
Their links reference geometry, and our own `robot_semantic_quad.jsonc` shows
both forms in use: `"type": "unreal_mesh"` by asset path, and `"type":
"geometry"` with a `box` primitive.

So there are two routes, and they differ in how much Unreal content work they
need:

1. **Primitive links.** A figure assembled from box links — torso, thighs,
   shins, arms, head — needs no imported assets at all. It would be blocky. At a
   9 m cruise a person occupies roughly 10 px of the 400 x 225 detector frame, so
   the *detector* would likely be unaffected, but that has to be measured rather
   than assumed: a box figure that OWL-ViT does not read as "a pedestrian" fails
   the actual mission.
2. **Imported limb meshes.** Human-looking, and needs the limbs brought into
   PASBlocks as separate meshes through the Unreal editor.

The same choice applies to vehicles, except vehicles need no articulation at all
— a trajectory alone fixes them, with the existing packaged or glTF mesh.

## Built and proven

`demo/pas_config/env_actor_car.jsonc` declares the vehicle as an environment
actor, `demo/car_trajectory.py` samples the existing `MovingCar` motion model
into a trajectory, and `follow_vlm.py --car-mode envactor` uploads it once and
lets the simulator play it. Sampled at 20 Hz the route is a point every 12.5 cm,
against the 54 cm teleport steps it replaces.

Two failures on the way, both worth keeping:

**Playback starts when the trajectory is bound, not when the mission does.**
Binding it during scene setup had the car driving through arming and the climb
to cruise, so by the time the camera was looking it had gone. `set_trajectory`
now runs where the mission clock starts.

**Yaw needs unwrapping before upload.** `pose_at` wraps heading to (-pi, pi],
and interpolating from +179 to -179 spins the car 358 degrees the wrong way in
one sample interval. The demo route turns through 90 degrees and crosses that
boundary.

## Why it is opt-in rather than the default

Env-actor links take a packaged `unreal_mesh` by asset path. They cannot take a
glTF, because `spawn_object_from_file` has no equivalent here — and the demo's
subject is `taxi.glb`, chosen because it measured the strongest colour gate of
the fleet at 0.317.

The only packaged car available is `/Rover/OffroadCar/SM_Offroad_Body`, and the
only material this build will bind renders **white**. Against the demo's
standing query, "a yellow car", the colour gate rejects it:

| query | detector hit rate | target held |
|---|---|---|
| `a yellow car` | **0.232** | 50 % |
| `a white car` | **0.958** | 99 % |

Same actor, same trajectory, same flight. Nothing was wrong with the mechanism —
the car was the wrong colour for the question being asked. That measurement is
also the proof that the trajectory itself works.

So `--car-mode spawn` stays the default, keeping the yellow taxi and every
recorded number with it, and `--car-mode envactor --object "a white car"` is the
smooth-motion path. Making it the default needs a yellow car mesh imported into
PASBlocks, which is Unreal editor work.

## Still to do

- Import a coloured car mesh, then make the env actor the default and re-measure
  the duplicate-frame fraction, which is the number the whole exercise is about.
- The pedestrian. Env actors are articulated — `set_link_rotation_angles` over
  named links — so a figure with thigh, shin and arm links gives a real gait
  driven by the simulator. It needs either limb meshes imported, or links built
  from `box` primitives, which our own robot config shows is supported. A box
  figure needs no assets but has to be checked against the detector rather than
  assumed: a pedestrian OWL-ViT does not read as a person fails the mission the
  pedestrian exists for.
