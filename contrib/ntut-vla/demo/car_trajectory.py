"""
Hand a MovingCar's route to the simulator as a trajectory, once.

The car used to be teleported, one `set_object_pose` per control tick. The
motion model was never the problem - `moving_car.py` rounds corners, caps
cornering speed by a lateral-acceleration limit, and brakes and accelerates
under a longitudinal one, so the speed it produces is continuous. The problem is
that a continuous model sampled at 8.69 Hz and rendered at 15.57 Hz repeats a
position on 44 % of frames. Fifty-four centimetres, then a pause.

`World.import_ned_trajectory()` takes the whole path in one call and
`EnvActor.set_trajectory()` has the SIMULATOR interpolate it at render rate. The
judder is gone by construction rather than out-run, and there is no per-frame
RPC - which matters, because scene motion and the detector share the simulator's
budget and det_hz only just cleared its 4.0 Hz gate.

A client-side thread was tried first and rejected: the Project AirSim client is
not thread-safe and driving poses from a second thread aborted the mission with
a pynng ConnectionReset. See
docs/FINDING-scene-motion-belongs-to-the-simulator.md.

Ground truth is unaffected. `MovingCar.pose_at(t)` stays the authority for
scoring, because it is a pure function of time and describes exactly the path
that was uploaded.
"""
from __future__ import annotations

import math
from typing import List, Tuple

# The trajectory is a polyline the simulator interpolates BETWEEN, so the sample
# spacing sets how faithfully a curve is reproduced. 20 Hz of samples at 2.5 m/s
# puts a point every 12.5 cm; the tightest corner on the demo route has a 6 m
# radius, where that is a chord error under a millimetre. Denser costs nothing at
# run time - it is one upload - but makes the payload larger for no visible gain.
SAMPLE_HZ = 20.0

# NED. The car sits on the ground, and z is DOWN-positive at this boundary, so a
# small negative value lifts the body clear of z-fighting with the road surface
# rather than burying it.
GROUND_Z = -0.05


def sample_route(car, seconds: float, sample_hz: float = SAMPLE_HZ
                 ) -> Tuple[List[float], List[float], List[float], List[float],
                            List[float], List[float], List[float]]:
    """Sample a MovingCar's own motion model into trajectory arrays.

    Returns (time, x, y, z, roll, pitch, yaw) in NED, ready for
    World.import_ned_trajectory. The car's `pose_at` is the single source of
    truth for both this and the scoring, so the simulator and the metrics cannot
    disagree about where the car was.
    """
    n = max(2, int(seconds * sample_hz) + 1)
    dt = 1.0 / sample_hz

    ts: List[float] = []
    xs: List[float] = []
    ys: List[float] = []
    zs: List[float] = []
    yaws: List[float] = []

    for i in range(n):
        t = i * dt
        x, y, h = car.pose_at(t)
        ts.append(round(t, 4))
        xs.append(float(x))
        ys.append(float(y))
        zs.append(GROUND_Z)
        yaws.append(float(h))

    yaws = _unwrap(yaws)
    zeros = [0.0] * len(ts)
    return ts, xs, ys, zs, zeros, zeros, yaws


def _unwrap(angles: List[float]) -> List[float]:
    """Remove 2*pi jumps so the simulator interpolates the short way round.

    `pose_at` returns a heading wrapped to (-pi, pi]. Interpolating between
    +179 deg and -179 deg without unwrapping spins the car 358 degrees the wrong
    way over one sample interval - a violent flick that would look far worse than
    the judder this is meant to remove. The demo route turns left through 90
    degrees, so it crosses the wrap on some headings.
    """
    if not angles:
        return angles
    out = [angles[0]]
    for a in angles[1:]:
        prev = out[-1]
        d = a - prev
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        out.append(prev + d)
    return out


def upload(world, car, seconds: float, name: str = "car_route",
           sample_hz: float = SAMPLE_HZ) -> dict:
    """Import the route. Does NOT start it - see `start()`."""
    ts, xs, ys, zs, roll, pitch, yaw = sample_route(car, seconds, sample_hz)

    world.import_ned_trajectory(
        name, time=ts, pose_x=xs, pose_y=ys, pose_z=zs,
        pose_roll=roll, pose_pitch=pitch, pose_yaw=yaw)

    return {
        "name": name,
        "samples": len(ts),
        "seconds": round(ts[-1], 2),
        "sample_hz": sample_hz,
    }


def start(env_actor, name: str = "car_route", loop: bool = False) -> None:
    """Begin playback. Call this when the MISSION clock starts, not at setup.

    Playback begins the moment this is called, and the trajectory's t=0 is the
    route's start. Binding it during scene setup therefore has the car driving
    while the aircraft is still arming and climbing - by the time the camera is
    looking, the car is most of the way through its route.

    Measured when this was bound at setup: detector hit rate 0.238, the target
    held on 44 % of ticks, mean separation 73 m against the 16 m the same route
    produces when the two clocks agree. Nothing was wrong with the trajectory or
    the mesh; the car had simply left.
    """
    env_actor.set_trajectory(name, to_loop=loop)
