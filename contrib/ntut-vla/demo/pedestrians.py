"""
People in the city, as scenery.

Scope, set at the 2026-08-25 review: pedestrians are DECORATION. Nothing is meant
to detect them and no rule refers to them yet. Rules come later, and the
machinery is already waiting - `SubjectStandoff` is a constraint type,
`subject_class` selects which rule binds, and policies/follow_pedestrian.yaml
already says "10 m from a person, 5 m from anything else".

Two consequences follow from "decoration", and they pull in opposite directions.

STANDING FIGURES ARE FREE. Once spawned they are never touched again: no
per-tick RPC, so they cannot slow the control loop or take anything from the
detector. det_hz only just cleared its 4.0 Hz gate, so that matters more than it
sounds. This is why most of them stand.

BUT A DECORATIVE FIGURE MUST NOT STEAL THE LOCK. The demo query is "a yellow
car" and the grounder ranks by score x (0.25 + 0.75 x colour_match), so a person
in a yellow jacket standing beside the route is exactly the kind of thing that
has taken a lock before. Hence: figures go on PAVEMENTS, never on the
carriageway the subject drives, and det_hit_rate is treated as a gate on this
change rather than an observation about it.

Where the models come from
--------------------------
Quaternius "Animated Men Pack", CC0 (public domain), four figures, fetched as
GLB from poly.pizza. They are RIGGED, and Project AirSim's importer keeps no
bones (AssimpToProcMesh), so what gets spawned is whatever pose the vertices are
already in. tools/bake_glb_poses.py resolves that by doing the skinning offline
and writing a static mesh - see docs/FINDING-scene-motion-belongs-to-the-simulator.md.

Their bind pose turned out to be an A-pose rather than the T-pose that was
feared, so baking mostly buys correct world-space geometry rather than rescuing
the pose. It is still worth doing: the baked file carries no node transforms, so
it cannot depend on what the importer does with a node hierarchy.

Like the vehicle fleet, this degrades quietly. `posed_dir()` returns None when
the models were never fetched, the flight says so once, and everything else
carries on - the repository stays runnable for anyone without third-party assets.
"""
from __future__ import annotations

import math
import os
import pathlib
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# Baked by tools/bake_glb_poses.py from the Quaternius pack. Point
# VLA_PEOPLE_DIR elsewhere, or pass --people-dir.
PEOPLE_DIR_ENV = "VLA_PEOPLE_DIR"
PEOPLE_DIR_DEFAULT = "D:/models/quaternius_people/posed"

# Measured on the baked meshes: 4.806, 4.812, 4.821, 4.812 units tall, so
# 1.75 / 4.81 puts them at human height. Getting this wrong by a factor is
# obvious; getting it wrong by twenty percent is not, which is why it is derived
# from the measurement rather than guessed.
#
# This read 4.85 while the baker still edited the source file in place, because
# that file kept the ORIGINAL bind-pose POSITION accessors alongside the posed
# ones and a bounding box over all of them is the union of two poses. The clean
# writer emits only the posed geometry, so this is now the figure's real height.
MODEL_HEIGHT_UNITS = 4.81
TARGET_HEIGHT_M = 1.75
GLB_SCALE = TARGET_HEIGHT_M / MODEL_HEIGHT_UNITS

STAND_FILES = ["stand_a.glb", "stand_b.glb", "stand_c.glb", "stand_d.glb"]

# A pacing figure in a STANDING pose glides, which reads worse than not moving
# at all. These are the same four models baked mid-stride (Man_Run at 0.18 s):
# measured stride 1.29 m and the head dips from 1.75 m to 1.62 m, as a walking
# person's does. Still a single static pose - the importer keeps no bones - but
# a moving figure in a stride pose reads as walking at the 11-20 m the camera
# sees them from. Optional: if they were never baked, walkers fall back to
# standing meshes rather than failing.
WALK_FILES = ["walk_a.glb", "walk_b.glb", "walk_c.glb", "walk_d.glb"]

# A pedestrian walks at about 1.4 m/s. At the ~8.7 Hz control loop that is a
# 16 cm step per update, against the 54 cm that made the CAR look choppy - a
# third of the distance on a subject moving half as fast, so the same
# teleport-per-tick artefact is far less visible here. The simulator-side
# trajectory route is not open to us: env-actor links take a packaged
# unreal_mesh and cannot take a glTF.
WALK_SPEED_MPS = 1.4


def searched_dir(explicit: Optional[str] = None) -> str:
    """The path that WOULD be used, whether or not anything is there."""
    return explicit or os.environ.get(PEOPLE_DIR_ENV) or PEOPLE_DIR_DEFAULT


def posed_dir(explicit: Optional[str] = None) -> Optional[pathlib.Path]:
    """Where the baked figures live, or None if they were never fetched."""
    p = pathlib.Path(searched_dir(explicit))
    if not p.is_dir():
        return None
    return p if any(p.glob("stand_*.glb")) else None


@dataclass
class Figure:
    """One spawned person. `walker` is None for the standing majority."""
    name: str
    glb: pathlib.Path
    x: float
    y: float
    heading: float = 0.0
    actual_name: Optional[str] = None
    walker: Optional[object] = None          # a MovingCar-style path follower
    _failed: bool = False


def _pose(x: float, y: float, h: float, z: float = 0.0):
    """NED pose, built exactly the way demo/moving_car.py builds one.

    The TYPES matter, not just the fields. A plain dict of the same shape is
    refused by the RPC layer with `ERROR code: 2.0, message: Unknown exception`
    - which reads like a bad mesh and is not one. It must be a `Pose` holding a
    `Vector3` and a `Quaternion`, and it must carry `frame_id`.

    z is DOWN-positive at this boundary, so 0.0 is ground level.
    """
    from projectairsim.types import Pose, Quaternion, Vector3
    from projectairsim.utils import rpy_to_quaternion
    w, qx, qy, qz = rpy_to_quaternion(0.0, 0.0, h)
    return Pose({
        "translation": Vector3({"x": float(x), "y": float(y), "z": float(z)}),
        "rotation": Quaternion({"w": w, "x": qx, "y": qy, "z": qz}),
        "frame_id": "DEFAULT_ID",
    })


def _dist_to_route(x: float, y: float,
                   route: List[Tuple[float, float]]) -> float:
    """Distance to the route POLYLINE, not to its nearest corner.

    Measuring to waypoints is what emptied the first populated flight of
    visible people. The demo route is three points spanning the whole map, so
    a pavement halfway down a 48 m straight measured as ~16 m from the nearest
    CORNER while being 11 m from the road the camera actually flies along -
    and spots genuinely near the far end of the map measured as "close"
    because one distant waypoint happened to be within the band. Figures ended
    up 12 to 42 m away, mostly where the camera never looks.
    """
    if not route:
        return 0.0
    if len(route) == 1:
        return math.hypot(x - route[0][0], y - route[0][1])
    best = float("inf")
    for (ax, ay), (bx, by) in zip(route, route[1:]):
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
        best = min(best, math.hypot(x - (ax + t * vx), y - (ay + t * vy)))
    return best


def pavement_spots(street_mask, buildings, n: int, rng: random.Random,
                   avoid: List[Tuple[float, float]] = (),
                   avoid_radius_m: float = 7.0,
                   near_radius_m: float = 20.0) -> List[Tuple[float, float]]:
    """Cells that are street AND touch a building - i.e. a pavement.

    The street mask alone will not do: it marks roads and pavements alike, so
    sampling it directly drops people in the middle of a carriageway. A cell
    that is walkable and adjacent to a building footprint is against a
    shopfront, which is where people actually stand.

    Placement is a BAND along the route, not a scatter across the map. Too
    close and a figure competes with the subject for the tracker's attention;
    too far and it is decoration nobody ever sees, since the camera only ever
    looks near the route. So: at least `avoid_radius_m` from the route line, at
    most `near_radius_m` - measured to the polyline, which is the whole point,
    see `_dist_to_route`.
    """
    st = street_mask["street"]
    res = street_mask["res"]
    ox, oy = street_mask["ox"], street_mask["oy"]
    ni, nj = st.shape

    def touches_building(i: int, j: int) -> bool:
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < ni and 0 <= b < nj and buildings[a, b]:
                return True
        return False

    spots: List[Tuple[float, float]] = []
    for i in range(ni):
        for j in range(nj):
            if not st[i, j] or not touches_building(i, j):
                continue
            # The cell's centre is `ox + i*res`. Adding half a cell was the
            # only place in this repository that did so - the other twelve
            # world-from-grid conversions all use this form - and it put every
            # figure exactly 1 m from the cell that had just been validated as
            # pavement, on the boundary between two cells. Checked against a
            # correctly indexed street lookup, 5 of 12 figures in the
            # `city_people` flight were not standing on street at all.
            x = ox + i * res
            y = oy + j * res
            if avoid:
                d = _dist_to_route(x, y, list(avoid))
                if d < avoid_radius_m or d > near_radius_m:
                    continue
            spots.append((x, y))

    rng.shuffle(spots)
    return spots[:n]


class _Pace:
    """Back-and-forth along one straight stretch of pavement.

    Deliberately not a route. A walker that wanders needs its whole path
    checked against the pavement, and every metre of that is a chance to put a
    figure in a carriageway - which is the one placement mistake that can cost
    the demo its lock. Pacing a segment that was verified walkable once cannot
    drift off it.

    Motion is a triangle wave in arc length, so speed is constant and only the
    heading flips at the ends. `pose_at` is a pure function of time, which is
    what lets the caller sample it at whatever rate the control loop happens to
    run at.
    """

    def __init__(self, x0, y0, x1, y1, speed=WALK_SPEED_MPS):
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1
        self.len = math.hypot(x1 - x0, y1 - y0)
        self.speed = speed
        self.period = 2.0 * self.len / speed if self.len > 1e-6 else 1.0

    def pose_at(self, t):
        if self.len < 1e-6:
            return self.x0, self.y0, 0.0
        u = (t % self.period) / self.period          # 0..1 over there-and-back
        f = 2.0 * u if u < 0.5 else 2.0 * (1.0 - u)  # triangle wave
        x = self.x0 + (self.x1 - self.x0) * f
        y = self.y0 + (self.y1 - self.y0) * f
        h = math.atan2(self.y1 - self.y0, self.x1 - self.x0)
        if u >= 0.5:
            h += math.pi
        return x, y, h


def _pace_segment(x, y, street_mask, buildings, max_m=10.0, step=1.0):
    """Longest walkable straight run from (x, y), up to `max_m`.

    Tries eight compass directions and keeps the best. Every sample must be
    street AND next to a building - the same pavement test used for placement -
    so the walker stays on the footway for its whole cycle rather than only at
    the point where it was spawned.
    """
    st = street_mask["street"]
    res, ox, oy = street_mask["res"], street_mask["ox"], street_mask["oy"]
    ni, nj = st.shape

    def pavement(px, py):
        i = int(round((px - ox) / res))
        j = int(round((py - oy) / res))
        if not (0 <= i < ni and 0 <= j < nj) or not st[i, j]:
            return False
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (2, 0), (-2, 0), (0, 2), (0, -2)):
            a, b = i + di, j + dj
            if 0 <= a < ni and 0 <= b < nj and buildings[a, b]:
                return True
        return False

    best = (0.0, x, y)
    for k in range(8):
        a = k * math.pi / 4.0
        dx, dy = math.cos(a), math.sin(a)
        d = 0.0
        while d + step <= max_m and pavement(x + dx * (d + step), y + dy * (d + step)):
            d += step
        if d > best[0]:
            best = (d, x + dx * d, y + dy * d)
    return best


class Pedestrians:
    """Spawn a handful of people and, for nearly all of them, forget about it."""

    def __init__(self, world, street_mask, buildings, count: int = 6,
                 walking: int = 2, seed: int = 20260825,
                 people_dir: Optional[str] = None,
                 avoid: List[Tuple[float, float]] = ()):
        self.world = world
        self.rng = random.Random(seed)
        self.dir = posed_dir(people_dir)
        self.searched = searched_dir(people_dir)
        self.figures: List[Figure] = []
        self.count = count
        self.walking = min(walking, count)
        self.street_mask = street_mask
        self.buildings = buildings
        self.avoid = list(avoid)
        self.n_updates = 0

    def available(self) -> bool:
        return self.dir is not None

    def spawn(self) -> None:
        if self.dir is None:
            print(f"[people] no baked figures at {self.searched!r}; the "
                  f"city stays empty. Fetch the pack and run "
                  f"tools/bake_glb_poses.py to populate it.")
            return

        models = [self.dir / f for f in STAND_FILES if (self.dir / f).is_file()]
        if not models:
            print("[people] figure directory has no stand_*.glb; nothing spawned")
            return
        strides = [self.dir / f for f in WALK_FILES if (self.dir / f).is_file()]

        spots = pavement_spots(self.street_mask, self.buildings,
                               self.count, self.rng, avoid=self.avoid)
        if len(spots) < self.count:
            print(f"[people] only {len(spots)} pavement spots clear of the route; "
                  f"asked for {self.count}")

        for k, (x, y) in enumerate(spots):
            glb = models[k % len(models)]
            if k < self.walking and strides:
                glb = strides[k % len(strides)]
            # Face roughly along the pavement, varied so they do not look ranked.
            h = self.rng.uniform(-math.pi, math.pi)
            fig = Figure(name=f"Person{k}", glb=glb, x=x, y=y, heading=h)
            if k < self.walking:
                # A walker needs somewhere to walk. If this spot has no straight
                # pavement worth pacing, the figure simply stands - better a
                # still person than one sliding through a wall.
                run, ex, ey = _pace_segment(x, y, self.street_mask, self.buildings)
                if run >= 3.0:
                    fig.walker = _Pace(x, y, ex, ey)
                    h = math.atan2(ey - y, ex - x)
                    fig.heading = h
                else:
                    # No room to pace, so it will stand - and a stride pose
                    # standing still is worse than a standing pose standing
                    # still. Put the standing mesh back.
                    glb = models[k % len(models)]
                    fig.glb = glb
            try:
                data = glb.read_bytes()
                s = GLB_SCALE
                fig.actual_name = self.world.spawn_object_from_file(
                    fig.name, "gltf", data, True, _pose(x, y, h), [s, s, s], False)
                self.figures.append(fig)
            except Exception as exc:
                fig._failed = True
                if not any(f._failed for f in self.figures):
                    print(f"[people] spawn failed ({type(exc).__name__}: {exc})")

        n_walk = sum(1 for f in self.figures if f.walker is not None)
        print(f"[people] {len(self.figures)} figure(s) on pavements at "
              f"{TARGET_HEIGHT_M:.2f} m (scale {GLB_SCALE:.3f}), "
              f"{n_walk} walking, {len(self.figures) - n_walk} costing "
              f"nothing per tick")

    def update(self, t: float) -> None:
        """Move only the walkers. The standing majority is never touched."""
        for fig in self.figures:
            if fig.walker is None or fig.actual_name is None:
                continue
            x, y, h = fig.walker.pose_at(t)
            if abs(x - fig.x) < 1e-4 and abs(y - fig.y) < 1e-4:
                continue                      # no-op teleports are refused
            fig.x, fig.y, fig.heading = x, y, h
            try:
                self.world.set_object_pose(fig.actual_name, _pose(x, y, h), True)
                self.n_updates += 1
            except Exception:
                pass                          # scenery must never fail a flight

    def destroy(self) -> None:
        # Report the RPC actually spent. A walker that silently never moved
        # looks identical to a standing figure in every metric the flight
        # records, so without this number "3 walking" is an unchecked claim.
        n_walk = sum(1 for f in self.figures if f.walker is not None)
        if self.figures:
            print(f"[people] {self.n_updates} pose update(s) for {n_walk} walker(s)")
        for fig in self.figures:
            if fig.actual_name:
                try:
                    self.world.destroy_object(fig.actual_name)
                except Exception:
                    pass
