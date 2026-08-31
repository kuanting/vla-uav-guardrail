"""
A car that drives a closed circuit, the way a car actually drives.

The first version ping-ponged along a straight line: it reached the end, reversed
its heading 180 degrees in one frame, and drove back. That is bad for the demo
and bad for tracking — an instantaneous heading flip breaks the tracker's
continuity filter, and a target that teleports its orientation looks wrong on
video.

This version drives a closed loop with rounded corners and a speed profile:

  * **Closed circuit.** 111 m around the block, verified clear of buildings by at
    least 10 m the whole way. The car never reverses; it just keeps going.
  * **Rounded corners.** Each corner is a circular arc tangent to both legs, so
    heading is continuous everywhere. No discontinuity for the tracker to lose.
  * **Speed profile.** Cornering speed is capped by a lateral-acceleration limit,
    and the car brakes into corners and accelerates out of them under a
    longitudinal limit — the same forward/backward pass a racing line uses.
    Speed is continuous, so the apparent size the servo loop keys on changes
    smoothly.

Motion is by client-side teleport (`world.set_object_pose(..., teleport=True)`)
rather than an environment actor, because env actors must be declared when the
scene loads and cannot be added to a running sim.

Two kinds of vehicle can be driven, and which one you get depends on whether the
glTF models are installed:

  * **A packaged mesh.** `SM_Offroad_Body` from `PASBlocks/Plugins/Rover` — a real
    vehicle, 3.70 x 1.79 x 1.16 m by the sim's own bounding box — painted orange
    so the colour word in the instruction has something to verify against. This
    is the fallback, and it can produce exactly ONE colour, because the build
    binds exactly one material.
  * **A glTF file**, via `CarSpec.glb_path`. The colour is embedded in the mesh
    file, so no material is bound and the fleet can carry as many different
    colours as the texture atlas holds. Measured better on both the detector
    score and the colour gate; see docs/FINDING-glb-vehicles-aug15.md.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class CarSpec:
    """The car itself. Dimensions come from the mesh, not from a guess."""

    # BACK TO SM_Offroad_Body, and the reason is the ADJECTIVE, not the noun.
    #
    # SKM_SportsCar detects far better as "a car" - 0.1272 against 0.0395 - so it
    # was made the target. That was the wrong trade, and a flight showed it: the
    # box sat on distant pale buildings while the real car filled the lower half
    # of the frame.
    #
    # M_Orange is the only material this build will bind, and it renders WHITE on
    # the sports car and genuinely ORANGE on this mesh. Measured over the scene:
    #
    #     orange   0.0% of pixels        white   8.5% of pixels
    #
    # One pixel in twelve of this city passes the white test - pale concrete,
    # road markings, lit facades. Orange passes essentially nowhere else. The
    # colour gate is what turns "some car-like thing" into "THAT car", so a
    # unique colour is worth more than a better noun score. Optimising the noun
    # and losing the adjective is what broke the tracking.
    #
    # The sports car keeps its place as a DISTRACTOR, where its silhouette is an
    # asset rather than a liability.
    asset: str = "SM_Offroad_Body"
    length_m: float = 3.7
    width_m: float = 1.8
    height_m: float = 1.2
    unit_scale: bool = True            # mesh ships at real scale; do not resize
    materials: List[str] = field(default_factory=lambda: [
        "/Game/Geometry/Materials/M_Orange",
    ])
    desc_match: str = "an orange car"
    desc_mismatch: str = "a red car"
    ground_z_ned: float = 0.0

    # --- glTF vehicles -------------------------------------------------------
    # Set `glb_path` and the car is spawned from a FILE instead of a packaged
    # asset, which is the only way this build produces more than one colour.
    #
    # The constraint that forced it: `set_object_material` binds exactly one
    # material (M_Orange) and refuses every other, including ones that provably
    # exist on disk, and `set_object_texture_from_packaged_asset` returns True
    # while changing nothing. Colour therefore has to arrive INSIDE the mesh
    # file. A glTF carrying an embedded baseColorTexture does that, and measured
    # in-sim it works: taxi renders yellow, police white, sedan red, van blue.
    #
    # Measured against the incumbent at the same pose in the same run:
    #
    #     SM_Offroad_Body + M_Orange   "an orange car" 0.047   colour 0.119
    #     taxi.glb                     "a yellow car"  0.108   colour 0.317
    #
    # so the glTF is better on BOTH the detector score and the colour gate, and
    # supplies four distinguishable vehicles where the packaged path supplies
    # one. See docs/FINDING-glb-vehicles-aug15.md.
    glb_path: Optional[str] = None
    # Kenney ships ~2 units per car; x2 gives a ~4 m vehicle. Measured by eye
    # against the road markings and against the sim's own lane width.
    glb_scale: float = 2.0
    # glTF is Y-up/-Z-forward, the sim is Z-up NED with heading 0 = +x/North;
    # the two do not agree. Measured by sweeping yaw and taking the widest
    # detection box (broadside): the flank appears at 90 and 270 deg, and the
    # bonnet points North at 270. So quaternion yaw = heading + 270 deg.
    # experiments/probe_glb_yaw.py reproduces it.
    glb_yaw_offset_deg: float = 270.0


# Closed circuit around the block. Every leg verified against the occupancy map
# at >= 10 m from the nearest building. 111 m round, about 44 s a lap at 2.5 m/s.
# `phase_s` decides where on the lap the car sits at t=0 — see the constructor.
#
# A wider 151 m circuit was tried first and was measurably worse to follow (mean
# separation 29.0 m against 16.4 m on the old straight route) simply because the
# far side of the loop puts the car 60 m away. Natural motion is worth having;
# a lap so large the target spends half of it out of reach is not.
DEFAULT_ROUTE: List[Tuple[float, float]] = [
    (34.0, -24.0), (34.0, 24.0), (46.0, 24.0), (46.0, -24.0),
]

# Straight run for the follow demo: no turns at all, so the car never swings
# through the aircraft's blind spot. It accelerates, cruises, and brakes to a
# stop at the far end — used with one_shot=True so it parks instead of looping.
# The street at x = 38 is clear of buildings by >= 10 m from y = -40 to y = +60.
STRAIGHT_ROUTE: List[Tuple[float, float]] = [(38.0, -8.0), (38.0, 58.0)]

# Up the street, then LEFT at the intersection onto the cross street.
#
# The city map is a regular block grid: 28 m streets between building blocks, at
# x in [28,54] and y in [28,54] (and mirrored at [-54,-28]). The car's usual
# street runs along y and occupies x in [28,54]; the cross street runs along x
# and occupies y in [28,54]. They meet in the square x,y in [28,54] - the
# intersection already visible in the demo frames, with the zebra crossing.
#
# The corner sits at (38, 40), inside that square, and the second leg runs SOUTH
# along the y = 40 lane - decreasing x - because the cross street is free for
# every x, so going that way gives a long leg without leaving the mapped grid.
# 93 m along the path: 58 s of motion at 2.5 m/s with two 8 s stops, and 0 of
# 521 sampled points outside mapped free space.
#
# WHY THIS IS BACK. The closed circuit was abandoned because its turns swung the
# car through the aircraft's blind spot - the front camera sees nothing closer
# than 0.86 x altitude, about 7.7 m at 9 m - and every lost lock cost a chunk of
# the flight. Three things changed since: the colour gate no longer rejects a
# sunlit car, the target estimator predicts through detection gaps and serves a
# bearing on 100% of ticks instead of 38-55%, and the search sweep is bounded so
# a brief loss no longer becomes a 292 degree spin. A corner is survivable now.
# It is still the risk of this route, and analyse_lost_lock.py is how it is
# checked rather than assumed.
TURN_ROUTE: List[Tuple[float, float]] = [(38.0, -8.0), (38.0, 40.0), (-10.0, 40.0)]

ROUTES = {"straight": STRAIGHT_ROUTE, "turn": TURN_ROUTE}


class _Path:
    """Polyline with circular corner arcs, sampled by arc length.

    `closed=False` treats the route as an open run: no wrap-around, and no
    "corner" invented between the last point and the first. Without that a
    two-point straight line is read as a 180-degree hairpin at each end, which
    both divides by zero and is exactly the manoeuvre the straight route exists
    to avoid.
    """

    def __init__(self, pts: List[Tuple[float, float]], corner_r: float = 6.0,
                 ds: float = 0.25, closed: bool = True):
        self.pts = [np.array(p, float) for p in pts]
        self.closed = closed
        n = len(self.pts)
        segs = []                       # ('line', a, b) | ('arc', c, r, a0, a1, sign)
        corner_idx = range(n) if closed else range(1, n - 1)
        for i in corner_idx:
            p_prev, p, p_next = self.pts[(i - 1) % n], self.pts[i], self.pts[(i + 1) % n]
            v_in = p - p_prev
            v_out = p_next - p
            li, lo = np.linalg.norm(v_in), np.linalg.norm(v_out)
            if li < 1e-6 or lo < 1e-6:
                continue
            u_in, u_out = v_in / li, v_out / lo
            cross = u_in[0] * u_out[1] - u_in[1] * u_out[0]
            dot = float(np.clip(np.dot(u_in, u_out), -1, 1))
            turn = math.atan2(cross, dot)
            if abs(turn) < 1e-3:
                continue
            # tangent distance from the corner for an arc of radius corner_r
            r = min(corner_r, 0.45 * li, 0.45 * lo)
            tan_d = r / math.tan((math.pi - abs(turn)) / 2)
            tan_d = min(tan_d, 0.45 * li, 0.45 * lo)
            r = tan_d * math.tan((math.pi - abs(turn)) / 2)
            a_start = p - u_in * tan_d
            a_end = p + u_out * tan_d
            nrm = np.array([-u_in[1], u_in[0]]) * (1 if turn > 0 else -1)
            centre = a_start + nrm * r
            th0 = math.atan2(a_start[1] - centre[1], a_start[0] - centre[0])
            th1 = math.atan2(a_end[1] - centre[1], a_end[0] - centre[0])
            sign = 1 if turn > 0 else -1
            while sign * (th1 - th0) < 0:
                th1 += sign * 2 * math.pi
            segs.append(("corner", i, a_start, a_end, centre, r, th0, th1))

        corners = {c[1]: c for c in segs}
        pieces = []
        leg_idx = range(n) if closed else range(n - 1)
        for i in leg_idx:
            p, p_next = self.pts[i], self.pts[(i + 1) % n]
            start = corners[i][3] if i in corners else p
            end = corners[(i + 1) % n][2] if (i + 1) % n in corners else p_next
            if i in corners:
                _, _, a_s, a_e, ctr, r, th0, th1 = corners[i]
                pieces.append(("arc", ctr, r, th0, th1))
            pieces.append(("line", np.array(start, float), np.array(end, float)))

        # sample uniformly in arc length
        xs, ys, hs = [], [], []
        for pc in pieces:
            if pc[0] == "line":
                a, b = pc[1], pc[2]
                L = float(np.linalg.norm(b - a))
                if L < 1e-9:
                    continue
                k = max(1, int(L / ds))
                h = math.atan2(b[1] - a[1], b[0] - a[0])
                for j in range(k):
                    f = j / k
                    xs.append(a[0] + (b[0] - a[0]) * f)
                    ys.append(a[1] + (b[1] - a[1]) * f)
                    hs.append(h)
            else:
                _, ctr, r, th0, th1 = pc
                L = abs(th1 - th0) * r
                k = max(1, int(L / ds))
                sgn = 1 if th1 > th0 else -1
                for j in range(k):
                    th = th0 + (th1 - th0) * j / k
                    xs.append(ctr[0] + r * math.cos(th))
                    ys.append(ctr[1] + r * math.sin(th))
                    hs.append(th + sgn * math.pi / 2)
        if not closed:                      # include the final endpoint
            b = self.pts[-1]
            xs.append(float(b[0])); ys.append(float(b[1])); hs.append(hs[-1])
        self.x = np.array(xs)
        self.y = np.array(ys)
        self.h = np.unwrap(np.array(hs))
        if closed:
            d = np.hypot(np.diff(self.x, append=self.x[0]),
                         np.diff(self.y, append=self.y[0]))
        else:
            d = np.hypot(np.diff(self.x, append=self.x[-1]),
                         np.diff(self.y, append=self.y[-1]))
        self.s = np.concatenate([[0.0], np.cumsum(d)[:-1]])
        self.total = float(self.s[-1] + d[-1])
        # curvature from the heading rate along arc length
        dh = np.abs(np.diff(self.h, append=(self.h[0] + 2 * math.pi) if closed
                            else self.h[-1]))
        dh = np.minimum(dh, 2 * math.pi - dh)
        self.kappa = dh / np.maximum(d, 1e-6)


class MovingCar:
    """Spawns the car and drives it round the circuit.

    `update(t)` is one RPC and is called at reduced rate by the flight loop.
    Position is a pure function of elapsed time, so a skipped update introduces
    no drift and the ground truth used for scoring is always exactly where the
    car is.
    """

    def __init__(self, world, speed_mps: float = 3.0,
                 route: Optional[List[Tuple[float, float]]] = None,
                 name: str = "SemCar", spec: Optional[CarSpec] = None,
                 corner_r: float = 6.0, lat_acc: float = 1.6,
                 lon_acc: float = 1.2, phase_s: float = 10.0,
                 one_shot: bool = False,
                 stops: Optional[List[Tuple[float, float]]] = None):
        self.world = world
        self.spec = spec or CarSpec()
        self.route = list(route or DEFAULT_ROUTE)
        self.speed = float(speed_mps)
        self.name = name
        self.actual_name: Optional[str] = None
        self.material_used: Optional[str] = None
        self.path = _Path(self.route, corner_r=corner_r, closed=not one_shot)
        # (fraction along the route, seconds to wait there). Stopping and
        # pulling away again is the clearest evidence that the aircraft is
        # tracking the car rather than just flying down the same street: a
        # follower must stop too, and start again when the car does.
        self.stops = list(stops or [])
        self._build_speed_profile(lat_acc, lon_acc)
        # Where on the lap the car is at t=0. It must not begin directly beneath
        # the drone: the front camera is pitched 20 deg down with a 29.4 deg
        # vertical half-FOV, so anything closer than ~0.86 x altitude is under
        # the field of view entirely. A first attempt started the car 4 m from
        # the spawn and the detector never saw it once in 132 s.
        self.phase_s = float(phase_s)
        self.one_shot = bool(one_shot)
        x, y, h = self.pose_at(0.0)
        self.pos, self.heading = (x, y), h
        # Last pose actually sent to the sim, so a no-op teleport can be skipped.
        # See update() for why that mattered so much.
        self._last_sent: Optional[Tuple[float, float, float]] = None
        self._parked_ticks = 0

    def _build_speed_profile(self, lat_acc: float, lon_acc: float) -> None:
        """Cap speed by cornering grip, then by how hard it can brake and pull.

        Without this the car would hold a constant speed through the corners,
        which looks wrong and makes the target's apparent size jump as it swings
        through the turn.
        """
        p = self.path
        v = np.minimum(self.speed, np.sqrt(lat_acc / np.maximum(p.kappa, 1e-6)))
        ds = np.diff(p.s, append=p.total)
        if not self.path.closed:
            # start from rest and come to rest, so the car pulls away smoothly
            # and brakes to a stop instead of vanishing at the end
            v[0] = v[-1] = 0.0
        stop_idx = []
        for frac, _dwell in getattr(self, "stops", []):
            i = int(np.clip(round(frac * (len(v) - 1)), 1, len(v) - 2))
            v[i] = 0.0                      # the fwd/bwd pass shapes the ramps
            stop_idx.append(i)
        # two passes so a closed profile wraps consistently
        for _ in range(2):
            for i in range(len(v) - (0 if self.path.closed else 1)):
                j = (i + 1) % len(v)
                v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * lon_acc * ds[i]))
            for i in range(len(v) - 1, 0 if self.path.closed else 0, -1):
                j = (i - 1) % len(v)
                v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * lon_acc * ds[j]))
        self.v = np.maximum(v, 0.4)
        # Time to reach each sample, integrating ds / v. The floor matters: the
        # profile legitimately reaches zero at a standing start, but dt = ds/v
        # then diverges and the car sits on the line for seconds. 0.6 m/s keeps
        # the pull-away brief without distorting the rest of the profile.
        dt = ds / np.maximum(self.v, 0.6)
        for i, (_frac, dwell) in zip(stop_idx, getattr(self, "stops", [])):
            dt[i] += float(dwell)           # sit still at the stop
        self.t = np.concatenate([[0.0], np.cumsum(dt)[:-1]])
        self.lap_time = float(self.t[-1] + dt[-1])
        self.stop_idx = stop_idx

    def pose_at(self, t: float) -> Tuple[float, float, float]:
        """(x, y, heading) at elapsed time t.

        With `one_shot`, the car drives its route once and parks at the end
        instead of looping. A closed circuit looks natural but its turns swing
        the car through the aircraft's blind spot — the front camera cannot see
        anything closer than 0.86 x altitude — and every lost lock cost a chunk
        of the flight. Driving straight and stopping keeps the target in view for
        the whole run.
        """
        if getattr(self, "one_shot", False):
            tt = min(t + getattr(self, "phase_s", 0.0), self.lap_time - 1e-3)
        else:
            tt = (t + getattr(self, "phase_s", 0.0)) % self.lap_time
        i = int(np.searchsorted(self.t, tt, side="right") - 1)
        i = max(0, min(i, len(self.t) - 1))
        j = (i + 1) % len(self.t)
        t0 = self.t[i]
        t1 = self.t[j] if j > i else self.lap_time
        f = 0.0 if t1 <= t0 else (tt - t0) / (t1 - t0)
        p = self.path
        x = p.x[i] + (p.x[j] - p.x[i]) * f
        y = p.y[i] + (p.y[j] - p.y[i]) * f
        # Heading is stored unwrapped, so it climbs by 2*pi over a lap. At the
        # seam (j wraps to 0) the raw difference is -2*pi, which would render as
        # a 175-degree flip in one frame — exactly the discontinuity this class
        # exists to remove. Take the short way round instead.
        dh = p.h[j] - p.h[i]
        dh = (dh + math.pi) % (2 * math.pi) - math.pi
        h = p.h[i] + dh * f
        return float(x), float(y), float(h)

    def speed_at(self, t: float) -> float:
        if getattr(self, "one_shot", False):
            tt = min(t + getattr(self, "phase_s", 0.0), self.lap_time - 1e-3)
        else:
            tt = (t + getattr(self, "phase_s", 0.0)) % self.lap_time
        i = int(np.searchsorted(self.t, tt, side="right") - 1)
        return float(self.v[max(0, min(i, len(self.v) - 1))])

    # ------------------------------------------------------------------ sim --

    def _pose(self, x: float, y: float, h: float):
        from projectairsim.types import Pose, Quaternion, Vector3
        from projectairsim.utils import rpy_to_quaternion
        if self.spec.glb_path:
            h = h + math.radians(self.spec.glb_yaw_offset_deg)
        w, qx, qy, qz = rpy_to_quaternion(0.0, 0.0, h)
        # mesh origin sits at road level, so no half-height offset
        return Pose({
            "translation": Vector3({"x": x, "y": y, "z": self.spec.ground_z_ned}),
            "rotation": Quaternion({"w": w, "x": qx, "y": qy, "z": qz}),
            "frame_id": "DEFAULT_ID",
        })

    def spawn(self) -> str:
        x, y, h = self.pose_at(0.0)
        if self.spec.glb_path:
            # The glTF carries its own colour, so no material is applied and
            # `material_used` deliberately stays None. Callers that treat "no
            # material" as a paint failure must check `glb_path` first --
            # Traffic.spawn() does.
            import pathlib as _pl
            data = _pl.Path(self.spec.glb_path).read_bytes()
            s = float(self.spec.glb_scale)
            self.actual_name = self.world.spawn_object_from_file(
                self.name, "gltf", data, True, self._pose(x, y, h), [s, s, s], False)
            print(f"[car] spawned {self.actual_name!r} from "
                  f"{_pl.Path(self.spec.glb_path).name} x{s:g} at ({x:.1f}, {y:.1f})")
        else:
            scale = ([1.0, 1.0, 1.0] if self.spec.unit_scale
                     else [self.spec.length_m, self.spec.width_m, self.spec.height_m])
            self.actual_name = self.world.spawn_object(
                self.name, self.spec.asset, self._pose(x, y, h), scale, False)
            print(f"[car] spawned {self.actual_name!r} "
                  f"{self.spec.length_m}x{self.spec.width_m}x{self.spec.height_m} m "
                  f"at ({x:.1f}, {y:.1f})")
        print(f"[car] circuit {self.path.total:.0f} m, lap {self.lap_time:.0f}s, "
              f"speed {self.v.min():.1f}-{self.v.max():.1f} m/s")
        for mat in (() if self.spec.glb_path else self.spec.materials):
            try:
                self.world.set_object_material(self.actual_name, mat)
                self.material_used = mat
                print(f"[car] material -> {mat}")
                break
            except Exception as e:
                print(f"[car] material {mat} failed ({type(e).__name__})")
        if self.material_used is None and not self.spec.glb_path:
            print("[car] WARNING: no material applied — the colour word in the "
                  "instruction will not match what the camera sees")
        self.pos, self.heading = (x, y), h
        self._fail_streak = 0
        self._last_sent = None
        self._parked_ticks = 0
        return self.actual_name

    def update(self, t: float) -> Tuple[float, float]:
        x, y, h = self.pose_at(t)
        self.pos, self.heading = (x, y), h
        if self.actual_name is None:
            return x, y

        # Do not re-send a pose the object is already at.
        #
        # This was the whole of the "SetObjectPose ... not movable" mystery. With
        # one_shot the car parks at the end of its route, pose_at clamps, and
        # update() then asks the sim to teleport it to where it already is on
        # every remaining tick. The sim refuses a no-op teleport and reports it as
        # "check if object state is movable!", which reads like actor corruption
        # and was diagnosed as such for weeks.
        #
        # It is not. Across five flights the fault fired at EXACTLY (38.0, 58.0)
        # every time - the last point of STRAIGHT_ROUTE - and the time it fired
        # tracked the route length plus the stop dwell: 34.4 s with no stops,
        # 48.5 s with two 6 s stops. Nothing degrades and nothing is corrupt; the
        # car has simply arrived.
        moved = (abs(x - self._last_sent[0]) > 1e-4
                 or abs(y - self._last_sent[1]) > 1e-4
                 or abs(h - self._last_sent[2]) > 1e-4) if self._last_sent else True
        if not moved:
            self._parked_ticks += 1
            return x, y

        try:
            self.world.set_object_pose(self.actual_name, self._pose(x, y, h), True)
            self._fail_streak = 0
            self._last_sent = (x, y, h)
        except Exception as e:
            # A real failure now means a real failure: the car was asked to move
            # somewhere new and could not.
            self._fail_streak = getattr(self, "_fail_streak", 0) + 1
            if self._fail_streak in (1, 10):
                print(f"[car] teleport failed ({type(e).__name__}: {e})")
            if self._fail_streak == 25:
                print("[car] *** THE CAR IS NOT MOVING - respawning it ***")
                try:
                    self.world.destroy_object(self.actual_name)
                except Exception:
                    pass
                self.actual_name = None
                try:
                    self.spawn()
                    self._fail_streak = 0
                except Exception as e2:
                    print(f"[car] respawn failed ({type(e2).__name__}: {e2})")
        return x, y

    def destroy(self) -> None:
        if self.actual_name:
            try:
                self.world.destroy_object(self.actual_name)
                print(f"[car] destroyed {self.actual_name!r}")
            except Exception as e:
                print(f"[car] destroy failed ({type(e).__name__})")
            self.actual_name = None

    def truth(self) -> dict:
        import pathlib as _pl
        return {"name": self.actual_name, "asset": self.spec.asset,
                "glb": None if not self.spec.glb_path
                       else _pl.Path(self.spec.glb_path).name,
                "size_m": [self.spec.length_m, self.spec.width_m, self.spec.height_m],
                "material": self.material_used, "speed_mps": self.speed,
                "route": self.route, "circuit_m": round(self.path.total, 1),
                "lap_s": round(self.lap_time, 1), "phase_s": self.phase_s,
                "speed_range": [round(float(self.v.min()), 2),
                                round(float(self.v.max()), 2)],
                "desc_match": self.spec.desc_match,
                "desc_mismatch": self.spec.desc_mismatch,
                "parked_ticks": self._parked_ticks}
