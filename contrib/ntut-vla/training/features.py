"""
Shared feature encoding for the behavior-cloning policy.

One place defines how a (state, mission, policy) situation becomes the input
vector — the dataset generator and the runtime BCVLA must never drift apart,
so they both import from here.

Feature vector (8 floats, all roughly in [-1, 1]):
    0  dx      target offset North, /50 m
    1  dy      target offset East,  /50 m
    2  dalt    (cruise_alt - up), /10 m
    3  nx      vector to nearest NFZ boundary point, North, /30 m  (0 if no NFZ)
    4  ny      same, East, /30 m
    5  ndist   distance to nearest NFZ boundary, /30 m  (1.0 if no NFZ)
    6  inside  1.0 if currently inside an NFZ (incl. margin), else 0.0
    7  up      altitude, /30 m

Label vector (3 floats): the shield-corrected action (vx, vy, vz_up) / 4.0.
Yaw rate is not learned in v1 (stub never yaws).
"""
from __future__ import annotations

from shapely.geometry import Point

from guardrail.geometry import fence_polygon, point_in_fence
from guardrail.models import Policy, PolygonFence, State

ACTION_SCALE = 4.0     # divisor for labels; matches the policy speed cap


def decode_action(y0: float, y1: float, y2: float,
                  up: float | None = None,
                  alt_min: float | None = None,
                  alt_max: float | None = None):
    """Model head -> physical action, with action-space bounds ENFORCED.
    tanh outputs are per-axis, so (vx, vy) combinations can exceed the
    horizontal speed cap by up to sqrt(2) — which made the Shield's
    SpeedClamp fire on almost every cruise tick. Clamping in the decoder is
    part of the model contract (same as any bounded action head).

    Altitude-aware vz (when up/alt band given): clamp vz so the 3 s forecast
    stays inside the band with 0.2 m headroom. Diagnosis showed 100% of the
    residual shield interventions were AltitudeFix — regression noise of
    ±0.3 m/s in vz, forecast over 3 s, grazes the band edge whenever cruise
    sits near it. Same rule as the Shield's own AltitudeFix, moved into the
    action head so the Shield no longer needs to fire."""
    vx, vy, vz = y0 * ACTION_SCALE, y1 * ACTION_SCALE, y2 * ACTION_SCALE
    h = (vx * vx + vy * vy) ** 0.5
    if h > 3.8:
        s = 3.8 / h
        vx, vy = vx * s, vy * s
    vz = max(-1.9, min(1.9, vz))
    if up is not None and alt_min is not None and alt_max is not None:
        look = 3.0
        vz = min(vz, (alt_max - 0.2 - up) / look)
        vz = max(vz, (alt_min + 0.2 - up) / look)
    return vx, vy, vz


def make_fences(policy: Policy):
    """Pre-build (fence, polygon) pairs once — same trick the Shield uses."""
    return [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]


def encode(state: State, target_x: float, target_y: float, cruise_alt: float,
           fences) -> list[float]:
    dx = (target_x - state.x) / 50.0
    dy = (target_y - state.y) / 50.0
    dalt = (cruise_alt - state.up) / 10.0

    nx = ny = 0.0
    ndist = 1.0
    inside = 0.0
    best = None
    p = Point(state.x, state.y)
    for f, poly in fences:
        d = poly.exterior.distance(p)
        if best is None or d < best[0]:
            best = (d, f, poly)
    if best is not None:
        d, f, poly = best
        nearest = poly.exterior.interpolate(poly.exterior.project(p))
        nx = (nearest.x - state.x) / 30.0
        ny = (nearest.y - state.y) / 30.0
        ndist = min(d / 30.0, 1.0)
        inside = 1.0 if point_in_fence(state.x, state.y, state.up, f, poly) else 0.0

    return [dx, dy, dalt, nx, ny, ndist, inside, state.up / 30.0]


# ---------------------------------------------------------------------------
# v2 encoding (auto-research pipeline): v1's 8 features + the PREVIOUS action.
# Giving the model its own last action lets it learn smooth, stateful behaviour
# (and to hold still at the target — impossible to express with v1 features,
# which is why the v1 model orbited).
# ---------------------------------------------------------------------------
FEAT_V2_DIM = 11


def encode_v2(state: State, target_x: float, target_y: float, cruise_alt: float,
              fences, prev_action: tuple[float, float, float]) -> list[float]:
    base = encode(state, target_x, target_y, cruise_alt, fences)
    return base + [prev_action[0] / ACTION_SCALE,
                   prev_action[1] / ACTION_SCALE,
                   prev_action[2] / ACTION_SCALE]


# ---------------------------------------------------------------------------
# v3 encoding: v2 + FULL zone geometry. The v2 nearest-point encoding was too
# poor for the student to decide WHICH WAY around a zone (and it was blind to
# any second zone) — run #2 plateaued at reach ~80% / interventions ~45%.
# Up to 3 zone slots (2 static + 1 dynamic max in the scenario factory),
# nearest-first; empty slots = zeros with active=0.
# Per zone: [center_dx/30, center_dy/30, half_x/30, half_y/30, active]
# Total: 8 (base) + 3 (prev action) + 15 (zones) = 26
# ---------------------------------------------------------------------------
FEAT_V3_DIM = 26


def encode_v3(state: State, target_x: float, target_y: float, cruise_alt: float,
              fences, prev_action: tuple[float, float, float]) -> list[float]:
    feats = encode_v2(state, target_x, target_y, cruise_alt, fences, prev_action)

    p = Point(state.x, state.y)
    zones = []
    for f, poly in fences:
        minx, miny, maxx, maxy = poly.bounds
        cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
        hx, hy = (maxx - minx) / 2, (maxy - miny) / 2
        zones.append((poly.exterior.distance(p),
                      [(cx - state.x) / 30.0, (cy - state.y) / 30.0,
                       hx / 30.0, hy / 30.0, 1.0]))
    zones.sort(key=lambda z: z[0])
    for i in range(3):
        feats += zones[i][1] if i < len(zones) else [0.0, 0.0, 0.0, 0.0, 0.0]
    return feats
