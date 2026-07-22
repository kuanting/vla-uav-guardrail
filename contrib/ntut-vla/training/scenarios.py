"""
Scenario factory shared by the v2 dataset generator and the closed-loop
evaluator. One definition of "a random mission" so train and eval can only
differ by SEED, never by distribution (held-out eval = different seed).
"""
from __future__ import annotations

import random

import numpy as np

from guardrail.models import (
    AltitudeEnvelope, KinematicEnvelope, Policy, PolygonFence, State, XY,
)


def make_policy(rng: random.Random, n_nfz: int, margin: float = 2.5) -> Policy:
    constraints = [
        AltitudeEnvelope(id="alt-band", type="altitude_envelope",
                         alt_min_m=15, alt_max_m=25),
        KinematicEnvelope(id="kin", type="kinematic_envelope",
                          speed_max_mps=4.0, climb_rate_max_mps=2.0,
                          yaw_rate_max_dps=45.0),
    ]
    for i in range(n_nfz):
        cx = rng.uniform(6, 32)
        cy = rng.uniform(6, 32)
        hx = rng.uniform(4, 11)
        hy = rng.uniform(4, 11)
        constraints.append(PolygonFence(
            id=f"nfz-{i}", type="polygon_fence",
            vertices=[XY(x=cx - hx, y=cy - hy), XY(x=cx + hx, y=cy - hy),
                      XY(x=cx + hx, y=cy + hy), XY(x=cx - hx, y=cy + hy)],
            # 2.5 m margin: the teacher keeps a WIDE berth so the student's
            # regression error (~0.5 m/s) still clears the real zone.
            altitude_floor_m=0, altitude_ceiling_m=100, margin_m=margin,
        ))
    return Policy(policy_id="auto-research", constraints=constraints)


def spawn_clear(fence: PolygonFence, state: State, clearance: float = 6.0) -> bool:
    """May a dynamic zone activate now? Only if the vehicle is outside its
    margin ring + clearance. Real no-fly zones don't materialize on top of an
    aircraft — without this rule ~6% of eval episodes were auto-fails.

    Clearance 6 m (was 2): at cruise 3.8 m/s, 2 m gave the pilot ~1.2 s to
    react — diagnosed as the cause of 2 of the 3 residual entries. 6 m ≈ 2.4 s
    of warning, in line with real NFZ activation notice."""
    return _clear_of(fence, state.x, state.y, clearance)


def _clear_of(fence: PolygonFence, x: float, y: float, clearance: float) -> bool:
    """Point at least `clearance` outside the fence's margin ring?"""
    from shapely.geometry import Point
    from guardrail.geometry import fence_polygon
    return fence_polygon(fence).distance(Point(x, y)) >= fence.margin_m + clearance


def make_scenario(rng: random.Random, margin: float = 2.5, hard: bool = False):
    """Returns (policy, start_state, target_x, target_y, cruise_alt,
    dynamic_fence_or_None, dynamic_at_tick).

    Guarantees WINNABLE missions: start and target keep >= 3 m clearance
    outside every zone's margin ring. (Found the hard way: without this,
    ~10% of scenarios buried the target inside a zone — unreachable for any
    rule-respecting pilot, which silently capped every teacher/student score.)
    """
    ang = rng.uniform(0, 6.283)
    dist = rng.uniform(25, 45)
    tx, ty = float(dist * np.cos(ang)), float(dist * np.sin(ang))
    cruise = rng.uniform(16, 24)
    start = State(x=0.0, y=0.0, up=rng.uniform(15.5, 24.5))

    r = rng.random()
    if hard:
        # training-only curriculum: more multi-zone and (below) more dynamic
        # events than the fixed benchmark — domain randomization beyond eval
        n_nfz = 0 if r < 0.1 else (1 if r < 0.5 else 2)
    else:
        n_nfz = 0 if r < 0.2 else (1 if r < 0.7 else 2)
    for _ in range(60):                          # rejection-sample legal zones
        policy = make_policy(rng, n_nfz, margin)
        fences = [c for c in policy.constraints if c.type == "polygon_fence"]
        if all(_clear_of(f, tx, ty, 3.0) and _clear_of(f, 0.0, 0.0, 3.0)
               for f in fences):
            break
    else:
        policy = make_policy(rng, 0, margin)     # give up on zones, keep mission

    dyn = None
    dyn_at = -1
    if rng.random() < (0.4 if hard else 0.25):
        for _ in range(30):
            f = rng.uniform(0.4, 0.7)
            cx, cy = tx * f, ty * f
            half = rng.uniform(4, 8)
            cand = PolygonFence(
                id="nfz-dyn", type="polygon_fence",
                vertices=[XY(x=cx - half, y=cy - half), XY(x=cx + half, y=cy - half),
                          XY(x=cx + half, y=cy + half), XY(x=cx - half, y=cy + half)],
                altitude_floor_m=0, altitude_ceiling_m=100, margin_m=margin)
            if _clear_of(cand, tx, ty, 3.0) and _clear_of(cand, 0.0, 0.0, 3.0):
                dyn = cand
                dyn_at = rng.randint(30, 120)
                break
    return policy, start, tx, ty, cruise, dyn, dyn_at
