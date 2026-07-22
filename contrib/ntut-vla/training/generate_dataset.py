"""
Dataset generator — offline rollouts, no simulator needed.

Key idea: the Shield, the stub pilot, and simple kinematics (pos += v * dt)
are all pure Python, so we can fly thousands of missions per minute at
1000x real time. Each tick records:

    features(state, mission, NFZs)  ->  the SHIELD-CORRECTED action

Training on the corrected action means the student model learns to fly the
way the guardrail *wants* — "learning manners from the fence". A model that
learns well will need fewer runtime repairs (grant: lower mean repair
magnitude), which is exactly the Prefix-Compiler hypothesis, demonstrated
through learning.

Run (vla-drone env):
    python training/generate_dataset.py --episodes 600 --out training/data/bc_dataset.npz
"""
from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.models import (                                   # noqa: E402
    Action4D, AltitudeEnvelope, KinematicEnvelope, Policy, PolygonFence,
    State, XY,
)
from guardrail.shield import Shield                              # noqa: E402
from training.features import ACTION_SCALE, encode, make_fences  # noqa: E402

DT = 0.1
MAX_TICKS = 700          # 70 s cap per episode
REACH_M = 2.0


def random_policy(rng: random.Random) -> Policy:
    """Random scenario: alt band 15-25, speed cap 4, and (usually) one square
    NFZ dropped somewhere near the direct path."""
    constraints = [
        AltitudeEnvelope(id="alt-band", type="altitude_envelope",
                         alt_min_m=15, alt_max_m=25),
        KinematicEnvelope(id="kin", type="kinematic_envelope",
                          speed_max_mps=4.0, climb_rate_max_mps=2.0,
                          yaw_rate_max_dps=45.0),
    ]
    if rng.random() > 0.3:                       # 70% of episodes have an NFZ
        cx = rng.uniform(8, 30)
        cy = rng.uniform(8, 30)
        half = rng.uniform(5, 10)
        constraints.append(PolygonFence(
            id="nfz", type="polygon_fence",
            vertices=[XY(x=cx - half, y=cy - half), XY(x=cx + half, y=cy - half),
                      XY(x=cx + half, y=cy + half), XY(x=cx - half, y=cy + half)],
            altitude_floor_m=0, altitude_ceiling_m=100, margin_m=1.0,
        ))
    return Policy(policy_id="bc-train", constraints=constraints)


def stub_pilot(state: State, tx: float, ty: float, cruise: float,
               speed: float) -> Action4D:
    """Same shape as guardrail.vla_stub — reckless straight-line pilot."""
    dx, dy = tx - state.x, ty - state.y
    dist = (dx * dx + dy * dy) ** 0.5
    if dist < 1e-6:
        return Action4D()
    v = min(speed, dist)
    return Action4D(vx=dx / dist * v, vy=dy / dist * v,
                    vz_up=0.8 * (cruise - state.up))


def rollout(rng: random.Random):
    policy = random_policy(rng)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    fences = make_fences(policy)

    # random mission: start near origin, target on a ring 25-45 m out
    ang = rng.uniform(0, 6.283)
    r = rng.uniform(25, 45)
    tx, ty = r * np.cos(ang), r * np.sin(ang)
    cruise = rng.uniform(16, 24)
    vla_speed = rng.uniform(4.0, 7.0)            # sometimes legal, often not
    state = State(x=0.0, y=0.0, up=rng.uniform(15.5, 24.5))

    X, Y = [], []
    reached = False
    for _ in range(MAX_TICKS):
        if ((tx - state.x) ** 2 + (ty - state.y) ** 2) ** 0.5 < REACH_M:
            reached = True
            break
        raw = stub_pilot(state, tx, ty, cruise, vla_speed)
        emitted = shield.filter(state, raw).emitted
        X.append(encode(state, tx, ty, cruise, fences))
        Y.append([emitted.vx / ACTION_SCALE, emitted.vy / ACTION_SCALE,
                  emitted.vz_up / ACTION_SCALE])
        state = State(x=state.x + emitted.vx * DT,
                      y=state.y + emitted.vy * DT,
                      up=state.up + emitted.vz_up * DT)
    return X, Y, reached


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=600)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default=str(ROOT / "training" / "data" / "bc_dataset.npz"))
    args = ap.parse_args()

    rng = random.Random(args.seed)
    allX, allY = [], []
    n_reached = 0
    for ep in range(args.episodes):
        X, Y, reached = rollout(rng)
        allX.extend(X)
        allY.extend(Y)
        n_reached += int(reached)
        if (ep + 1) % 100 == 0:
            print(f"episode {ep + 1}/{args.episodes}  samples={len(allX)}")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, X=np.array(allX, dtype=np.float32),
                        Y=np.array(allY, dtype=np.float32))
    print(f"done: {len(allX)} samples from {args.episodes} episodes "
          f"({n_reached} reached target) -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
