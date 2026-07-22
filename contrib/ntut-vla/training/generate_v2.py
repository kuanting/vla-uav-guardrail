"""
Dataset generator v2 — offline rollouts, PARALLEL across CPU cores.

Fixes over v1 (each one earned by a bug):
    1. HOVER samples after reaching the target (v1 model orbited — it had
       never seen "stop").
    2. Near-target oversampling x3 (sparse-data region).
    3. Multi-NFZ + dynamic-NFZ scenarios (shared factory: scenarios.py).
    4. prev-action feature (v2 encoding) for smooth stateful behaviour.
    5. DAgger mode: the CURRENT model flies, the Shield corrects, corrections
       become labels — fixes BC distribution shift.

Parallelism: episodes are sharded over a process pool (Windows spawn-safe).
DAgger workers each load the TorchScript model from disk once.
"""
from __future__ import annotations

import argparse
import random
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.models import Action4D, State                     # noqa: E402
from guardrail.shield import Shield                              # noqa: E402
from training.features import ACTION_SCALE, decode_action, encode_v3, make_fences  # noqa: E402
from training.planner_expert import PlannerExpert               # noqa: E402
from training.scenarios import make_scenario                     # noqa: E402

DT = 0.1
MAX_TICKS = 700
REACH_M = 2.0
HOVER_TICKS = 20
NEAR_M = 8.0
NEAR_DUP = 3
TRAIN_MARGIN = 4.0   # teacher keeps a wider berth than the deploy margin (2.5)

_worker_model = None      # per-process DAgger model cache


def stub_pilot(state, tx, ty, cruise, speed):
    dx, dy = tx - state.x, ty - state.y
    d = (dx * dx + dy * dy) ** 0.5
    if d < 1e-6:
        return Action4D()
    v = min(speed, d)
    return Action4D(vx=dx / d * v, vy=dy / d * v, vz_up=0.8 * (cruise - state.up))


def expert_pilot(state, tx, ty, cruise, fences, speed=4.0):
    """Deterministic potential-field pilot — the LABEL source (run #4 fix).

    Run #3 finding: labels from stub+shield are MULTIMODAL near head-on
    approaches (slide side flips with tiny angle changes); MSE regression
    averages "go left" and "go right" into "fly straight in" — NFZ entry was
    pinned at exactly 9.5% three iterations running. This pilot is a smooth,
    DETERMINISTIC function of state: attraction to target + repulsion from
    zones + a FIXED clockwise circulation that breaks the symmetry the same
    way every time. Single-mode labels -> learnable. Shield still filters
    every action, so labels stay provably safe.
    """
    from shapely.geometry import LineString, Point
    dx, dy = tx - state.x, ty - state.y
    dist = (dx * dx + dy * dy) ** 0.5
    if dist < 1e-6:
        return Action4D()
    ux, uy = dx / dist, dy / dist
    ray = LineString([(state.x, state.y), (tx, ty)])

    rx = ry = cx_ = cy_ = 0.0
    p = Point(state.x, state.y)
    for f, poly in fences:
        d = poly.exterior.distance(p)
        inside = poly.contains(p)
        nearest = poly.exterior.interpolate(poly.exterior.project(p))
        ax, ay = state.x - nearest.x, state.y - nearest.y
        n = (ax * ax + ay * ay) ** 0.5 or 1.0
        ax, ay = ax / n, ay / n
        if inside:
            ax, ay = -ax, -ay
            w = 3.0
        elif d < 8.0:
            w = (8.0 - d) / 8.0
        else:
            continue
        rx += ax * w * 2.0
        ry += ay * w * 2.0
        # clockwise circulation ONLY while this zone actually blocks the
        # straight ray to the target — enough to route around, no orbiting.
        if inside or ray.intersects(poly.buffer(1.5)):
            cx_ += ay * w * 1.4
            cy_ += -ax * w * 1.4

    vx, vy = ux + rx + cx_, uy + ry + cy_
    n = (vx * vx + vy * vy) ** 0.5 or 1.0
    v = min(speed, dist)
    return Action4D(vx=vx / n * v, vy=vy / n * v,
                    vz_up=0.8 * (cruise - state.up))


def _get_model(path: str):
    global _worker_model
    if _worker_model is None:
        import torch
        torch.set_num_threads(1)          # 1 thread per worker — no oversubscription
        _worker_model = torch.jit.load(path)
        _worker_model.eval()
    return _worker_model


def rollout(rng: random.Random, model_path: str | None):
    policy, state, tx, ty, cruise, dyn, dyn_at = make_scenario(rng, margin=TRAIN_MARGIN, hard=True)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    fences = make_fences(policy)
    expert_planner = PlannerExpert(fences)
    speed = rng.uniform(4.0, 7.0)
    prev = (0.0, 0.0, 0.0)

    X, Y, near = [], [], []
    reached = False
    for tick in range(MAX_TICKS):
        if dyn is not None and tick >= dyn_at:
            from training.scenarios import spawn_clear
            if spawn_clear(dyn, state):
                shield.hot_apply(dyn)
                fences = make_fences(policy)
                expert_planner.refresh(fences)
                dyn = None                     # spawned; stop checking

        dx, dy = tx - state.x, ty - state.y
        dist = (dx * dx + dy * dy) ** 0.5
        if dist < REACH_M:
            reached = True
            break

        feats = encode_v3(state, tx, ty, cruise, fences, prev)

        # LABEL is always the EXPERT at this state (DAgger only changes which
        # states get visited). Expert = deterministic potential-field pilot,
        # shield-filtered — single-mode AND provably safe labels.
        expert = shield.filter(
            state, expert_planner.act(state, tx, ty, cruise)).emitted
        X.append(feats)
        Y.append([expert.vx / ACTION_SCALE, expert.vy / ACTION_SCALE,
                  expert.vz_up / ACTION_SCALE])
        # oversample the two safety-critical regions: near the target (stop
        # behaviour) and near a zone boundary (avoidance behaviour)
        near.append(dist < NEAR_M or feats[5] < 0.2)

        if model_path is None:
            exec_act = expert
        else:
            import torch
            m = _get_model(model_path)
            with torch.no_grad():
                y = m(torch.tensor([feats], dtype=torch.float32))[0]
            dvx, dvy, dvz = decode_action(float(y[0]), float(y[1]), float(y[2]))
            raw = Action4D(vx=dvx, vy=dvy, vz_up=dvz)
            exec_act = shield.filter(state, raw).emitted   # model drives (safely)

        prev = (exec_act.vx, exec_act.vy, exec_act.vz_up)
        state = State(x=state.x + exec_act.vx * DT, y=state.y + exec_act.vy * DT,
                      up=state.up + exec_act.vz_up * DT)

    if reached:
        for _ in range(HOVER_TICKS):
            feats = encode_v3(state, tx, ty, cruise, fences, prev)
            X.append(feats)
            Y.append([0.0, 0.0, 0.0])
            near.append(True)
            prev = (0.0, 0.0, 0.0)
    return X, Y, near, reached


def _chunk(args):
    """Worker: run a shard of episodes. args = (n, seed, dagger_path, dagger_frac)."""
    n, seed, dagger_path, dagger_frac = args
    rng = random.Random(seed)
    X, Y = [], []
    reached = 0
    for _ in range(n):
        use_model = dagger_path is not None and rng.random() < dagger_frac
        ex, ey, near, r = rollout(rng, dagger_path if use_model else None)
        reached += int(r)
        if not r and not use_model:
            continue        # don't teach from teacher dead-ends (rare local minima)
        for x, y, nr in zip(ex, ey, near):
            for _ in range(NEAR_DUP if nr else 1):
                X.append(x)
                Y.append(y)
    return X, Y, reached


def generate(episodes: int, seed: int, dagger_path: str | None = None,
             dagger_frac: float = 0.0, workers: int = 12,
             progress=None):
    """Parallel generation. progress = optional callback(done_eps, total_eps, samples)."""
    shard = max(50, episodes // (workers * 4))
    jobs = []
    left, s = episodes, seed
    while left > 0:
        n = min(shard, left)
        jobs.append((n, s, dagger_path, dagger_frac))
        left -= n
        s += 1

    allX, allY = [], []
    reached = done = 0
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futs = {pool.submit(_chunk, j): j[0] for j in jobs}
        for fut in as_completed(futs):
            X, Y, r = fut.result()
            allX.extend(X)
            allY.extend(Y)
            reached += r
            done += futs[fut]
            if progress:
                progress(done, episodes, len(allX))
    return (np.array(allX, dtype=np.float32), np.array(allY, dtype=np.float32),
            reached / episodes)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=3000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=str(ROOT / "training" / "data" / "bc_v2.npz"))
    args = ap.parse_args()
    X, Y, rr = generate(args.episodes, args.seed,
                        progress=lambda d, t, s: print(f"  {d}/{t} eps, {s} samples"))
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, X=X, Y=Y)
    print(f"done: {len(X)} samples, teacher reach {rr:.1%} -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
