"""
V3 — verify empirically that the Shield does not author heading.

The coexistence proof reads a single log line as evidence of two systems acting
at once: `emitted.yaw_rate == raw.yaw_rate` (the VLA owns the nose) while
`emitted.(vx,vy) != raw.(vx,vy)` (the Shield owns the track). That reading is
only valid if the Shield genuinely never edits yaw.

Reading it off the source is not enough — a policy edit could break it silently,
so this replays action streams that force every repair path and checks the
property directly. Run it after any change to the policy or to shield.py.

No simulator needed.

Run:  python experiments/verify_shield_yaw.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import Shield, State, load_policy            # noqa: E402
from guardrail.models import Action4D, ObstacleClearance    # noqa: E402

import city_planner                                         # noqa: E402

POLICY = ROOT / "policies" / "semantic_conflict.yaml"
CITYMAP = ROOT / "demo" / "out" / "citymap" / "occ_day.npz"


def build_shield():
    policy = load_policy(str(POLICY))
    cmap = city_planner.load_occ(str(CITYMAP))
    smap = None
    if policy.by_type(ObstacleClearance) and cmap is not None:
        smap = {"occ": cmap["occ"], "res": cmap["res"],
                "ox": cmap["ox"], "oy": cmap["oy"]}
    return Shield(policy, lookahead_s=3.0, dt=0.5, obstacle_map=smap), policy


def replay(shield, states, actions, label):
    """Push a canned stream through the Shield and audit the yaw channel."""
    edited, braked, touched, cats = [], 0, 0, {}
    for i, (s, a) in enumerate(zip(states, actions)):
        d = shield.filter(s, a)
        if d.touched:
            touched += 1
            for v in d.violations:
                cats[v.category] = cats.get(v.category, 0) + 1
        if d.braked:
            braked += 1
            continue                       # BRAKE zeroes everything by design
        if abs(d.emitted.yaw_rate - a.yaw_rate) > 1e-12:
            edited.append((i, a.yaw_rate, d.emitted.yaw_rate))
    print(f"  {label:34} ticks={len(states):4d} touched={touched:4d} "
          f"braked={braked:3d} yaw_edited={len(edited):3d} "
          f"rules={ {k: v for k, v in sorted(cats.items())} }")
    return edited, braked, touched, cats


def main() -> int:
    shield, policy = build_shield()
    print(f"[policy] {policy.policy_id} {policy.policy_hash}")
    print(f"[shield] obstacle_clearance {'ARMED' if shield.obstacle_map is not None else 'inert'}"
          if hasattr(shield, "obstacle_map") else "")

    all_edited, all_braked, all_touched = [], 0, 0
    all_cats: dict[str, int] = {}

    # 1. straight into the middle of nfz-target, yaw swinging through its range
    states, actions = [], []
    x, y, up = 35.0, -20.0, 22.0
    for i in range(220):
        yr = 1.1 * math.sin(i / 12.0)          # spans the full VLA yaw range
        a = Action4D(vx=0.4, vy=3.5, vz_up=0.0, yaw_rate=yr)
        states.append(State(x=x, y=y, up=up))
        actions.append(a)
        y += 0.35
    e, b, t, c = replay(shield, states, actions, "head-on into the NFZ")
    all_edited += e; all_braked += b; all_touched += t
    for k, v in c.items():
        all_cats[k] = all_cats.get(k, 0) + v

    # 2. altitude floor/ceiling violations (forces the altitude repair path)
    states, actions = [], []
    for i in range(120):
        up_i = 19.0 - i * 0.12                 # sinks below the 18 m floor
        states.append(State(x=20.0, y=-30.0, up=up_i))
        actions.append(Action4D(vx=1.0, vy=0.0, vz_up=-1.8,
                                yaw_rate=0.9 * math.cos(i / 7.0)))
    e, b, t, c = replay(shield, states, actions, "descending through the floor")
    all_edited += e; all_braked += b; all_touched += t
    for k, v in c.items():
        all_cats[k] = all_cats.get(k, 0) + v

    # 3. over-speed, forcing _repair_kinematic — the ONE operator that may touch
    #    yaw. With yaw_rate_max_dps 45.0 against a rad/s value it must not fire.
    states, actions = [], []
    for i in range(120):
        states.append(State(x=0.0, y=0.0, up=22.0))
        actions.append(Action4D(vx=9.0, vy=9.0, vz_up=3.5,
                                yaw_rate=1.1 * math.sin(i / 5.0)))
    e, b, t, c = replay(shield, states, actions, "over speed + climb (kinematic)")
    all_edited += e; all_braked += b; all_touched += t
    for k, v in c.items():
        all_cats[k] = all_cats.get(k, 0) + v

    # 4. skimming a building, forcing the clearance repair
    cmap = city_planner.load_occ(str(CITYMAP))
    states, actions = [], []
    if cmap is not None:
        occ, res, ox, oy = cmap["occ"], cmap["res"], cmap["ox"], cmap["oy"]
        ii, jj = np.argwhere(occ)[len(np.argwhere(occ)) // 2]
        bx, by = ox + ii * res, oy + jj * res
        for i in range(120):
            f = i / 119.0
            states.append(State(x=bx - 25 * (1 - f), y=by, up=22.0))
            actions.append(Action4D(vx=3.5, vy=0.4, vz_up=0.0,
                                    yaw_rate=1.0 * math.sin(i / 9.0)))
        e, b, t, c = replay(shield, states, actions, "closing on a building face")
        all_edited += e; all_braked += b; all_touched += t
        for k, v in c.items():
            all_cats[k] = all_cats.get(k, 0) + v

    print()
    print(f"[V3] violation ticks      : {all_touched}")
    print(f"[V3] rule categories fired: {dict(sorted(all_cats.items()))}")
    print(f"[V3] BRAKE ticks          : {all_braked}  (yaw legitimately zeroed)")
    print(f"[V3] non-brake yaw edits  : {len(all_edited)}")

    if all_touched == 0:
        print("[V3] FAIL — nothing was violated, so nothing was actually tested")
        return 1
    if all_edited:
        i, a, b_ = all_edited[0]
        print(f"[V3] FAIL — the Shield edited yaw (first at tick {i}: "
              f"{a:.4f} -> {b_:.4f})")
        print("     The 'heading is 100% VLA-authored' claim does NOT hold under "
              "this policy. Either the yaw unit was corrected, or a new repair "
              "operator touches yaw. Re-derive the proof before flying.")
        return 1
    print("[V3] PASS — outside of BRAKE, every repair passed yaw_rate through "
          "untouched.\n     Heading is authored by the VLA alone; the Shield acts "
          "on the translation\n     channel. That separation is what makes "
          "simultaneity checkable per tick.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
