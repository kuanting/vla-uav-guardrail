"""
Generate a RANDOM waypoint mission with a no-fly-zone placed on EACH leg, so the
global planner must detour around every NFZ (and the buildings). Writes a policy
YAML + prints the --route string for aerialvla_pas_demo.py.

Run:  python demo/gen_random_scenario.py --seed 7 --n 4
"""
from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))
import city_planner as cp  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--n", type=int, default=4, help="number of waypoints")
    ap.add_argument("--citymap", default=str(ROOT / "demo/out/citymap/occ_day.npz"))
    ap.add_argument("--out", default=str(ROOT / "policies/random_scenario.yaml"))
    ap.add_argument("--band-lo", type=float, default=35.0)
    ap.add_argument("--band-hi", type=float, default=55.0)
    ap.add_argument("--nfz", type=float, default=12.0, help="NFZ side length (m)")
    args = ap.parse_args()
    random.seed(args.seed)

    m = cp.load_occ(args.citymap)
    occ, res, ox, oy = m["occ"], m["res"], m["ox"], m["oy"]
    grid = cp.inflate(occ, res, 5.0)

    def free_pt(margin=55.0):
        for _ in range(2000):
            x = random.uniform(-margin, margin)
            y = random.uniform(-margin, margin)
            if not cp.is_blocked(grid, res, ox, oy, x, y):
                return (round(x), round(y))
        return (0, 0)

    # spawn is (35, -20); pick N spread-out free waypoints
    wps = []
    while len(wps) < args.n:
        p = free_pt()
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 30 for q in wps):
            wps.append(p)

    SPAWN = (35.0, -20.0)
    stops = [SPAWN] + wps
    nfzs = []
    for i, (a, b) in enumerate(zip(stops[:-1], stops[1:])):
        mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)      # midpoint of the leg
        h = args.nfz / 2
        # NEVER let the NFZ box contain the spawn or any waypoint (with a 4 m
        # buffer) — otherwise the drone starts/sits inside it during takeoff and
        # racks up unavoidable NFZ time (the Shield can't undo a spawn-inside).
        if any(abs(mid[0] - s[0]) < h + 4 and abs(mid[1] - s[1]) < h + 4
               for s in stops):
            continue
        # only keep the NFZ if a detour still exists (planner finds a path)
        test = grid.copy()
        for ii in range(occ.shape[0]):
            wx = ox + ii * res
            if abs(wx - mid[0]) > h:
                continue
            for jj in range(occ.shape[1]):
                wy = oy + jj * res
                if abs(wy - mid[1]) <= h:
                    test[ii, jj] = 1
        if cp.plan(test, res, ox, oy, a, b) is not None:
            nfzs.append((mid[0] - h, mid[0] + h, mid[1] - h, mid[1] + h))

    # write the policy YAML
    lines = ["policy_id: random-scenario", "version: 0.1.0", "constraints:"]
    for i, (xmn, xmx, ymn, ymx) in enumerate(nfzs):
        lines += [
            f"  - id: nfz-leg-{i+1}", "    type: polygon_fence",
            "    constraint_type: hard", "    priority: P0",
            "    violation_action: repair", "    vertices:",
            f"      - {{x: {xmn:.1f}, y: {ymn:.1f}}}",
            f"      - {{x: {xmx:.1f}, y: {ymn:.1f}}}",
            f"      - {{x: {xmx:.1f}, y: {ymx:.1f}}}",
            f"      - {{x: {xmn:.1f}, y: {ymx:.1f}}}",
            "    altitude_floor_m: 0", "    altitude_ceiling_m: 60",
            "    margin_m: 1.0",
        ]
    lines += [
        "  - id: alt-band", "    type: altitude_envelope",
        "    constraint_type: hard", "    priority: P0",
        "    violation_action: repair",
        f"    alt_min_m: {args.band_lo:.0f}", f"    alt_max_m: {args.band_hi:.0f}",
        "  - id: kin-caps", "    type: kinematic_envelope",
        "    constraint_type: hard", "    priority: P1",
        "    violation_action: repair",
        "    speed_max_mps: 4.0", "    climb_rate_max_mps: 2.0",
        "    yaw_rate_max_dps: 45.0",
    ]
    Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")

    route = "; ".join(f"{x},{y}" for x, y in wps)
    print(f"WAYPOINTS: {wps}")
    print(f"NFZS ({len(nfzs)}): {[tuple(round(v) for v in n) for n in nfzs]}")
    print(f"POLICY: {args.out}")
    print(f"ROUTE={route}")


if __name__ == "__main__":
    main()
