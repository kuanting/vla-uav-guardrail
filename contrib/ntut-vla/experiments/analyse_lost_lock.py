"""Why does the lock drop, and why does the aircraft spin looking for it?

Reported from the video: "ketika masuk di perempatan / intersection, object
detection seperti hilang ... sehingga drone membuat manuver 360 dan sampai
menemukan kembali objectnya."

This finds every episode where the controller left `track`, and prints the state
around it, so the cause is read off the flight rather than guessed at:

  * WHERE the car was (tgt_x, tgt_y) and where the aircraft was
  * the RANGE, against the camera's near blind spot (0.86 x altitude, because
    the front camera is pitched 20 deg down with a 29.4 deg vertical half-FOV)
  * the ASPECT the car presented - measured in probe_glb_yaw.py, a car seen
    end-on scores about HALF what it scores broadside (0.066-0.071 against
    0.126-0.146), so a following aircraft looking at a rear bumper is in the
    worst case the detector has
  * the last detection's score and colour before the drop, which separates "the
    detector stopped finding a car" from "it found one and the colour gate
    rejected it"
  * how far the aircraft yawed during the episode, which is the 360 the video
    shows

    python experiments/analyse_lost_lock.py demo/out/demo_traffic
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys


def load(run: pathlib.Path):
    f = run / "flight_log.jsonl"
    rows = []
    for line in f.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return rows


def episodes(rows, modes=("search", "scan")):
    out, cur = [], None
    for r in rows:
        if r.get("mode") in modes:
            if cur is None:
                cur = [r]
            else:
                cur.append(r)
        elif cur is not None:
            out.append(cur)
            cur = None
    if cur:
        out.append(cur)
    return out


def aspect_of(r):
    """Angle between the car's heading and the line of sight, in degrees.

    0 means the aircraft is looking at the car's back (worst for the detector),
    90 means broadside (best). The car drives +y on the straight route, so its
    heading is +y; the line of sight is aircraft -> car."""
    tx, ty = r.get("tgt_x"), r.get("tgt_y")
    if tx is None:
        return None
    dx, dy = tx - r["x"], ty - r["y"]
    if math.hypot(dx, dy) < 1e-6:
        return None
    los = math.atan2(dy, dx)
    car = math.atan2(1.0, 0.0)          # +y, the straight route's direction
    a = abs(math.degrees((los - car + math.pi) % (2 * math.pi) - math.pi))
    return round(min(a, 180.0 - a), 1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--min-ticks", type=int, default=3)
    args = ap.parse_args()

    for rn in args.runs:
        run = pathlib.Path(rn)
        rows = load(run)
        if not rows:
            print(f"{run}: no flight_log.jsonl")
            continue
        print(f"\n===== {run.name}  ({len(rows)} ticks)")

        # Blind spot: anything closer than 0.86 x altitude is under the camera.
        near = [r for r in rows if r.get("rng_m") is not None
                and r["rng_m"] < 0.86 * r.get("up", 9.0)]
        print(f"  ticks inside the camera near blind spot: {len(near)}")

        eps = [e for e in episodes(rows) if len(e) >= args.min_ticks]
        print(f"  lost-lock episodes (>= {args.min_ticks} ticks): {len(eps)}")

        for e in eps:
            i0 = rows.index(e[0])
            before = rows[max(0, i0 - 6):i0]
            last_det = None
            for r in reversed(before):
                if r.get("det"):
                    last_det = r
                    break
            yaws = [r["psi"] for r in e]
            swept = sum(abs((yaws[i + 1] - yaws[i] + math.pi) % (2 * math.pi) - math.pi)
                        for i in range(len(yaws) - 1))
            r0 = e[0]
            print(f"\n  t={r0['t']:6.1f}s  {len(e):3d} ticks ({len(e) * 0.1:.1f}s)  "
                  f"mode={r0['mode']}")
            print(f"     drone ({r0['x']:5.1f},{r0['y']:6.1f}) alt {r0.get('up', 0):.1f}   "
                  f"car ({r0.get('tgt_x')},{round(r0.get('tgt_y', 0), 1)})")
            print(f"     range {r0.get('rng_m')}  aspect-to-car {aspect_of(r0)} deg "
                  f"(0=rear-on, 90=broadside)")
            print(f"     presence={r0.get('presence')} ({r0.get('presence_why')})")
            print(f"     yaw swept during episode: {math.degrees(swept):6.0f} deg")
            if last_det:
                d = last_det["det"]
                print(f"     last box before the drop: score {d.get('score')} "
                      f"colour {d.get('colour')} w {d.get('w')}px "
                      f"at t={last_det['t']:.1f}")
            else:
                print("     no box in the 0.6 s before the drop")
    return 0


if __name__ == "__main__":
    sys.exit(main())
