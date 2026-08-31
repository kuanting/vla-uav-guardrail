"""How often does the tracked box leap to a different object?

This is the metric behind the complaint that started the vehicle work --
"object trackingnya berpindah-pindah tidak jelas" -- and it is not in
metrics.json, so it is computed here from `detections.jsonl`.

A jump is a box centre moving more than `--px` pixels between consecutive
detections. At 3-5 Hz and 400x225, a car being followed moves a few pixels a
frame; 60 px is a third of the frame width and cannot be the same vehicle.

Runs are compared with their QUERY printed, because that is what makes a
comparison honest or dishonest here. `v7_traffic` scores a perfect 1.000
detector hit rate and is NOT the baseline to beat: its query was "a white car"
against a scene that is 8.5% white, and it reported the target ABSENT on 83% of
ticks. It was detecting pale buildings. The baseline that matters is whatever
configuration was actually shipping.

    python experiments/compare_track_jumps.py demo/out/demo_traffic <others...>
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys


def load(run: pathlib.Path):
    f = run / "detections.jsonl"
    if not f.is_file():
        return None
    rows = []
    for line in f.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        d = r.get("det")
        if d and d.get("cx") is not None:
            rows.append((float(r.get("t", 0.0)), float(d["cx"]), float(d["cy"]),
                         float(d.get("score", 0.0)), float(d.get("colour", 0.0) or 0.0),
                         bool(r.get("switched", False))))
    return rows


def stats(rows, px: float, gap_s: float):
    if not rows or len(rows) < 2:
        return None
    jumps = pairs = 0
    for (t0, x0, y0, *_), (t1, x1, y1, *_) in zip(rows, rows[1:]):
        # A long gap means the detector lost the object and reacquired it; the
        # displacement across that gap is not a "jump between objects" and
        # counting it would flatter or punish a run for its miss rate instead.
        if t1 - t0 > gap_s:
            continue
        pairs += 1
        if math.dist((x0, y0), (x1, y1)) > px:
            jumps += 1
    scores = [r[3] for r in rows]
    colours = [r[4] for r in rows]
    return {
        "detections": len(rows),
        "pairs_compared": pairs,
        "jumps": jumps,
        "jump_rate": None if not pairs else round(jumps / pairs, 4),
        "switched": sum(1 for r in rows if r[5]),
        "score_mean": round(sum(scores) / len(scores), 4),
        "colour_mean": round(sum(colours) / len(colours), 4),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--px", type=float, default=60.0)
    ap.add_argument("--gap-s", type=float, default=1.0)
    args = ap.parse_args()

    print(f"{'run':22s} {'query':16s} {'dets':>5s} {'jump%':>7s} "
          f"{'switch':>6s} {'score':>7s} {'colour':>7s}")
    for r in args.runs:
        run = pathlib.Path(r)
        rows = load(run)
        if rows is None:
            print(f"{run.name:22s} no detections.jsonl")
            continue
        s = stats(rows, args.px, args.gap_s)
        query = "?"
        m = run / "metrics.json"
        if m.is_file():
            try:
                query = json.load(open(m, encoding="utf-8")).get("object", "?")
            except Exception:
                pass
        if s is None:
            print(f"{run.name:22s} {query:16s} too few detections")
            continue
        print(f"{run.name:22s} {query:16s} {s['detections']:5d} "
              f"{100 * s['jump_rate']:6.1f}% {s['switched']:6d} "
              f"{s['score_mean']:7.4f} {s['colour_mean']:7.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
