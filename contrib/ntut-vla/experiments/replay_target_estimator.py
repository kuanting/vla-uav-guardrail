"""Score the target estimator against real recorded noise, without flying.

`flight_log.jsonl` already carries everything the estimator consumes - the
aircraft pose, the detection box, the depth range - and `tgt_x`/`tgt_y` for
scoring. So the estimator can be judged on the exact noise that produced the
1.897 m/s forward command steps, before any simulator time is spent.

What is measured, per flight:

  * **command roughness** - the per-tick step in the forward command computed the
    old way (apparent box width) against the new way (estimated range). This is
    the number the change exists to move;
  * **accuracy** - estimate error against `tgt_x`/`tgt_y`. Ground truth is used
    HERE and only here, for scoring. It never enters the estimator.

    python experiments/replay_target_estimator.py demo/out/demo_traffic
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics as st
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from target_state import TargetState, want_range_from_width   # noqa: E402


def q(a, p):
    return sorted(a)[min(len(a) - 1, int(p * len(a)))] if a else float("nan")


def replay(run: pathlib.Path, want_w: float, speed_max: float,
           range_gain: float, hfov: float = 90.0):
    rows = [json.loads(l) for l in (run / "flight_log.jsonl").read_text(
        encoding="utf-8").splitlines() if l.strip()]
    if not rows:
        return None

    est = TargetState()
    want_rng = want_range_from_width(want_w)
    old_cmd, new_cmd, errs, served = [], [], [], 0
    last_seq = None

    for r in rows:
        t = r["t"]
        det = r.get("det")
        # --- update only when a NEW detection arrived, as in flight ----------
        if det and r.get("det_seq") != last_seq:
            last_seq = r.get("det_seq")
            W = 400.0
            bearing = math.radians(hfov / 2.0) * ((det["cx"] - W / 2) / (W / 2))
            rng = r.get("rng_m")
            if rng is None and det.get("w"):
                # fall back to apparent width, the same geometry implied_width_m
                # uses, inverted for range
                half = math.radians(hfov / 2.0) * (det["w"] / W)
                rng = (4.0 / 2.0) / max(1e-3, math.tan(half))
            if rng:
                est.update(t, r["x"], r["y"], r["psi"], bearing, float(rng))

        # --- the OLD forward command: proportional on apparent width ---------
        if det and det.get("w"):
            w_frac = det["w"] / 400.0
            err = (want_w - w_frac) / max(1e-3, want_w)
            b = math.radians(hfov / 2.0) * ((det["cx"] - 200.0) / 200.0)
            old = float(np.clip(err * speed_max, -0.4 * speed_max, speed_max))
            old *= max(0.0, math.cos(b))
            old_cmd.append(old)

        # --- the NEW forward command: proportional on estimated range --------
        obs = est.observe(t, r["x"], r["y"], r["psi"])
        if obs is not None:
            served += 1
            b, rng = obs
            new = float(np.clip((rng - want_rng) * range_gain,
                                -0.4 * speed_max, speed_max))
            new *= max(0.0, math.cos(b))
            new_cmd.append(new)
            if r.get("tgt_x") is not None:
                errs.append(math.dist((est.x[0], est.x[1]),
                                      (r["tgt_x"], r["tgt_y"])))

    def steps(a):
        return [abs(a[i + 1] - a[i]) for i in range(len(a) - 1)]

    so, sn = steps(old_cmd), steps(new_cmd)
    return {
        "ticks": len(rows),
        "served_frac": round(served / max(1, len(rows)), 3),
        "old_p95": round(q(so, .95), 4), "old_max": round(max(so), 4) if so else None,
        "new_p95": round(q(sn, .95), 4), "new_max": round(max(sn), 4) if sn else None,
        "err_p50": round(st.median(errs), 2) if errs else None,
        "err_p95": round(q(errs, .95), 2) if errs else None,
        "gated_out": est.n_rejected, "updates": est.n_updates,
        "want_range_m": round(want_rng, 1),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--want-width", type=float, default=0.16)
    ap.add_argument("--speed-max", type=float, default=3.0)
    ap.add_argument("--range-gain", type=float, default=0.25)
    args = ap.parse_args()

    print(f"stand-off implied by --want-width {args.want_width}: "
          f"{want_range_from_width(args.want_width):.1f} m")
    print(f"{'run':16s} {'served':>7s} {'OLD p95':>8s} {'OLD max':>8s} "
          f"{'NEW p95':>8s} {'NEW max':>8s} {'err p50':>8s} {'err p95':>8s} {'gated':>6s}")
    for r in args.runs:
        run = pathlib.Path(r)
        if not (run / "flight_log.jsonl").is_file():
            print(f"{run.name:16s} no flight_log.jsonl")
            continue
        m = replay(run, args.want_width, args.speed_max, args.range_gain)
        print(f"{run.name:16s} {m['served_frac']:7.3f} {m['old_p95']:8.4f} "
              f"{m['old_max']:8.4f} {m['new_p95']:8.4f} {m['new_max']:8.4f} "
              f"{str(m['err_p50']):>8s} {str(m['err_p95']):>8s} {m['gated_out']:6d}")
    print("\nOLD = forward command from apparent box width (what flies today)")
    print("NEW = forward command from the estimated range")
    print("err = estimate against tgt_x/tgt_y, used for SCORING only")
    return 0


if __name__ == "__main__":
    sys.exit(main())
