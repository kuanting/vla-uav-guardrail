"""Was the target in frame when the lock dropped, and what was in its place?

`analyse_lost_lock.py` shows that every lost-lock episode happens at the same
spot with the car dead ahead and the presence monitor still saying PRESENT. That
is consistent with occlusion, but "there is a tree in the picture" is an
impression, not a measurement.

So project the car's GROUND-TRUTH position into the camera and look at what the
pixels there actually are. Three outcomes, and they are distinguishable:

  * projects OUTSIDE the frame        -> not occlusion, a framing/FOV problem
  * projects inside, target colour present -> the detector missed a visible car
  * projects inside, target colour absent  -> something is in front of it

Camera model from demo/pas_config/robot_semantic_quad.jsonc: 400x225, 90 deg
horizontal FOV, mounted "0 -20.0 0" i.e. pitched 20 degrees down.

    python experiments/check_target_occlusion.py demo/out/demo_traffic
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

W, H = 400, 225
FX = (W / 2.0) / math.tan(math.radians(90.0) / 2.0)     # = 200
CX, CY = W / 2.0, H / 2.0
PITCH_DOWN = math.radians(20.0)


def project(row):
    """Car ground truth -> pixel, or None if behind the camera."""
    tx, ty = row.get("tgt_x"), row.get("tgt_y")
    if tx is None:
        return None
    dx, dy = tx - row["x"], ty - row["y"]
    dz = row.get("up", 9.0)                 # car on the ground, drone above it
    psi = row["psi"]
    xb = dx * math.cos(psi) + dy * math.sin(psi)
    yb = -dx * math.sin(psi) + dy * math.cos(psi)
    zb = dz
    xc = xb * math.cos(PITCH_DOWN) + zb * math.sin(PITCH_DOWN)
    zc = -xb * math.sin(PITCH_DOWN) + zb * math.cos(PITCH_DOWN)
    if xc <= 0.5:
        return None
    return (CX + FX * (yb / xc), CY + FX * (zc / xc), xc)


def main() -> int:
    import follow_vlm as fv
    from PIL import Image

    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--colour", default="yellow")
    ap.add_argument("--pad", type=int, default=18)
    args = ap.parse_args()

    run = pathlib.Path(args.run)
    rows = [json.loads(l) for l in (run / "flight_log.jsonl").read_text(
        encoding="utf-8").splitlines() if l.strip()]
    fpv = run / "view" / "fpv"

    lost = [r for r in rows if r.get("mode") in ("search", "scan")]
    print(f"{run.name}: {len(lost)} lost-lock ticks")
    print(f"{'t':>7s} {'u':>6s} {'v':>6s} {'rng':>6s}  in-frame  "
          f"{args.colour}-frac-at-that-spot")

    inside = outside = 0
    fracs = []
    for r in lost[::10]:
        pr = project(r)
        if pr is None:
            print(f"{r['t']:7.1f}  behind the camera")
            continue
        u, v, rng = pr
        ok = 0 <= u < W and 0 <= v < H
        inside += ok
        outside += (not ok)
        frac = None
        if ok:
            # The saved frame is the ANNOTATED fpv jpg at 2x; sample the patch
            # around the projected point and ask how much of it is the target
            # colour, using the same test the live colour gate uses.
            cand = fpv / f"{r['tick']:05d}.jpg"
            if cand.is_file():
                img = np.asarray(Image.open(cand).convert("RGB"))
                sy = img.shape[0] / H
                sx = img.shape[1] / W
                cu, cv = int(u * sx), int(v * sy)
                p = int(args.pad * max(sx, sy))
                box = [max(0, cu - p), max(0, cv - p),
                       min(img.shape[1], cu + p), min(img.shape[0], cv + p)]
                frac = fv.colour_match(img, box, args.colour)
                fracs.append(frac)
        print(f"{r['t']:7.1f} {u:6.1f} {v:6.1f} {rng:6.1f}  "
              f"{'yes' if ok else 'NO ':>7s}   "
              f"{'-' if frac is None else f'{frac:.3f}'}")

    print(f"\n  projected INSIDE the frame: {inside}   outside: {outside}")
    if fracs:
        a = np.array(fracs)
        print(f"  {args.colour} fraction where the car should be: "
              f"mean {a.mean():.3f}  max {a.max():.3f}  "
              f"frac of samples above the 0.10 gate: {(a > 0.10).mean():.2f}")
        print("\n  Reading: in-frame with the target colour ABSENT at the spot "
              "means something is in front of the car, not that the detector "
              "is blind.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
