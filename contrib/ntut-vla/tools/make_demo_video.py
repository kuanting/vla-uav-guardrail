"""
Stitch a flight's recorded frames into one demo video.

Left: what the drone sees, with the detected box and the live telemetry drawn on
it. Right: the third-person chase view. Side by side, because the claim only
reads if both are visible at once — a word chose the box, the box drove the
aircraft, and the guardrail was underneath the whole time.

Run:
    python tools/make_demo_video.py --tag vlm_col
    python tools/make_demo_video.py --tag vlm_col --fps 12 --out demo.mp4
"""
from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    import cv2
    import numpy as np

    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True, help="flight tag under demo/out/")
    ap.add_argument("--fps", default="auto",
                    help="frames per second. 'auto' (default) DERIVES it from "
                         "the flight log so the video runs in real time. A "
                         "fixed value is a guess about the control loop's rate, "
                         "and that guess was wrong: the loop holds about 7.4 Hz, "
                         "not the nominal 10, so writing at 10 played every "
                         "recording 1.35x fast while claiming to be real time.")
    ap.add_argument("--out", default=None)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    base = ROOT / "demo" / "out" / args.tag / "view"
    fpv = sorted(glob.glob(str(base / "fpv" / "*.jpg")))
    tps = sorted(glob.glob(str(base / "tps" / "*.jpg")))
    if not fpv and not tps:
        print(f"no frames under {base} — was the flight run with --save-view?")
        return 1

    # Real time means the rate the frames were ACTUALLY CAPTURED at, and the
    # only component that knows that is the recorder, which now writes it to
    # view/recorder.json.
    #
    # Deriving it from the flight log instead is wrong, and was wrong here for
    # two different reasons in turn. First this script assumed the nominal 10 Hz
    # tick rate, while the loop held about 7.4 Hz, so every clip played 1.35x
    # fast. The fix was frames / flight_seconds - which is also wrong, because
    # the recorder starts before the mission clock (scene load, prop spawn,
    # take-off) and stops after it. Measured: 1734 frames spanning ~87 s of
    # recorder life, divided by a 70 s mission, gives 24.8 fps for a recorder
    # pacing at 20. That plays 1.24x fast while claiming real time.
    #
    # The sidecar removes the guess. The flight-log path is kept only as a
    # fallback for older runs, and it says so on the console when it is used.
    fps = None
    if str(args.fps).lower() == "auto":
        side = base / "recorder.json"
        if side.is_file():
            try:
                rec = json.loads(side.read_text(encoding="utf-8"))
                cand = rec.get("achieved_hz") or rec.get("target_hz")
                if cand and float(cand) > 0:
                    fps = round(float(cand), 2)
                    # The WRITING window, not the recorder's whole life. The
                    # recorder is up before the mission clock is, and quoting
                    # the lifetime here made the arithmetic look wrong even
                    # once achieved_hz was right.
                    win = rec.get("writing_seconds") or rec.get("seconds")
                    print(f"[video] recorder captured {rec.get('frames')} frames "
                          f"over {win} s of writing -> {fps} fps "
                          f"(target was {rec.get('target_hz')} Hz)")
            except (ValueError, KeyError, TypeError):
                pass

        if fps is None:
            log = ROOT / "demo" / "out" / args.tag / "flight_log.jsonl"
            if log.is_file():
                try:
                    lines = [l for l in log.read_text(encoding="utf-8").splitlines() if l.strip()]
                    secs = json.loads(lines[-1]).get("t")
                    n_frames = max(len(fpv), len(tps))
                    if secs and secs > 1 and n_frames > 1:
                        fps = max(1.0, round(n_frames / float(secs), 2))
                        print(f"[video] NO recorder.json; falling back to "
                              f"{n_frames} frames / {secs:.1f} s of flight = {fps} fps. "
                              f"This OVERSTATES the rate if the recorder outlived "
                              f"the mission clock; playback will be faster than real time.")
                except (ValueError, KeyError, IndexError):
                    pass

        if fps is None:
            fps = 10.0
            print("[video] no recorder.json and no usable flight log; falling back to 10 fps")
    else:
        fps = float(args.fps)

    out_path = Path(args.out) if args.out else (
        ROOT / "demo" / "out" / args.tag / f"{args.tag}_demo.mp4")

    def load(p, h):
        im = cv2.imread(p)
        if im is None:
            return None
        s = h / im.shape[0]
        return cv2.resize(im, (int(im.shape[1] * s), h))

    n = max(len(fpv), len(tps))
    first = None
    for i in range(n):
        a = load(fpv[min(i, len(fpv) - 1)], args.height) if fpv else None
        b = load(tps[min(i, len(tps) - 1)], args.height) if tps else None
        if a is None and b is None:
            continue
        panel = (np.hstack([a, b]) if a is not None and b is not None
                 else (a if a is not None else b))
        first = panel.shape
        break
    if first is None:
        print("frames present but none decodable")
        return 1

    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"),
                             fps, (first[1], first[0]))
    written = 0
    for i in range(n):
        a = load(fpv[min(i, len(fpv) - 1)], args.height) if fpv else None
        b = load(tps[min(i, len(tps) - 1)], args.height) if tps else None
        if a is None and b is None:
            continue
        if a is not None and b is not None:
            if a.shape[0] != b.shape[0]:
                b = cv2.resize(b, (b.shape[1], a.shape[0]))
            panel = np.hstack([a, b])
        else:
            panel = a if a is not None else b
        if panel.shape[:2] != first[:2]:
            panel = cv2.resize(panel, (first[1], first[0]))
        writer.write(panel)
        written += 1
    writer.release()
    print(f"[video] {written} frames at {fps} fps "
          f"({written/fps:.0f}s) -> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
