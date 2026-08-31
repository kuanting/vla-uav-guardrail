"""
Re-encode a demo master into an H.264 copy small enough to send.

Why this is a separate step
---------------------------
`tools/make_demo_video.py` writes the master with OpenCV's `mp4v` writer
(FMP4, MPEG-4 Part 2). That path is verified — it derives fps from
`flight_log.jsonl` so the clip's duration matches the flight by construction —
and it is not worth destabilising to save disk. The cost is size: a 70 s
2560x720 master lands at 130-161 MB, which cannot be attached to an email or
embedded in a deck. A 486 MB .pptx was produced exactly that way.

So the master stays as it is and this script produces the DELIVERABLE copy.
Two artefacts, one of them auditable against the other.

The check that matters
----------------------
A re-encode that silently drops frames still plays, and still looks fine, and
is wrong: it would misreport how long the aircraft flew. So after encoding this
script reads both files back and refuses the result unless the duration agrees
within `--tol` seconds. A bad encode fails loudly instead of shipping.

Usage:
    python tools/encode_deliverable.py --tag demo_follow
    python tools/encode_deliverable.py --all
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "docs" / "video"

# winget puts ffmpeg on PATH only for shells started after the install, so the
# absolute WinGet location is tried as well rather than requiring a restart.
WINGET_FFMPEG = (
    Path(os.environ.get("LOCALAPPDATA", "")) / "Microsoft" / "WinGet" / "Packages"
    / "Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe"
    / "ffmpeg-9.0-full_build" / "bin" / "ffmpeg.exe"
)

DEMO_TAGS = ["demo_follow", "demo_traffic", "demo_nfz"]


def find_ffmpeg() -> str:
    """Locate ffmpeg, preferring PATH and falling back to the WinGet install."""
    onpath = shutil.which("ffmpeg")
    if onpath:
        return onpath
    if WINGET_FFMPEG.exists():
        return str(WINGET_FFMPEG)
    # Last resort: any ffmpeg.exe under the WinGet package root, so a version
    # bump does not break this.
    root = Path(os.environ.get("LOCALAPPDATA", "")) / "Microsoft" / "WinGet" / "Packages"
    if root.exists():
        for cand in root.glob("Gyan.FFmpeg*/**/bin/ffmpeg.exe"):
            return str(cand)
    raise SystemExit(
        "ffmpeg not found. Install it with:\n"
        "    winget install --id Gyan.FFmpeg -e"
    )


def probe(path: Path) -> dict:
    """Frame count, fps, size and duration of a video, read back from the file."""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"cannot open {path}")
    fps = cap.get(cv2.CAP_PROP_FPS)
    frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    cap.release()
    return {
        "fps": fps,
        "frames": frames,
        "w": w,
        "h": h,
        "seconds": (frames / fps) if fps else 0.0,
        "codec": "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4)),
        "mb": path.stat().st_size / 1048576.0,
    }


def encode(tag: str, crf: int, preset: str, tol: float) -> dict:
    master = ROOT / "demo" / "out" / tag / f"{tag}_demo.mp4"
    if not master.exists():
        raise SystemExit(f"master not found: {master}")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    dest = OUT_DIR / f"{tag}.mp4"

    src = probe(master)
    print(f"[{tag}] master  {src['w']}x{src['h']} {src['fps']:.2f} fps "
          f"{src['frames']} frames = {src['seconds']:.1f} s "
          f"{src['codec']} {src['mb']:.1f} MB")

    cmd = [
        find_ffmpeg(), "-y", "-hide_banner", "-loglevel", "error",
        "-i", str(master),
        "-c:v", "libx264", "-crf", str(crf), "-preset", preset,
        # yuv420p is the pixel format every player and PowerPoint accepts;
        # without it some encodes come out as yuv444p and refuse to play.
        "-pix_fmt", "yuv420p",
        "-movflags", "+faststart",
        "-an",                      # the masters carry no audio
        str(dest),
    ]
    subprocess.run(cmd, check=True)

    dst = probe(dest)
    print(f"[{tag}] encoded {dst['w']}x{dst['h']} {dst['fps']:.2f} fps "
          f"{dst['frames']} frames = {dst['seconds']:.1f} s "
          f"{dst['codec']} {dst['mb']:.1f} MB  "
          f"({src['mb'] / max(dst['mb'], 1e-9):.1f}x smaller)")

    drift = abs(dst["seconds"] - src["seconds"])
    if drift > tol:
        dest.unlink(missing_ok=True)
        raise SystemExit(
            f"[{tag}] REJECTED: duration moved {drift:.2f} s "
            f"({src['seconds']:.2f} -> {dst['seconds']:.2f}), tolerance {tol} s. "
            f"A clip that plays but misreports the flight length is worse than none.")

    return {"tag": tag, "master_mb": src["mb"], "out_mb": dst["mb"],
            "seconds": dst["seconds"], "path": dest}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", action="append", default=[],
                    help="flight tag under demo/out/ (repeatable)")
    ap.add_argument("--all", action="store_true",
                    help=f"encode the three demo tags: {', '.join(DEMO_TAGS)}")
    ap.add_argument("--crf", type=int, default=23,
                    help="H.264 quality, lower is better. 23 is visually clean here")
    ap.add_argument("--preset", default="slow")
    ap.add_argument("--tol", type=float, default=0.15,
                    help="how far the encoded duration may drift, seconds")
    args = ap.parse_args()

    tags = args.tag or (DEMO_TAGS if args.all else [])
    if not tags:
        ap.error("give --tag TAG or --all")

    print(f"ffmpeg: {find_ffmpeg()}\n")
    results = [encode(t, args.crf, args.preset, args.tol) for t in tags]

    print("\n  " + f"{'tag':<16}{'master':>10}{'delivered':>12}{'seconds':>10}")
    for r in results:
        print(f"  {r['tag']:<16}{r['master_mb']:>9.1f}M{r['out_mb']:>11.1f}M"
              f"{r['seconds']:>10.1f}")
    print(f"\n  written to {OUT_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
