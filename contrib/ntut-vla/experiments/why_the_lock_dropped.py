"""When the lock dropped on a car in plain view, which gate rejected it?

The flight log proves the target was NOT hidden: at the start of every lost-lock
episode the car projects to u ~ 200 of 400 -- dead centre -- at 17-22 m, well
outside the camera's 7.7 m near blind spot, with the presence monitor still
saying PRESENT. The detector simply stopped returning it.

`detections.jsonl` cannot answer why, because it only records detections that
were ACCEPTED. So re-run the detector offline on the saved frames from the loss
window and print what the live pipeline would have done with each box:

    raw OWL-ViT score   vs  --det-thresh
    colour fraction     vs  --colour-min

That separates the two candidate causes, which need different fixes:

  * score below threshold  -> the detector genuinely lost a small/odd-angled car
  * score fine, colour low -> the COLOUR GATE rejected a car it had found

The second is the live suspicion, because the last accepted box before each
episode had a colour fraction of 0.126 and 0.101 against a 0.10 gate: sitting
right on the threshold, where a small change in aspect tips it over. And the
aspect does change - the aircraft ends up directly behind the car (measured
0.4-2.9 deg), which shows a taxi's least-yellow face.

    python experiments/why_the_lock_dropped.py demo/out/demo_traffic --t0 19 --t1 25
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

DETECTOR_ID = "google/owlvit-base-patch32"


def main() -> int:
    import torch
    from PIL import Image
    from transformers import OwlViTForObjectDetection, OwlViTProcessor
    import follow_vlm as fv

    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--object", default="a yellow car")
    ap.add_argument("--det-thresh", type=float, default=0.008)
    ap.add_argument("--colour-min", type=float, default=0.10)
    ap.add_argument("--t0", type=float, default=19.0)
    ap.add_argument("--t1", type=float, default=25.0)
    args = ap.parse_args()

    run = pathlib.Path(args.run)
    rows = [json.loads(l) for l in (run / "flight_log.jsonl").read_text(
        encoding="utf-8").splitlines() if l.strip()]
    fpv = run / "view" / "fpv"

    proc = OwlViTProcessor.from_pretrained(DETECTOR_ID)
    model = OwlViTForObjectDetection.from_pretrained(DETECTOR_ID).to("cuda").eval()
    word = fv.colour_word(args.object)

    print(f"query {args.object!r}   colour word {word!r}   "
          f"gates: score > {args.det_thresh}, colour >= {args.colour_min}")
    print(f"{'t':>6s} {'mode':>7s} {'live':>5s} {'raw_score':>10s} "
          f"{'colour':>7s} {'w_px':>6s}  offline verdict")

    n_colour_reject = n_score_reject = n_would_pass = 0
    for r in rows:
        if not (args.t0 <= r["t"] <= args.t1):
            continue
        f = fpv / f"{r['tick']:05d}.jpg"
        if not f.is_file():
            continue
        img = Image.open(f).convert("RGB")
        inputs = proc(text=[[args.object]], images=img, return_tensors="pt").to("cuda")
        with torch.no_grad():
            out = model(**inputs)
        Wi, Hi = img.size
        res = proc.post_process_object_detection(
            out, threshold=0.0, target_sizes=torch.tensor([[Hi, Wi]]).to("cuda"))[0]
        sc, bx = res["scores"], res["boxes"]
        if not len(sc):
            print(f"{r['t']:6.1f} {r.get('mode'):>7s} {str(r.get('seen')):>5s} "
                  f"{'none':>10s}       -      -  no box at all")
            continue
        i = int(sc.argmax())
        s = float(sc[i])
        box = [float(v) for v in bx[i].tolist()]
        cm = fv.colour_match(np.asarray(img), box, word) if word else None
        wpx = box[2] - box[0]

        if s <= args.det_thresh:
            verdict = "REJECTED: score below threshold"
            n_score_reject += 1
        elif cm is not None and cm < args.colour_min:
            verdict = f"REJECTED BY THE COLOUR GATE ({cm:.3f} < {args.colour_min})"
            n_colour_reject += 1
        else:
            verdict = "would pass"
            n_would_pass += 1
        print(f"{r['t']:6.1f} {r.get('mode'):>7s} {str(r.get('seen')):>5s} "
              f"{s:10.4f} {('-' if cm is None else f'{cm:7.3f}')} {wpx:6.1f}  {verdict}")

    print(f"\n  colour-gate rejections: {n_colour_reject}   "
          f"score rejections: {n_score_reject}   would pass: {n_would_pass}")
    print("  NOTE: these are the ANNOTATED frames, so the drawn HUD and box "
          "perturb the score slightly. The colour fraction is measured inside "
          "the box and is unaffected.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
