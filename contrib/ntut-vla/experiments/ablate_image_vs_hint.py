"""
What actually drives AerialVLA's action: the pixels, the object phrase, or the
direction hint?

This question decides whether a language-only semantic mission is possible at
all with this model, and it does not need a single flight to answer. Every
combination of (captured frame) x (direction hint) x (object description) is run
through the model once, and the variance of the output is attributed to each
factor.

The prompt AerialVLA was trained on is

    "<image>\nFly {direction}and find the target. {object}\nAction: "

and `{direction}` is computed from the GROUND-TRUTH target coordinates. So if the
output turns out to be a function of `{direction}` alone, then AerialVLA as
deployed is not doing vision-based object navigation — it is following a compass
phrase derived from coordinates it was handed, and the camera is close to
decorative. That is a claim worth making carefully, hence this script.

Prerequisite: experiments/capture_frames.py (needs the sim once).
This script needs no simulator.

Run:  python experiments/ablate_image_vs_hint.py
"""
from __future__ import annotations

import itertools
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from aerialvla_demo import build_prompt, dequantize          # noqa: E402

FRAMES = ROOT / "experiments" / "out" / "frames"
OUT = ROOT / "experiments" / "out"
BASE_ID = "D:/models/openvla-7b"

HINTS = ["", "straight ahead ", "forward-right ", "forward-left ",
         "to your right ", "to your left "]
OBJECTS = ["orange barrier", "blue barrier", ""]


def load_model(lora_id: str):
    import torch
    from peft import PeftModel
    from transformers import (AutoImageProcessor, AutoModelForVision2Seq,
                              AutoTokenizer, BitsAndBytesConfig)
    tok = AutoTokenizer.from_pretrained(BASE_ID, trust_remote_code=True)
    ip = AutoImageProcessor.from_pretrained(BASE_ID, trust_remote_code=True)
    t0 = time.time()
    base = AutoModelForVision2Seq.from_pretrained(
        BASE_ID, attn_implementation="eager", torch_dtype=torch.bfloat16,
        quantization_config=BitsAndBytesConfig(
            load_in_4bit=True, bnb_4bit_compute_dtype=torch.bfloat16,
            bnb_4bit_quant_type="nf4", llm_int8_skip_modules=["projector"]),
        device_map={"": 0}, low_cpu_mem_usage=True, trust_remote_code=True)
    model = PeftModel.from_pretrained(base, lora_id).eval()
    print(f"[model] {lora_id} loaded in {time.time()-t0:.0f}s")
    return torch, tok, ip, model


def infer(torch, tok, ip, model, img, hint: str, obj: str) -> dict:
    import re
    prompt = build_prompt(hint, obj)
    enc = tok(prompt, return_tensors="pt")
    pv = ip(images=img, return_tensors="pt")["pixel_values"]
    with torch.inference_mode():
        out = model.generate(input_ids=enc["input_ids"].to("cuda"),
                             attention_mask=enc["attention_mask"].to("cuda"),
                             pixel_values=pv.to("cuda", dtype=torch.bfloat16),
                             max_new_tokens=20, do_sample=False,
                             eos_token_id=[tok.eos_token_id])
    tail = tok.decode(out[0], skip_special_tokens=False).split("Action:")[-1]
    ints = re.findall(r"\d+", tail)
    if len(ints) < 3:
        return {"ok": False, "tail": tail}
    b = (int(ints[-3]), int(ints[-2]), int(ints[-1]))
    return {"ok": True, "bins": b, "fwd": dequantize(b[0], "forward"),
            "down": dequantize(b[1], "down"), "yaw": dequantize(b[2], "yaw"),
            "land": "LAND" in tail, "tail": tail.strip()}


def spread(vals) -> float:
    v = [x for x in vals if x is not None]
    return float(np.ptp(v)) if len(v) > 1 else 0.0


def main() -> int:
    from PIL import Image
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--adapter", default="D:/models/aerialvla-lora/aero_vla")
    ap.add_argument("--label", default="orig")
    args = ap.parse_args()

    meta = json.loads((FRAMES / "frames.json").read_text(encoding="utf-8"))
    frames = meta["frames"]
    imgs = {f["file"]: Image.open(FRAMES / f["file"]).convert("RGB") for f in frames}
    torch, tok, ip, model = load_model(args.adapter)

    recs = []
    total = len(frames) * len(HINTS) * len(OBJECTS)
    t0 = time.time()
    for k, (f, hint, obj) in enumerate(
            itertools.product(frames, HINTS, OBJECTS), 1):
        r = infer(torch, tok, ip, model, imgs[f["file"]], hint, obj)
        recs.append({"file": f["file"], "scene": f["scene"],
                     "beta_actual_deg": f["beta_actual_deg"],
                     "hint": hint.strip() or "(none)", "object": obj or "(none)",
                     **r})
        if k % 12 == 0:
            print(f"  {k}/{total}  ({(time.time()-t0)/k:.1f}s each)")
    (OUT / f"ablation_{args.label}.json").write_text(
        json.dumps(recs, indent=1), encoding="utf-8")

    ok = [r for r in recs if r.get("ok")]
    print(f"\n=== {args.label}: {len(ok)}/{len(recs)} parsed ===\n")

    # 1. hold image+object fixed, vary the hint -> how much does yaw move?
    by_hint, by_img, by_obj = [], [], []
    for f in frames:
        for obj in OBJECTS:
            g = [r for r in ok if r["file"] == f["file"] and r["object"] == (obj or "(none)")]
            if len(g) > 1:
                by_hint.append(spread([r["yaw"] for r in g]))
    for hint in HINTS:
        for obj in OBJECTS:
            g = [r for r in ok if r["hint"] == (hint.strip() or "(none)")
                 and r["object"] == (obj or "(none)")]
            if len(g) > 1:
                by_img.append(spread([r["yaw"] for r in g]))
    for f in frames:
        for hint in HINTS:
            g = [r for r in ok if r["file"] == f["file"]
                 and r["hint"] == (hint.strip() or "(none)")]
            if len(g) > 1:
                by_obj.append(spread([r["yaw"] for r in g]))

    print("Yaw-command range (rad/s) when ONE factor varies and the others are held fixed:")
    print(f"  vary the DIRECTION HINT      : mean {np.mean(by_hint):.3f}  max {np.max(by_hint):.3f}")
    print(f"  vary the IMAGE               : mean {np.mean(by_img):.3f}  max {np.max(by_img):.3f}")
    print(f"  vary the OBJECT DESCRIPTION  : mean {np.mean(by_obj):.3f}  max {np.max(by_obj):.3f}")

    # 2. does the hint alone predict the sign of the commanded turn?
    print("\nMean commanded yaw per hint (averaged over every image and object):")
    for hint in HINTS:
        g = [r for r in ok if r["hint"] == (hint.strip() or "(none)")]
        if g:
            lands = sum(1 for r in g if r["land"])
            print(f"  {hint.strip() or '(none)':16} yaw={np.mean([r['yaw'] for r in g]):+.3f}  "
                  f"fwd={np.mean([r['fwd'] for r in g]):.2f}  "
                  f"LAND {lands}/{len(g)}")

    # 3. the decisive one for a semantic mission: with NO hint, does the
    #    presence of the object in the image change anything at all?
    print("\nNo hint at all — target present vs absent in the pixels:")
    for scene in ("present", "absent"):
        g = [r for r in ok if r["hint"] == "(none)" and r["scene"] == scene]
        if g:
            print(f"  {scene:8} n={len(g):2d} yaw={np.mean([r['yaw'] for r in g]):+.3f} "
                  f"fwd={np.mean([r['fwd'] for r in g]):.2f} "
                  f"LAND {sum(1 for r in g if r['land'])}/{len(g)}")

    # 4. and the object-grounding question: correct vs wrong colour, same pixels
    print("\nSame pixels (target present), correct vs wrong colour word:")
    for obj in OBJECTS:
        g = [r for r in ok if r["scene"] == "present" and r["object"] == (obj or "(none)")]
        if g:
            print(f"  {obj or '(none)':16} yaw={np.mean([r['yaw'] for r in g]):+.3f} "
                  f"fwd={np.mean([r['fwd'] for r in g]):.2f} "
                  f"LAND {sum(1 for r in g if r['land'])}/{len(g)}")
    print(f"\n[out] {OUT / f'ablation_{args.label}.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
