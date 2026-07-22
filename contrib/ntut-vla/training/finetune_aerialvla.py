"""
QLoRA fine-tuning of AerialVLA on our self-collected AirSimNH flight data.

Continues training the released AerialVLA LoRA adapter (r=64 on the LLM +
full projector) on top of the 4-bit-quantized openvla-7b base — the same
QLoRA recipe the adapter was born from, now specialized to OUR worlds/tasks.

Text target format matches the adapter's inference parser exactly:
    prompt : "<image>\n{instruction}\nAction: "
    target : "{bin_fwd} {bin_down} {bin_yaw}"   (99 bins, AerialVLA NORM ranges)
Loss is computed on the action tokens only (prompt tokens masked to -100).

Outputs:
    D:/models/aerialvla-ft/<run>/         adapter checkpoints (per epoch)
    training/ft_loss_log.csv              step,loss,lr,epoch (for the charts)

Run (vla-real env):  python training/finetune_aerialvla.py --epochs 2
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "dataset" / "aerialvla_ft"
BASE_ID = "D:/models/openvla-7b"
LORA_ID = "D:/models/aerialvla-lora/aero_vla"
FT_OUT = Path("D:/models/aerialvla-ft")
LOSS_CSV = ROOT / "training" / "ft_loss_log.csv"

NUM_BINS = 99
NORM = {"fwd": (0.0, 5.0), "down": (-5.0, 5.0), "yaw": (-1.1, 1.1)}


def quantize(v: float, axis: str) -> int:
    vmin, vmax = NORM[axis]
    b = round((v - vmin) / (vmax - vmin) * (NUM_BINS - 1))
    return int(max(0, min(NUM_BINS - 1, b)))


def main() -> int:
    global DATA
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=2)
    ap.add_argument("--lr", type=float, default=5e-5)
    ap.add_argument("--accum", type=int, default=8)
    ap.add_argument("--max-samples", type=int, default=0, help="0 = all")
    ap.add_argument("--run", default="run1")
    ap.add_argument("--data", default="")
    ap.add_argument("--adapter", default=LORA_ID,
                    help="starting adapter (original, or a previous checkpoint)")
    args = ap.parse_args()
    if args.data:
        DATA = Path(args.data)

    import torch
    from peft import PeftModel
    from PIL import Image
    from transformers import (AutoImageProcessor, AutoModelForVision2Seq,
                              AutoTokenizer, BitsAndBytesConfig)

    samples = json.loads((DATA / "samples.json").read_text(encoding="utf-8"))
    random.seed(0)
    random.shuffle(samples)
    if args.max_samples:
        samples = samples[: args.max_samples]
    print(f"[ft] {len(samples)} samples, {args.epochs} epochs, "
          f"accum {args.accum}, lr {args.lr}")

    tok = AutoTokenizer.from_pretrained(BASE_ID, trust_remote_code=True)
    imgproc = AutoImageProcessor.from_pretrained(BASE_ID, trust_remote_code=True)

    base = AutoModelForVision2Seq.from_pretrained(
        BASE_ID,
        attn_implementation="eager",
        torch_dtype=torch.bfloat16,
        quantization_config=BitsAndBytesConfig(
            load_in_4bit=True, bnb_4bit_compute_dtype=torch.bfloat16,
            bnb_4bit_quant_type="nf4", llm_int8_skip_modules=["projector"]),
        device_map={"": 0},
        low_cpu_mem_usage=True,
        trust_remote_code=True,
    )
    model = PeftModel.from_pretrained(base, args.adapter, is_trainable=True)
    model.print_trainable_parameters()
    model.gradient_checkpointing_enable()
    model.enable_input_require_grads()
    model.train()

    params = [p for p in model.parameters() if p.requires_grad]
    opt = torch.optim.AdamW(params, lr=args.lr, weight_decay=0.0)
    total_steps = math.ceil(len(samples) * args.epochs / args.accum)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=total_steps)

    new_csv = not LOSS_CSV.exists()
    fcsv = LOSS_CSV.open("a", newline="", encoding="utf-8")
    w = csv.writer(fcsv)
    if new_csv:
        w.writerow(["run", "epoch", "step", "loss", "lr"])

    def encode(s):
        img = Image.open(DATA / "images" / s["traj_rel_dir"] / s["img_name"]).convert("RGB")
        pv = imgproc(images=img, return_tensors="pt")["pixel_values"]
        lb = s["label"]
        act = f"{quantize(lb['fwd'], 'fwd')} {quantize(lb['down'], 'down')} " \
              f"{quantize(lb['yaw'], 'yaw')}"
        prompt = s["instruction"] + "\nAction: "
        p_ids = tok(prompt, return_tensors="pt")["input_ids"][0]
        a_ids = tok(act + tok.eos_token, add_special_tokens=False,
                    return_tensors="pt")["input_ids"][0]
        input_ids = torch.cat([p_ids, a_ids])
        labels = torch.cat([torch.full_like(p_ids, -100), a_ids.clone()])
        return input_ids, labels, pv

    step, running = 0, 0.0
    t0 = time.time()
    for epoch in range(args.epochs):
        random.shuffle(samples)
        opt.zero_grad()
        for i, s in enumerate(samples):
            try:
                input_ids, labels, pv = encode(s)
            except FileNotFoundError:
                continue
            out = model(
                input_ids=input_ids.unsqueeze(0).to("cuda"),
                labels=labels.unsqueeze(0).to("cuda"),
                pixel_values=pv.to("cuda", dtype=torch.bfloat16),
            )
            loss = out.loss / args.accum
            loss.backward()
            running += out.loss.item()
            if (i + 1) % args.accum == 0:
                torch.nn.utils.clip_grad_norm_(params, 1.0)
                opt.step()
                sched.step()
                opt.zero_grad()
                step += 1
                if step % 10 == 0:
                    avg = running / (10 * args.accum)
                    running = 0.0
                    el = time.time() - t0
                    w.writerow([args.run, epoch, step, f"{avg:.4f}",
                                f"{sched.get_last_lr()[0]:.2e}"])
                    fcsv.flush()
                    print(f"[ft] ep {epoch} step {step}/{total_steps} "
                          f"loss {avg:.4f} | {el/60:.1f} min", flush=True)
        ck = FT_OUT / args.run / f"epoch{epoch}"
        ck.mkdir(parents=True, exist_ok=True)
        model.save_pretrained(str(ck))
        print(f"[ft] saved {ck}", flush=True)

    fcsv.close()
    print(f"[ft] DONE in {(time.time()-t0)/60:.1f} min -> {FT_OUT / args.run}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
