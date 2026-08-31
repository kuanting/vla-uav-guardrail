"""
Where does OWL-ViT's 287 ms go?

The question
------------
In flight, `infer_ms` reads 286-287 ms median against 34-56 ms measured on an
idle GPU. Three configurations were flown to find out why
(docs/data/chase_resolution_study.json):

    Chase 1280x720, window 1280x720   det_hz 2.94
    Chase  960x540, window 1280x720   det_hz 3.66   infer 286 ms
    Chase  960x540, window  960x540   det_hz 3.62   infer 287 ms

Shrinking the recording camera helped by about 25 percent. Shrinking the
simulator window did nothing at all. A cost that barely moves across a 2.4x
change in capture pixels and a 1.8x change in window pixels is not contention -
it is fixed work done once per inference. But nothing has yet said WHICH work.

What `infer_ms` actually covers (demo/follow_vlm.py:960-965):

    t = time.time()
    inputs = proc(text=queries, images=img, return_tensors="pt").to("cuda")
    with torch.no_grad():
        out = model(**inputs)
    torch.cuda.synchronize()
    ms = (time.time() - t) * 1000

CPU preprocessing, the host-to-device copy and the forward pass, as one number.
The frame RPC and post-processing sit outside it.

Two hypotheses, written down before the measurement so the result cannot be read
selectively:

  H1  The text branch is constant work repeated about four times a second.
      `queries` never changes during a flight, yet the processor re-tokenises it
      and the model re-runs its text encoder on every single frame.

  H2  The image preprocessing is a CPU cost. OWL-ViT base-patch32 wants
      768x768; the frame is 400x225, so every inference resizes on the CPU
      through PIL inside `proc`.

Either could be most of it, or neither, in which case the honest conclusion is
that det_hz >= 4 needs a different detector rather than tuning.

This runs offline against a recorded frame, so it needs no simulator, costs no
flight time, and cannot disturb a measurement by competing with one.

Usage:
    python experiments/profile_owlvit.py
    python experiments/profile_owlvit.py --iters 50 --query "a yellow car"
"""
from __future__ import annotations

import argparse
import statistics as st
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))

DETECTOR_ID = "google/owlvit-base-patch32"


# The detector's real input. FrontCamera is 400x225 and every measured number in
# the repository is conditioned on it. The recorded frames under view/fpv/ are
# NOT this size: the recorder upscales to the video panel height before drawing
# the overlay, so they come back at 960x540. Profiling one of those measures a
# resize the detector never performs.
DETECTOR_INPUT = (400, 225)


def find_frame() -> Path:
    for tag in ("demo_follow", "demo_traffic", "demo_nfz"):
        d = ROOT / "demo" / "out" / tag / "view" / "fpv"
        if d.is_dir():
            frames = sorted(d.glob("*.jpg"))
            if frames:
                return frames[len(frames) // 2]
    raise SystemExit("no recorded frame found under demo/out/*/view/fpv/")


def bench(label: str, fn, iters: int, warmup: int = 5) -> dict:
    import torch
    for _ in range(warmup):
        fn()
    torch.cuda.synchronize()
    times = []
    for _ in range(iters):
        t = time.perf_counter()
        fn()
        torch.cuda.synchronize()
        times.append((time.perf_counter() - t) * 1000.0)
    return {"label": label, "median": st.median(times),
            "mean": sum(times) / len(times), "min": min(times), "max": max(times)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=30)
    ap.add_argument("--query", default="a yellow car")
    ap.add_argument("--frame", default=None)
    args = ap.parse_args()

    import torch
    from PIL import Image
    from transformers import OwlViTForObjectDetection, OwlViTProcessor

    frame = Path(args.frame) if args.frame else find_frame()
    img = Image.open(frame).convert("RGB")
    was = (img.width, img.height)
    if was != DETECTOR_INPUT:
        img = img.resize(DETECTOR_INPUT, Image.LANCZOS)
    print(f"frame  : {frame.name}  {was[0]}x{was[1]} -> {img.width}x{img.height} "
          f"(FrontCamera size, which is what the detector actually receives)")
    print(f"device : {torch.cuda.get_device_name(0)}")
    print(f"query  : {args.query!r}")

    t0 = time.time()
    proc = OwlViTProcessor.from_pretrained(DETECTOR_ID)
    model = OwlViTForObjectDetection.from_pretrained(DETECTOR_ID).to("cuda").eval()
    print(f"loaded : {time.time() - t0:.0f}s, "
          f"VRAM {torch.cuda.memory_allocated() / 1e9:.2f} GB\n")

    queries = [[args.query]]
    W, H = img.size
    rows = []

    # --- the whole thing, exactly as the flight measures it -----------------
    def full():
        inputs = proc(text=queries, images=img, return_tensors="pt").to("cuda")
        with torch.no_grad():
            model(**inputs)

    rows.append(bench("whole inference (as flown)", full, args.iters))

    # --- the parts ----------------------------------------------------------
    def preprocess_cpu():
        proc(text=queries, images=img, return_tensors="pt")

    rows.append(bench("  processor, CPU only", preprocess_cpu, args.iters))

    cpu_inputs = proc(text=queries, images=img, return_tensors="pt")

    def to_cuda():
        cpu_inputs.to("cuda")

    rows.append(bench("  host to device copy", to_cuda, args.iters))

    gpu_inputs = cpu_inputs.to("cuda")

    def forward():
        with torch.no_grad():
            model(**gpu_inputs)

    rows.append(bench("  forward pass", forward, args.iters))

    out_once = None
    with torch.no_grad():
        out_once = model(**gpu_inputs)
    sizes = torch.tensor([[H, W]]).to("cuda")

    def postprocess():
        proc.post_process_object_detection(out_once, threshold=0.0,
                                           target_sizes=sizes)

    rows.append(bench("  post-process (outside infer_ms)", postprocess, args.iters))

    # --- H1: is the text branch constant work, repeated? --------------------
    def image_only_preprocess():
        proc(images=img, return_tensors="pt")

    rows.append(bench("H1  processor, image only (no text)",
                      image_only_preprocess, args.iters))

    # --- H2: how much of the CPU cost is the resize to 768x768? -------------
    def resize_only():
        img.resize((768, 768), Image.BILINEAR)

    rows.append(bench(f"H2  PIL resize {img.width}x{img.height} -> 768x768",
                      resize_only, args.iters))

    # --- H3: the forward pass is what the simulator starves -----------------
    # In flight the split is 44 ms preprocessing and 242 ms forward, against
    # 26.7 and 36.2 idle: the CPU side inflates 1.6x and the GPU side 6.7x. So
    # the only lever worth pulling is the cost of the forward pass itself.
    # Half precision is the cheapest one available.
    half = OwlViTForObjectDetection.from_pretrained(DETECTOR_ID).half().to("cuda").eval()
    half_inputs = {k: (v.half() if v.dtype == torch.float32 else v)
                   for k, v in gpu_inputs.items()}

    def forward_fp16():
        with torch.no_grad():
            half(**half_inputs)

    rows.append(bench("H3  forward pass, fp16", forward_fp16, args.iters))

    width = max(len(r["label"]) for r in rows)
    print(f"{'stage':<{width}} {'median':>9} {'mean':>9} {'min':>8} {'max':>8}")
    print("-" * (width + 38))
    for r in rows:
        print(f"{r['label']:<{width}} {r['median']:>8.1f}ms {r['mean']:>8.1f}ms "
              f"{r['min']:>7.1f}ms {r['max']:>7.1f}ms")

    whole = rows[0]["median"]
    pre, cpy, fwd = rows[1]["median"], rows[2]["median"], rows[3]["median"]
    img_only, resize = rows[5]["median"], rows[6]["median"]
    parts = pre + cpy + fwd

    print()
    print(f"  parts sum to {parts:.1f} ms against {whole:.1f} ms measured whole "
          f"({100 * parts / whole:.0f} %)")
    print(f"  processor {100 * pre / whole:.0f} %   copy {100 * cpy / whole:.0f} %   "
          f"forward {100 * fwd / whole:.0f} %")
    print()
    text_share = pre - img_only
    print(f"  H1 text branch inside the processor: {text_share:.1f} ms "
          f"({100 * text_share / whole:.0f} % of an inference), repeated every frame "
          f"for a query that never changes")
    print(f"  H2 the 768x768 resize alone: {resize:.1f} ms "
          f"({100 * resize / whole:.0f} % of an inference)")
    fp16 = rows[7]["median"]
    print(f"  H3 forward in fp16: {fp16:.1f} ms against {fwd:.1f} ms in fp32 "
          f"({fwd / fp16:.2f}x)")
    print()
    print("  Measured in flight, for comparison (demo_follow, 212 inferences):")
    print("      total 286.8 ms = preprocessing 44.0 (15 %) + forward 242.6 (85 %)")
    print("      preprocessing inflates 1.6x over idle, the forward pass 6.7x.")
    print()
    print("  READ: the simulator starves the GPU, not the CPU. Preprocessing is a")
    print("        large share ONLY when idle; in flight it is 15 %. Caching the")
    print("        text branch or speeding up the resize would buy nothing.")
    print("        The levers that touch the 85 % are cheaper scene rendering and")
    print("        a cheaper forward pass.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
