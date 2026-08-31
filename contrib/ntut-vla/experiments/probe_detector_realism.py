"""
How detectable is the car we already fly against? Measured, so a candidate mesh
can be judged against a number.

We are considering swapping the current car mesh (SM_Offroad_Body, forced to
/Game/Geometry/Materials/M_Orange because that is the only material verified to
render correctly) for a downloaded glTF, which the simulator can spawn from raw
bytes. The question that decides whether a candidate is usable is not "does it
look like a car to me" but "does OWL-ViT score it high enough that the servo loop
keeps a lock at the standoff distance we actually fly". That needs a baseline for
the mesh in the working demo, and the baseline has to come from the same frames,
the same model, and the same selection procedure the live flights used.

No simulator is needed: two recorded flights already hold 347 usable frames and
the scores the live detector assigned to them.

Three things make this harder than "run the detector on the saved jpgs"
-----------------------------------------------------------------------
1. The saved FPV frames are ANNOTATED. follow_vlm.annotate() draws a pure-green
   (0,255,0) box round the detection, a full-height green line through its
   centre, a white crosshair and five lines of white HUD text, then saves that.
   The live detector ran on the raw camera image. Scoring the annotated jpg
   measures a green rectangle, not a car, so every frame is reconstructed by
   masking the overlay colours and inpainting. Both variants are scored, and the
   annotated-minus-clean gap is reported, because that gap is the size of the
   mistake anyone repeating this without the step would make.

2. demo/out/vlm_stopgo/view/fpv holds 337 jpgs but the logged flight is 490
   ticks at one frame per 3 ticks = 163 frames. The other 174 are leftovers from
   two earlier, longer runs written to the same tag; frame 01011 has no flight
   log row at all. Frames are grouped by write time and only the last session,
   intersected with the tick numbers the flight log actually contains, is used.

3. OWL-ViT is queried ONE PHRASE PER FORWARD PASS. Batching several phrases
   changes the scores, because post-processing normalises across the text batch,
   and the live Grounder passes text=[[query]] - a single phrase. Batching here
   would produce numbers that do not describe the system.

Selection replicates Grounder._worker: top 12 boxes by score, stop below
--det-thresh, reject a box whose named colour covers less than colour_min of it,
rank the survivors by score * (0.25 + 0.75 * colour_fraction). The one live rule
not replicated is the temporal jump filter, which needs the previous accepted
box; noted in the output because it makes these numbers marginally more
permissive than flight.

Run:  python experiments/probe_detector_realism.py
Out:  experiments/out/detector_baseline/{samples.jsonl,summary.json}
      docs/RESULT-detector-baseline.md
"""
from __future__ import annotations

import argparse
import json
import math
import statistics as st
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from follow_vlm import COLOUR_HUE, colour_match, colour_word   # noqa: E402

DETECTOR_ID = "google/owlvit-base-patch32"
FLIGHTS = ["vlm_stopgo", "vlm_nfz_smooth"]
QUERIES = ["an orange car", "a car", "a vehicle", "a truck", "a blue car"]

# --det-thresh 0.008 in run_follow_vlm.ps1, colour_min the Grounder default.
LIVE_THRESH = 0.008
LIVE_COLOUR_MIN = 0.10
TOPK = 12

# want_width 0.1 of a 400 px frame: the apparent width the servo law holds. Any
# threshold quoted for a candidate mesh has to be quoted at this width, because
# this is the width the aircraft spends the flight trying to maintain.
WANT_W_FRAC = 0.10
SESSION_GAP_S = 600.0     # frames written more than 10 min apart are separate runs


def overlay_mask(rgb: np.ndarray) -> np.ndarray:
    """Pixels that annotate() painted on top of the camera image.

    The overlay is drawn in saturated pure colours that the Japanese City map
    does not otherwise contain, so colour is enough to find it. Green catches the
    box, the centre line and the box label: JPEG at quality 85 smears (0,255,0)
    to roughly (30,240,50) and leaves a halo, so the test is a wide green
    dominance margin rather than an exact match. Foliage in this map sits near
    (60,110,50), well inside the margin.

    White text and the crosshair cannot be found by colour alone - sky, concrete
    and road paint are also near-white - so those two tests are confined to the
    fixed rectangles annotate() writes into: five 11 px lines from (6,6), and the
    17 px crosshair at the frame centre. Inpainting a bright sky pixel from
    neighbouring sky costs nothing, so being loose inside those rectangles is
    safe.
    """
    r = rgb[..., 0].astype(np.int16)
    g = rgb[..., 1].astype(np.int16)
    b = rgb[..., 2].astype(np.int16)
    lum = 0.299 * r + 0.587 * g + 0.114 * b
    H, W = rgb.shape[:2]

    m = ((g - np.maximum(r, b)) > 55) & (g > 120)

    hud = np.zeros((H, W), bool)
    hud[0:64, 0:250] = True                    # 5 HUD lines + JPEG bleed
    m |= hud & (lum > 185)
    m |= hud & (r > 200) & (g < 160) & (b < 160)   # the red "HOLDING"/"correcting" line

    cross = np.zeros((H, W), bool)
    cross[H // 2 - 11:H // 2 + 11, W // 2 - 11:W // 2 + 11] = True
    m |= cross & (lum > 185)
    return m


def clean_frame(img: Image.Image):
    """Annotated frame -> best available reconstruction of the raw camera image.

    Telea inpainting, radius 3, on a 1 px dilation of the overlay mask. This is
    not perfect and the imperfection is directional: the green box is drawn ON
    the car's silhouette, so inpainting it blurs the car's own edge and pushes
    the score DOWN. That makes the clean-frame numbers a conservative floor. The
    check that the reconstruction is good enough is the paired comparison against
    the scores the live flight recorded on the true raw frames.
    """
    rgb = np.asarray(img.convert("RGB"))
    m = overlay_mask(rgb)
    md = cv2.dilate(m.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=1)
    bgr = cv2.inpaint(rgb[:, :, ::-1].copy(), md, 3, cv2.INPAINT_TELEA)
    return Image.fromarray(bgr[:, :, ::-1]), float(m.mean())


def load_flight(tag: str):
    """Frames that belong to THIS logged flight, with their telemetry.

    Returns a list of dicts, tick-ordered. `live` is the detection record the
    control loop was holding when the frame was written, taken from
    detections.jsonl via the flight log's det_seq, so it carries the full box.
    `det_age_s` says how stale that record was: the detector runs asynchronously
    at 3-4 Hz, so the score it produced was computed on a frame grabbed up to a
    third of a second before this one.
    """
    out = ROOT / "demo" / "out" / tag
    log = {}
    for line in (out / "flight_log.jsonl").open(encoding="utf-8"):
        r = json.loads(line)
        log[r["tick"]] = r
    dets = {}
    for line in (out / "detections.jsonl").open(encoding="utf-8"):
        r = json.loads(line)
        dets[r["seq"]] = r

    fpv = sorted((out / "view" / "fpv").glob("*.jpg"), key=lambda p: p.stat().st_mtime)
    sessions, cur, prev = [], [], None
    for p in fpv:
        t = p.stat().st_mtime
        if prev is not None and t - prev > SESSION_GAP_S:
            sessions.append(cur)
            cur = []
        cur.append(p)
        prev = t
    sessions.append(cur)
    keep = sessions[-1]                        # the run that wrote the logs

    rows, dropped_no_log = [], 0
    for p in sorted(keep, key=lambda q: int(q.stem)):
        tick = int(p.stem)
        if tick not in log:
            dropped_no_log += 1
            continue
        fr = log[tick]
        live = dets.get(fr.get("det_seq"))
        sep = None
        if fr.get("tgt_x") is not None:
            sep = math.hypot(fr["x"] - fr["tgt_x"], fr["y"] - fr["tgt_y"])
        rows.append({
            "tag": tag, "tick": tick, "path": p, "t": fr["t"],
            "alt": fr["up"], "sep_m": sep, "seen": fr["seen"],
            "mode": fr["mode"], "det_age_s": fr.get("det_age_s"),
            "live": (live or {}).get("det"),
        })
    return rows, {"jpgs_in_dir": len(fpv), "sessions": [len(s) for s in sessions],
                  "kept_session": len(keep), "dropped_no_log_row": dropped_no_log,
                  "used": len(rows)}


def spread(rows, n: int):
    """Evenly spaced across the flight, because apparent size tracks range.

    Taking the first n frames would sample one distance and one lighting angle
    and call it a baseline.
    """
    if n >= len(rows):
        return rows
    idx = np.linspace(0, len(rows) - 1, n).round().astype(int)
    return [rows[i] for i in sorted(set(idx.tolist()))]


class Owl:
    def __init__(self):
        import torch
        from transformers import OwlViTForObjectDetection, OwlViTProcessor
        self.torch = torch
        t0 = time.time()
        # local_files_only: the weights are already cached from the flights, and
        # this probe must not reach the network.
        self.proc = OwlViTProcessor.from_pretrained(DETECTOR_ID, local_files_only=True)
        self.model = (OwlViTForObjectDetection
                      .from_pretrained(DETECTOR_ID, local_files_only=True)
                      .to("cuda").eval())
        print(f"[owl] {DETECTOR_ID} loaded in {time.time()-t0:.0f}s "
              f"(VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB)", flush=True)

    def query(self, img: Image.Image, phrase: str) -> dict:
        """One phrase, one forward pass, then the live selection procedure."""
        torch = self.torch
        W, H = img.size
        inputs = self.proc(text=[[phrase]], images=img, return_tensors="pt").to("cuda")
        t = time.time()
        with torch.no_grad():
            out = self.model(**inputs)
        torch.cuda.synchronize()
        ms = (time.time() - t) * 1000.0
        res = self.proc.post_process_object_detection(
            out, threshold=0.0, target_sizes=torch.tensor([[H, W]]).to("cuda"))[0]
        sc, bx = res["scores"], res["boxes"]
        word = colour_word(phrase)

        order = sc.argsort(descending=True)
        top1 = None
        if len(order):
            i = int(order[0])
            x0, y0, x1, y1 = [float(v) for v in bx[i].tolist()]
            top1 = {"score": float(sc[i]), "cx": (x0 + x1) / 2, "cy": (y0 + y1) / 2,
                    "w": x1 - x0, "h": y1 - y0,
                    "colour": colour_match(img, (x0, y0, x1, y1), word)}

        sel, best = None, -1.0
        for i in order[:TOPK]:
            s = float(sc[i])
            if s < LIVE_THRESH:
                break
            x0, y0, x1, y1 = [float(v) for v in bx[i].tolist()]
            cm = colour_match(img, (x0, y0, x1, y1), word)
            if word is not None and cm < LIVE_COLOUR_MIN:
                continue
            combined = s * (0.25 + 0.75 * cm)
            if combined > best:
                best = combined
                sel = {"score": s, "cx": (x0 + x1) / 2, "cy": (y0 + y1) / 2,
                       "w": x1 - x0, "h": y1 - y0, "colour": cm,
                       "combined": combined}
        return {"infer_ms": ms, "top1": top1, "sel": sel,
                "n_above_thresh": int((sc >= LIVE_THRESH).sum())}


def q(v, p):
    return float(np.percentile(v, p)) if len(v) else None


def describe(vals):
    if not vals:
        return None
    return {"n": len(vals), "min": round(min(vals), 5),
            "p10": round(q(vals, 10), 5), "median": round(float(np.median(vals)), 5),
            "p90": round(q(vals, 90), 5), "max": round(max(vals), 5)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-per-flight", type=int, default=60)
    ap.add_argument("--out", default=str(ROOT / "experiments" / "out" / "detector_baseline"))
    ap.add_argument("--doc", default=str(ROOT / "docs" / "RESULT-detector-baseline.md"))
    args = ap.parse_args()

    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)

    flights, prov = {}, {}
    for tag in FLIGHTS:
        rows, p = load_flight(tag)
        prov[tag] = p
        flights[tag] = spread(rows, args.n_per_flight)
        print(f"[{tag}] {p['jpgs_in_dir']} jpgs, write sessions {p['sessions']}, "
              f"kept {p['kept_session']}, {p['dropped_no_log_row']} had no log row, "
              f"{p['used']} usable -> sampling {len(flights[tag])}", flush=True)

    owl = Owl()
    samples, npass = [], 0
    t_start = time.time()
    for tag, rows in flights.items():
        for k, fr in enumerate(rows):
            img_raw = Image.open(fr["path"]).convert("RGB")
            img_cln, mask_frac = clean_frame(img_raw)
            rec = {kk: fr[kk] for kk in
                   ("tag", "tick", "t", "alt", "sep_m", "seen", "mode", "det_age_s")}
            rec["frame"] = fr["path"].name
            rec["overlay_frac"] = round(mask_frac, 4)
            rec["live"] = fr["live"]
            rec["r"] = {}
            for phrase in QUERIES:
                rec["r"][phrase] = {"clean": owl.query(img_cln, phrase),
                                    "annotated": owl.query(img_raw, phrase)}
                npass += 2
            samples.append(rec)
            if (k + 1) % 10 == 0:
                print(f"  {tag} {k+1}/{len(rows)}  ({npass} passes, "
                      f"{time.time()-t_start:.0f}s)", flush=True)
    print(f"[owl] {npass} forward passes in {time.time()-t_start:.0f}s", flush=True)

    with (outdir / "samples.jsonl").open("w", encoding="utf-8") as fh:
        for r in samples:
            fh.write(json.dumps(r) + "\n")

    summary = analyse(samples, prov)
    (outdir / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    write_doc(Path(args.doc), summary, args)
    print(json.dumps(summary["headline"], indent=1))
    print(f"[out] {outdir/'samples.jsonl'}\n[out] {outdir/'summary.json'}\n[doc] {args.doc}")


def analyse(samples, prov):
    """Everything the decision needs, computed once so the doc and the return
    value cannot drift apart."""
    per = {}
    for phrase in QUERIES:
        for variant in ("clean", "annotated"):
            hits = [(s, s["r"][phrase][variant]) for s in samples]
            sel = [(s, h["sel"]) for s, h in hits if h["sel"] is not None]
            scores = [d["score"] for _, d in sel]
            widths = [d["w"] for _, d in sel]
            t1 = [h["top1"]["score"] for _, h in hits if h["top1"]]
            per[f"{phrase}|{variant}"] = {
                "query": phrase, "variant": variant, "n_frames": len(hits),
                "n_selected": len(sel),
                "hit_rate": round(len(sel) / max(1, len(hits)), 3),
                "score_selected": describe(scores),
                "width_px_selected": describe(widths),
                "score_top1_raw": describe(t1),
                "median_infer_ms": round(float(np.median(
                    [h["infer_ms"] for _, h in hits])), 1),
            }

    # score vs apparent width, on the query the decision rule is written for
    bands = [(0, 15), (15, 25), (25, 40), (40, 60), (60, 400)]
    vs_width = {}
    for phrase in ("a car", "an orange car"):
        rowsb = []
        pts = [(d["w"], d["score"]) for s in samples
               if (d := s["r"][phrase]["clean"]["sel"]) is not None]
        for lo, hi in bands:
            v = [sc for w, sc in pts if lo <= w < hi]
            rowsb.append({"band_px": f"{lo}-{hi}", "n": len(v),
                          "score_median": (round(float(np.median(v)), 5) if v else None),
                          "score_p10": (round(q(v, 10), 5) if v else None),
                          "score_min": (round(min(v), 5) if v else None)})
        rho = None
        if len(pts) > 8:
            from scipy.stats import spearmanr
            rho = round(float(spearmanr([p[0] for p in pts],
                                        [p[1] for p in pts]).statistic), 3)
        vs_width[phrase] = {"bands": rowsb, "spearman_width_vs_score": rho,
                            "n_points": len(pts)}

    # score vs range, to show that width really is the range proxy
    vs_range = []
    for lo, hi in [(0, 10), (10, 15), (15, 20), (20, 30), (30, 45), (45, 200)]:
        v = [(s["r"]["an orange car"]["clean"]["sel"], s["sep_m"]) for s in samples
             if s["sep_m"] is not None and lo <= s["sep_m"] < hi]
        got = [d["score"] for d, _ in v if d is not None]
        wid = [d["w"] for d, _ in v if d is not None]
        vs_range.append({"sep_band_m": f"{lo}-{hi}", "n": len(v),
                         "n_selected": len(got),
                         "score_median": (round(float(np.median(got)), 5) if got else None),
                         "width_median_px": (round(float(np.median(wid)), 1) if wid else None)})

    # cross-reference against what the live flight recorded
    xref = {}
    for gate, label in ((None, "all"), (0.15, "det_age<=0.15s")):
        pairs = []
        for s in samples:
            live = s["live"]
            if not live or not s["seen"]:
                continue
            if gate is not None and (s["det_age_s"] is None or s["det_age_s"] > gate):
                continue
            for variant in ("clean", "annotated"):
                d = s["r"]["an orange car"][variant]["sel"]
                if d is None:
                    continue
                pairs.append({
                    "variant": variant, "live": live["score"], "off": d["score"],
                    "dcx": d["cx"] - live["cx"], "dcy": d["cy"] - live["cy"],
                    "wr": d["w"] / max(1e-6, live["w"]),
                    "dist": math.hypot(d["cx"] - live["cx"], d["cy"] - live["cy"]),
                })
        for variant in ("clean", "annotated"):
            p = [x for x in pairs if x["variant"] == variant]
            if not p:
                continue
            lv = [x["live"] for x in p]
            ov = [x["off"] for x in p]
            rel = [(x["off"] - x["live"]) / max(1e-9, x["live"]) for x in p]
            rho = None
            if len(p) > 8:
                from scipy.stats import spearmanr
                rho = round(float(spearmanr(lv, ov).statistic), 3)
            xref[f"{label}|{variant}"] = {
                "n": len(p),
                "live_score_median": round(float(np.median(lv)), 5),
                "offline_score_median": round(float(np.median(ov)), 5),
                "rel_err_median_pct": round(100 * float(np.median(rel)), 1),
                "rel_err_p90_pct": round(100 * q(rel, 90), 1),
                "abs_rel_err_median_pct": round(
                    100 * float(np.median([abs(x) for x in rel])), 1),
                "spearman_live_vs_offline": rho,
                "box_centre_dist_px_median": round(float(np.median(
                    [x["dist"] for x in p])), 1),
                "frac_box_within_25px": round(
                    sum(1 for x in p if x["dist"] <= 25) / len(p), 3),
                "width_ratio_median": round(float(np.median([x["wr"] for x in p])), 3),
            }

    # the decision rule, at the width the servo law holds
    want_px = WANT_W_FRAC * 400
    rule = {}
    for phrase in ("a car", "an orange car"):
        v = [d["score"] for s in samples
             if (d := s["r"][phrase]["clean"]["sel"]) is not None
             and abs(d["w"] - want_px) <= 10]
        rule[phrase] = {
            "want_width_px": want_px, "band_px": "30-50", "n": len(v),
            "score_median": (round(float(np.median(v)), 5) if v else None),
            "score_p10": (round(q(v, 10), 5) if v else None),
            "score_min": (round(min(v), 5) if v else None),
            "margin_median_over_thresh": (round(float(np.median(v)) / LIVE_THRESH, 2)
                                          if v else None),
            "margin_p10_over_thresh": (round(q(v, 10) / LIVE_THRESH, 2) if v else None),
        }

    base = rule["a car"]
    accept = round(max(2.0 * LIVE_THRESH, 0.6 * base["score_p10"]), 4)
    headline = {
        "frames_scored": len(samples),
        "forward_passes": len(samples) * len(QUERIES) * 2,
        "live_det_thresh": LIVE_THRESH,
        "baseline_a_car_at_40px_median": base["score_median"],
        "baseline_a_car_at_40px_p10": base["score_p10"],
        "baseline_an_orange_car_at_40px_median": rule["an orange car"]["score_median"],
        "accept_score_at_40px": accept,
        "xref_clean_rel_err_median_pct": xref.get("det_age<=0.15s|clean", {}).get(
            "rel_err_median_pct"),
        "xref_annotated_rel_err_median_pct": xref.get("det_age<=0.15s|annotated", {}).get(
            "rel_err_median_pct"),
    }
    return {"provenance": prov, "per_query": per, "vs_width": vs_width,
            "vs_range": vs_range, "xref": xref, "rule": rule,
            "accept_score_at_40px": accept, "headline": headline}


def write_doc(path: Path, S: dict, args):
    P, R, X = S["provenance"], S["rule"], S["xref"]
    base = R["a car"]
    L = []
    A = L.append
    A("# OWL-ViT detection baseline for the current car mesh")
    A("")
    A(f"Measured {time.strftime('%Y-%m-%d')} by `experiments/probe_detector_realism.py`. "
      "No simulator involved: recorded flight frames only.")
    A("")
    A("## Why")
    A("")
    A("A downloaded glTF car can be spawned from raw bytes "
      "(`spawn_object_from_file`, format `gltf`), which makes swapping the mesh "
      "cheap. What is not cheap is discovering after the swap that OWL-ViT no "
      "longer grounds the word on it. The current mesh is "
      "`SM_Offroad_Body` + `/Game/Geometry/Materials/M_Orange` - the only "
      "material verified to render correctly - so it is the reference. This is "
      "how detectable it actually is.")
    A("")
    A("## Method")
    A("")
    A(f"* Model `{DETECTOR_ID}`, CUDA, `local_files_only` (no download).")
    A(f"* {S['headline']['frames_scored']} frames, evenly spread across two flights, "
      f"{S['headline']['forward_passes']} forward passes.")
    A("* **One phrase per forward pass** (`text=[[phrase]]`), as the live "
      "`Grounder` does. Batching phrases changes the scores.")
    A(f"* Selection replicates `Grounder._worker`: top {TOPK} by score, stop below "
      f"`--det-thresh {LIVE_THRESH}`, drop a box whose named colour covers "
      f"< {LIVE_COLOUR_MIN:.0%} of it, rank survivors by "
      "`score * (0.25 + 0.75 * colour_fraction)`. The live temporal jump filter "
      "is NOT replicated (it needs the previous accepted box), so these numbers "
      "are marginally more permissive than flight.")
    A("")
    A("### Two data problems that had to be fixed first")
    A("")
    A("**The saved FPV frames are annotated.** `follow_vlm.annotate()` draws a "
      "pure-green box, a full-height green centre line, a white crosshair and "
      "five lines of HUD text, and *that* is what is written to "
      "`view/fpv/*.jpg`. The live detector ran on the raw camera image. Every "
      "frame here is therefore reconstructed - overlay colours masked, Telea "
      "inpaint - and both variants are scored so the size of the contamination "
      "is on the record.")
    A("")
    for tag in FLIGHTS:
        p = P[tag]
        A(f"**`{tag}`**: {p['jpgs_in_dir']} jpgs in `view/fpv`, in write sessions of "
          f"{p['sessions']} frames. Only the last session was written by the run "
          f"that produced `metrics.json`; {p['dropped_no_log_row']} of its frames "
          f"had no flight-log row. **{p['used']} usable.**")
    A("")
    A("## Result 1 - per query, clean frames")
    A("")
    A("`hit` = fraction of frames where the live selection procedure returned a "
      "box. `score` is the raw detector score of the selected box.")
    A("")
    A("| query | hit | score min | p10 | median | p90 | max | box width px median | top-1 raw median |")
    A("|---|---|---|---|---|---|---|---|---|")
    for phrase in QUERIES:
        d = S["per_query"][f"{phrase}|clean"]
        s, w, t1 = d["score_selected"], d["width_px_selected"], d["score_top1_raw"]
        if s is None:
            A(f"| `{phrase}` | {d['hit_rate']:.3f} | - | - | - | - | - | - | "
              f"{t1['median'] if t1 else '-'} |")
            continue
        A(f"| `{phrase}` | {d['hit_rate']:.3f} | {s['min']} | {s['p10']} | "
          f"**{s['median']}** | {s['p90']} | {s['max']} | {w['median']} | {t1['median']} |")
    A("")
    A("### The annotation contamination, quantified")
    A("")
    A("| query | median score, clean | median score, annotated | inflation |")
    A("|---|---|---|---|")
    for phrase in QUERIES:
        c = S["per_query"][f"{phrase}|clean"]["score_selected"]
        a = S["per_query"][f"{phrase}|annotated"]["score_selected"]
        if not c or not a:
            A(f"| `{phrase}` | {c['median'] if c else '-'} | "
              f"{a['median'] if a else '-'} | - |")
            continue
        A(f"| `{phrase}` | {c['median']} | {a['median']} | "
          f"x{a['median']/c['median']:.2f} |")
    A("")
    A("## Result 2 - score against apparent width (the range proxy)")
    A("")
    for phrase in ("a car", "an orange car"):
        v = S["vs_width"][phrase]
        A(f"`{phrase}` - Spearman(width, score) = **{v['spearman_width_vs_score']}** "
          f"over {v['n_points']} boxes")
        A("")
        A("| box width px | n | score median | p10 | min |")
        A("|---|---|---|---|---|")
        for b in v["bands"]:
            A(f"| {b['band_px']} | {b['n']} | {b['score_median']} | "
              f"{b['score_p10']} | {b['score_min']} |")
        A("")
    A("### and against true range, from the flight log")
    A("")
    A("| separation m | n frames | n with a box | score median | width median px |")
    A("|---|---|---|---|---|")
    for b in S["vs_range"]:
        A(f"| {b['sep_band_m']} | {b['n']} | {b['n_selected']} | "
          f"{b['score_median']} | {b['width_median_px']} |")
    A("")
    A("## Result 3 - do these numbers agree with the live flight?")
    A("")
    A("`detections.jsonl` holds the score the live detector assigned. The "
      "detector ran asynchronously at 3-4 Hz, so the record current at a frame "
      "was computed on a frame grabbed up to a third of a second earlier; the "
      "tight rows below restrict the comparison to frames where that staleness "
      "was under 0.15 s.")
    A("")
    A("| subset | variant | n | live median | offline median | median rel. err | median abs rel. err | Spearman | box centre dist px | within 25 px |")
    A("|---|---|---|---|---|---|---|---|---|---|")
    for key in ("all|clean", "all|annotated",
                "det_age<=0.15s|clean", "det_age<=0.15s|annotated"):
        d = X.get(key)
        if not d:
            continue
        sub, var = key.split("|")
        A(f"| {sub} | {var} | {d['n']} | {d['live_score_median']} | "
          f"{d['offline_score_median']} | {d['rel_err_median_pct']:+.1f}% | "
          f"{d['abs_rel_err_median_pct']:.1f}% | "
          f"{d['spearman_live_vs_offline']} | {d['box_centre_dist_px_median']} | "
          f"{d['frac_box_within_25px']:.3f} |")
    A("")
    A("## Result 4 - THE DECISION RULE")
    A("")
    A(f"The servo law holds `want_width = {WANT_W_FRAC}` of a 400 px frame, so "
      f"**{int(WANT_W_FRAC*400)} px is the apparent width the aircraft spends the "
      "flight trying to maintain**. That is the width any threshold has to be "
      "quoted at; a score measured on a 90 px box says nothing about whether the "
      "lock survives at standoff.")
    A("")
    A("Measured on the current mesh in the 30-50 px band:")
    A("")
    A("| query | n | score median | p10 | min | median / 0.008 | p10 / 0.008 |")
    A("|---|---|---|---|---|---|---|")
    for phrase in ("a car", "an orange car"):
        r = R[phrase]
        A(f"| `{phrase}` | {r['n']} | {r['score_median']} | {r['score_p10']} | "
          f"{r['score_min']} | x{r['margin_median_over_thresh']} | "
          f"x{r['margin_p10_over_thresh']} |")
    A("")
    A("> **Rule.** A candidate mesh is acceptable if OWL-ViT scores it at least "
      f"**{S['accept_score_at_40px']}** for the bare noun phrase **`a car`** on a "
      f"box of apparent width **{int(WANT_W_FRAC*400)} px** (30-50 px band) in a "
      "400x225 frame from the FrontCamera geometry, on at least 90% of sampled "
      "frames.")
    A("")
    A("Where the number comes from, and why not simply 0.008:")
    A("")
    A(f"* `--det-thresh 0.008` is the *floor the code will accept*, not a quality "
      f"bar. At 40 px the current mesh sits at x{base['margin_median_over_thresh']} "
      f"that floor at the median and x{base['margin_p10_over_thresh']} at the 10th "
      "percentile. A candidate that merely clears 0.008 would be running with "
      "essentially no margin, and the p10 is what decides whether the lock "
      "survives the bad frames rather than the good ones.")
    A(f"* The rule takes 60% of the current mesh's p10 "
      f"({base['score_p10']}), floored at 2x the flight threshold. 60% is a "
      "deliberate concession: the point of a new mesh is a car that looks more "
      "like a real car, and we would accept somewhat worse detectability for "
      "that - but not a mesh that has to be carried by the threshold.")
    A("* Quote `a car`, not `an orange car`, because OWL-ViT does not "
      "discriminate colour (measured: `a car` and `a blue car` return an "
      "identical box on a real frame). The colour word's contribution is the HSV "
      "check, which is a separate and independent test of the candidate's "
      "material. **A candidate that passes this rule can still fail the flight** "
      f"if its material does not give colour_match >= {LIVE_COLOUR_MIN:.0%}; check both.")
    A("")
    A("### How to test a candidate")
    A("")
    A("Render or photograph the candidate at the geometry the FrontCamera gives - "
      "400x225, 90 deg horizontal FOV, pitched 20 deg down - framed so the car's "
      f"box is about {int(WANT_W_FRAC*400)} px wide, then run exactly the query "
      "path this script uses: one phrase per forward pass, "
      f"`post_process_object_detection(threshold=0.0)`, top {TOPK}, "
      "`score * (0.25 + 0.75 * colour)`. Do not batch the phrases and do not "
      "score an annotated frame.")
    A("")
    A("## Caveats")
    A("")
    A("* The inpaint is imperfect and the error has a direction: the green box is "
      "drawn ON the car's silhouette, so removing it blurs the car's own edge. "
      "The clean-frame scores are a floor, not a point estimate. Result 3 is what "
      "bounds the error.")
    A("* The live temporal jump filter is not replicated, so offline hit rates "
      "for the loose queries (`a car`, `a vehicle`, `a truck`) overstate what "
      "flight would accept - in flight a box far from the last one is rejected.")
    A("* Both flights are the same map, the same time of day, one car, one "
      "material. This is a baseline for *that* mesh under *those* conditions, "
      "which is all the swap decision needs.")
    A("")
    A("Raw per-frame records: `experiments/out/detector_baseline/samples.jsonl`, "
      "aggregates `summary.json`.")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(L) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
