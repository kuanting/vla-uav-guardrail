"""
Settle the material question by measuring what the camera actually renders.

Every colour claim in this project currently rests on eyeballing one or two
frames. The record says only /Game/Geometry/Materials/M_Orange renders as its
name: a blue material came out INVISIBLE, "Yellow" came out pale grey, and
MI_Emissive_Red came out pale pink-white. That is the single binding constraint
on the whole follow-a-named-object demo — a city of cars that all render orange
tests nothing, because the colour word in the instruction has nothing to bind to.

This script replaces the eyeballing with numbers. For each candidate material it
spawns the car mesh at a known pose in front of the camera, paints it, captures
one FrontCamera frame, and reports two things that both have to be true for a
material to be usable:

    1. does it render the colour its name claims?
       Measured on the pixels that the mesh actually changed (a before/after
       difference against the same view with no car), so road and sky cannot
       dilute the reading. Reported as mean HSV plus the fraction of pixels
       falling in each COLOUR_HUE band from demo/follow_vlm.py — the same table
       the flight's colour gate uses, imported rather than copied, so a change
       there changes this measurement too.

    2. does OWL-ViT still find the car once it is painted?
       Run with "a car" and with "a <colour> car" on every frame, plus the same
       queries on the empty-street baseline frame as a false-positive floor.
       A material that renders a perfect blue but makes the detector lose the car
       is not usable, and nothing measured so far would have caught that.

Two camera poses per material, because the two questions want different
geometry. Close and low (4 m altitude, 8 m range) puts ~2000 px of car in frame,
which is what a colour measurement needs. High and far (9 m, 22 m) is the actual
flight condition — the measured follow range, where a 3.7 m car is ~31 px wide —
and that is where the detector score means something.

Why the mesh region is measured by frame differencing rather than by trusting
the detector: the detector is one of the things under test. The car's projected
box is also computed analytically from the drone pose, the car pose and the
camera intrinsics (400x225, 90 deg HFOV, pitched 20 deg down => fx = fy = 200),
and used both as the search window for the difference and as a stand-in for the
detector's box when the detector finds nothing. An invisible material therefore
still produces a measurement: "0 changed pixels inside the box where the car
provably is" is the finding, not a crash.

The car is destroyed and respawned for every material instead of being
teleported, because "SetObjectPose ... not movable" appears after many sim
restarts and a frozen car would silently corrupt every later row. Every sim call
goes through _rpc(), which records the failure and carries on.

Run (sim already up, single exclusive resource — this script does not launch it):
    python experiments/survey_materials.py
    python experiments/survey_materials.py --materials auto --max-auto 16
    python experiments/survey_materials.py \
        --materials /Game/Geometry/Materials/M_Orange /Game/Geometry/Materials/M_Blue

Out (default experiments/out/materials/):
    assets_all.txt        every name list_assets('.*') returned, one per line
    assets_vehicle.txt    the vehicle-like subset
    assets_material.txt   the material-like subset
    frames/*.png          one capture per material per pose, plus the baselines
                          and _annot.png copies with the boxes drawn
    survey.json           every number below, unrounded
    VERDICT.md            the ranked verdict table
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import moving_car                                                    # noqa: E402
from follow_vlm import (                                             # noqa: E402
    COLOUR_ACHROMATIC, COLOUR_HUE, colour_match,
)
from semantic_demo import SemanticObs, quat_yaw                      # noqa: E402

SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_semantic.jsonc"
DETECTOR_ID = "google/owlvit-base-patch32"

# FrontCamera, from demo/pas_config/robot_semantic_quad.jsonc: 400x225, 90 deg
# horizontal FOV, mounted 0.3 m forward of the frame origin and pitched 20 deg
# down. fx = (W/2)/tan(hfov/2) = 200, and atan(112.5/200) = 29.36 deg recovers
# the measured 29.4 deg vertical half-FOV, so square pixels at fx = fy = 200 is
# the right model.
FRONT_W, FRONT_H = 400, 225
FRONT_HFOV_DEG = 90.0
FRONT_PITCH_DEG = 20.0          # nose-down, positive
CAM_FWD_M = 0.3
FX = (FRONT_W / 2.0) / math.tan(math.radians(FRONT_HFOV_DEG) / 2.0)

# Candidates worth spending sim time on. M_Orange is the control: it is the one
# material with a verified correct render, so if its row comes out wrong the
# measurement itself is broken and no other row can be trusted. The two-name
# variants are the pair moving_car.CarSpec already tries. MI_Emissive_Red is
# included precisely because it is on record as rendering pale pink-white — a
# known-bad row is as useful as a known-good one for validating the method.
DEFAULT_MATERIALS = [
    "/Game/Geometry/Materials/M_Orange",
    "/Game/Geometry/Materials/Orange",
    "/Game/Geometry/Materials/M_Red",
    "/Game/Geometry/Materials/MI_Emissive_Red",
    "/Game/Geometry/Materials/M_Blue",
    "/Game/Geometry/Materials/M_Green",
    "/Game/Geometry/Materials/M_Yellow",
    "/Game/Geometry/Materials/M_White",
    "/Game/Geometry/Materials/M_Black",
]
CONTROL_MATERIAL = "/Game/Geometry/Materials/M_Orange"

VEHICLE_RE = re.compile(
    r"car|vehicle|truck|van|bus|sedan|suv|coupe|offroad|rover|jeep|taxi|wheel"
    r"|chassis|body|trailer|motor|bike", re.I)
MATERIAL_RE = re.compile(r"(^|/)(m|mi|mat)[_\-]|material|colou?r|paint|texture", re.I)


def colour_words() -> list:
    """Every colour word the flight's gate can be asked about, deduplicated.

    COLOUR_ACHROMATIC maps both "grey" and "gray" onto the same test, so taking
    its keys directly would report the identical fraction twice.
    """
    words = list(COLOUR_HUE)
    seen = set()
    for word, kind in COLOUR_ACHROMATIC.items():
        if kind not in seen:
            seen.add(kind)
            words.append(word)
    return words


def phrase(word: str) -> str:
    """"orange" -> "an orange car". The article matters: OWL-ViT is a text
    encoder, and "a orange car" is not a string English text ever contains."""
    art = "an" if word[:1].lower() in "aeiou" else "a"
    return f"{art} {word} car"


def intended_colour(material: str | None) -> str | None:
    """The colour a material's NAME claims, which is the thing under test.

    Matched longest-first so "M_LightGrey" does not resolve on a shorter word
    that happens to be a substring of another.
    """
    if not material:
        return None
    low = material.lower()
    for word in sorted(colour_words(), key=len, reverse=True):
        if word in low:
            return word
    return None


def slug(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", text).strip("_") or "none"


def asset_name(entry) -> str:
    """list_assets entries have been seen as plain strings; tolerate dicts."""
    if isinstance(entry, dict):
        for key in ("name", "asset", "path", "asset_path"):
            if key in entry:
                return str(entry[key])
        return json.dumps(entry, sort_keys=True)
    return str(entry)


def _rpc(label: str, fn, *a, **kw):
    """Call the sim and survive it failing.

    The known one is "SetObjectPose ... not movable", which shows up after many
    sim restarts, but spawn and destroy fail in their own ways too. A survey that
    dies on row four has cost the human two minutes of sim startup for nothing,
    so every failure is recorded and the run continues.
    """
    try:
        return fn(*a, **kw), None
    except Exception as exc:
        msg = f"{type(exc).__name__}: {exc}"
        print(f"[rpc] {label} failed -- {msg}")
        return None, msg


# ------------------------------------------------------------------ geometry --

def project(points, cam, yaw: float, pitch_deg: float = FRONT_PITCH_DEG):
    """World points (x=North, y=East, z=up) -> pixel (u, v) in the front frame.

    Pinhole through a camera rotated by the drone's yaw and then pitched down.
    Returns (u, v, in_front) per point; in_front is False behind the image plane,
    where the projection is meaningless rather than merely off-screen.
    """
    cx, cy, cup = cam
    cyaw, syaw = math.cos(yaw), math.sin(yaw)
    p = math.radians(pitch_deg)
    cp, sp = math.cos(p), math.sin(p)
    out = []
    for (x, y, z) in points:
        dn, de, dup = x - cx, y - cy, z - cup
        fwd = dn * cyaw + de * syaw          # along the nose
        rgt = -dn * syaw + de * cyaw         # out the right side
        dwn = -dup                           # below the camera
        z_c = fwd * cp + dwn * sp            # along the optical axis
        y_c = -fwd * sp + dwn * cp           # down in the image
        x_c = rgt
        if z_c <= 0.1:
            out.append((float("nan"), float("nan"), False))
            continue
        out.append((FRONT_W / 2.0 + FX * x_c / z_c,
                    FRONT_H / 2.0 + FX * y_c / z_c, True))
    return out


def car_box_px(cam, yaw: float, car_xy, car_heading: float, spec):
    """Where the car provably is in the image, from poses alone.

    Projects all eight corners of the mesh's bounding box (dimensions from
    CarSpec, which took them from the sim's own 3d bounding box) and takes the
    extent. Independent of the detector, so it can be used to judge it.
    """
    hl, hw = spec.length_m / 2.0, spec.width_m / 2.0
    ch, sh = math.cos(car_heading), math.sin(car_heading)
    corners = []
    for dl in (-hl, hl):
        for dw in (-hw, hw):
            x = car_xy[0] + dl * ch - dw * sh
            y = car_xy[1] + dl * sh + dw * ch
            for z in (0.0, spec.height_m):
                corners.append((x, y, z))
    pts = project(corners, cam, yaw)
    us = [u for u, v, ok in pts if ok]
    vs = [v for u, v, ok in pts if ok]
    if len(us) < 4:
        return None
    return (min(us), min(vs), max(us), max(vs))


def clip_box(box, w: int, h: int):
    x0, y0, x1, y1 = box
    return (max(0.0, min(x0, w - 1.0)), max(0.0, min(y0, h - 1.0)),
            max(0.0, min(x1, w - 1.0)), max(0.0, min(y1, h - 1.0)))


def iou(a, b) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    iw = max(0.0, min(ax1, bx1) - max(ax0, bx0))
    ih = max(0.0, min(ay1, by1) - max(ay0, by0))
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
    return float(inter / max(1e-6, ua))


# ------------------------------------------------------------------ measuring --

def mesh_mask(cur, base, box, diff_thresh: int, margin_px: int, min_px: int):
    """Which pixels the car occupies, by differencing against the same view
    without it.

    Station keeping is not perfect, so a whole-frame difference also lights up
    every high-contrast edge in the scene by a pixel or two. Restricting the
    difference to the analytically projected box (plus a margin) removes all of
    that, and taking the largest connected component inside the window removes
    the rest. Returns (bool mask, n_px, window).
    """
    import cv2
    h, w = cur.shape[:2]
    x0, y0, x1, y1 = [int(round(v)) for v in box]
    x0 = max(0, x0 - margin_px); y0 = max(0, y0 - margin_px)
    x1 = min(w, x1 + margin_px + 1); y1 = min(h, y1 + margin_px + 1)
    mask = np.zeros((h, w), bool)
    if x1 - x0 < 2 or y1 - y0 < 2:
        return mask, 0, (x0, y0, x1, y1)
    d = np.abs(cur[y0:y1, x0:x1].astype(np.int16)
               - base[y0:y1, x0:x1].astype(np.int16)).max(axis=2)
    raw = (d > diff_thresh).astype(np.uint8)
    if raw.sum() == 0:
        return mask, 0, (x0, y0, x1, y1)
    n_lab, lab, stats, _ = cv2.connectedComponentsWithStats(raw, connectivity=8)
    if n_lab <= 1:
        return mask, 0, (x0, y0, x1, y1)
    areas = stats[1:, cv2.CC_STAT_AREA]
    keep = int(areas.argmax()) + 1
    if int(areas[keep - 1]) < min_px:
        # too small to be the car: report it, do not pretend it is a measurement
        return mask, int(areas[keep - 1]), (x0, y0, x1, y1)
    mask[y0:y1, x0:x1] = (lab == keep)
    return mask, int(mask.sum()), (x0, y0, x1, y1)


def strip_of(rgb, mask):
    """The masked pixels as a 2-row image, so colour_match can score them.

    colour_match takes an image and a box because that is what the flight gives
    it. Packing the mesh pixels into a 2 x N strip and asking for the whole strip
    reuses the imported function verbatim on exactly the pixels of interest —
    no second copy of the hue table, and no road pixels in the average.
    """
    px = rgb[mask]
    if px.shape[0] < 2:
        return None
    return np.repeat(px[None, :, :], 2, axis=0)


def hsv_stats(rgb_strip) -> dict:
    """Mean HSV of a pixel strip, with hue averaged on the circle.

    OpenCV hue is 0-179 for 0-358 degrees and red straddles the seam, so the
    arithmetic mean of a red car's hue lands near 90 — cyan. The circular mean
    does not, which is the whole point of measuring rather than assuming.
    """
    import cv2
    hsv = cv2.cvtColor(rgb_strip, cv2.COLOR_RGB2HSV)
    h = hsv[..., 0].astype(np.float64).ravel() * 2.0        # -> degrees
    s = float(hsv[..., 1].mean())
    v = float(hsv[..., 2].mean())
    ang = np.radians(h)
    hue = math.degrees(math.atan2(np.sin(ang).mean(), np.cos(ang).mean())) % 360.0
    return {"hue_deg": round(hue, 1), "hue_cv": round(hue / 2.0, 1),
            "sat": round(s, 1), "val": round(v, 1),
            "sat_frac": round(s / 255.0, 3), "val_frac": round(v / 255.0, 3)}


def colour_profile(img_rgb, mask, box) -> dict:
    """Fraction of pixels reading as each colour word, two ways.

    `mesh` is over the car's own pixels and answers "what colour did this
    material render". `box` is over the projected bounding box, road and sky
    included, and is what the flight's gate actually computes — so it is the
    number to compare against --colour-min. They differ by a factor of two or
    more on a small distant car, which is why both are here.
    """
    words = colour_words()
    strip = strip_of(img_rgb, mask)
    mesh = {}
    if strip is not None:
        n = strip.shape[1]
        mesh = {w: round(colour_match(strip, (0, 0, n, 2), w), 4) for w in words}
    bx = {w: round(colour_match(img_rgb, box, w), 4) for w in words}
    reads_as, best = None, 0.0
    for w, frac in (mesh or bx).items():
        if frac > best:
            reads_as, best = w, frac
    return {"mesh": mesh, "box": bx, "reads_as": reads_as,
            "reads_as_frac": round(best, 4),
            "hsv": (hsv_stats(strip) if strip is not None else None)}


class Detector:
    """OWL-ViT, run synchronously — nothing is flying, so nothing needs a thread.

    One forward pass scores every query at once (the processor takes a list of
    phrases per image and post_process returns a label index per box), so adding
    the colour phrase and the extra --queries costs no extra inference.
    """

    def __init__(self, model_id: str = DETECTOR_ID):
        import torch
        from transformers import OwlViTForObjectDetection, OwlViTProcessor
        self.torch = torch
        self.dev = "cuda" if torch.cuda.is_available() else "cpu"
        t0 = time.time()
        self.proc = OwlViTProcessor.from_pretrained(model_id)
        self.model = OwlViTForObjectDetection.from_pretrained(model_id).to(self.dev).eval()
        print(f"[owlvit] {model_id} on {self.dev} in {time.time() - t0:.0f}s")

    def run(self, img, queries, gt_box=None, iou_min: float = 0.10) -> dict:
        torch = self.torch
        w, h = img.size
        queries = list(queries)
        inputs = self.proc(text=[queries], images=img, return_tensors="pt").to(self.dev)
        t = time.time()
        with torch.no_grad():
            out = self.model(**inputs)
        if self.dev == "cuda":
            torch.cuda.synchronize()
        ms = (time.time() - t) * 1000
        res = self.proc.post_process_object_detection(
            out, threshold=0.0,
            target_sizes=torch.tensor([[h, w]]).to(self.dev))[0]
        scores = res["scores"].detach().cpu().numpy()
        labels = res["labels"].detach().cpu().numpy()
        boxes = res["boxes"].detach().cpu().numpy()
        per = {}
        for qi, q in enumerate(queries):
            sel = labels == qi
            s, b = scores[sel], boxes[sel]
            rec = {"n_boxes": int(sel.sum()), "infer_ms": round(ms, 1),
                   "best_score": None, "best_box": None, "iou_best": None,
                   "score_at_car": None, "iou_at_car": None, "box_at_car": None}
            if len(s):
                k = int(s.argmax())
                rec["best_score"] = round(float(s[k]), 4)
                rec["best_box"] = [round(float(v), 1) for v in b[k]]
                if gt_box is not None:
                    ious = np.array([iou(tuple(bb), gt_box) for bb in b])
                    rec["iou_best"] = round(float(ious[k]), 3)
                    cand = np.where(ious >= iou_min)[0]
                    if len(cand):
                        j = int(cand[int(s[cand].argmax())])
                        rec["score_at_car"] = round(float(s[j]), 4)
                        rec["iou_at_car"] = round(float(ious[j]), 3)
                        rec["box_at_car"] = [round(float(v), 1) for v in b[j]]
            per[q] = rec
        return per


# ------------------------------------------------------------------- flight --

def pose_of(x: float, y: float, heading: float, z_ned: float = 0.0):
    from projectairsim.types import Pose, Quaternion, Vector3
    from projectairsim.utils import rpy_to_quaternion
    w, qx, qy, qz = rpy_to_quaternion(0.0, 0.0, heading)
    return Pose({
        "translation": Vector3({"x": x, "y": y, "z": z_ned}),
        "rotation": Quaternion({"w": w, "x": qx, "y": qy, "z": qz}),
        "frame_id": "DEFAULT_ID",
    })


async def hold(drone, seconds: float, alt: float, yaw: float) -> None:
    """Station-keep at one altitude and heading.

    Every capture happens while this is running: the frames must come from a
    stationary camera or the before/after difference measures the drone's drift
    instead of the car.
    """
    t_end = time.time() + seconds
    while time.time() < t_end:
        kin = drone.get_ground_truth_kinematics()
        up = -kin["pose"]["position"]["z"]
        vz_up = float(np.clip((alt - up) * 0.8, -1.5, 1.5))
        await drone.move_by_velocity_async(0.0, 0.0, -vz_up, duration=0.3,
                                           yaw_is_rate=False, yaw=yaw)
        await asyncio.sleep(0.1)


async def settle_to(drone, alt: float, yaw: float, timeout_s: float = 40.0) -> None:
    """Reach an altitude and a heading, then confirm both before returning."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        kin = drone.get_ground_truth_kinematics()
        up = -kin["pose"]["position"]["z"]
        psi = quat_yaw(kin["pose"]["orientation"])
        err = (yaw - psi + math.pi) % (2 * math.pi) - math.pi
        if abs(up - alt) < 0.4 and abs(err) < math.radians(3.0):
            break
        vz_up = float(np.clip((alt - up) * 0.8, -2.0, 2.0))
        await drone.move_by_velocity_async(0.0, 0.0, -vz_up, duration=0.3,
                                           yaw_is_rate=False, yaw=yaw)
        await asyncio.sleep(0.1)
    kin = drone.get_ground_truth_kinematics()
    print(f"[pose] altitude {-kin['pose']['position']['z']:.1f} m, heading "
          f"{math.degrees(quat_yaw(kin['pose']['orientation'])):+.1f} deg "
          f"(wanted {alt:.1f} m, {math.degrees(yaw):+.1f} deg)")


async def capture(drone, obs, alt: float, yaw: float, fresh: int,
                  settle_s: float, timeout_s: float = 10.0):
    """Hold station until `fresh` new camera messages have arrived, then decode.

    Counting arrivals matters: the FrontCamera publishes at 10 Hz, so reading
    immediately after set_object_material returns hands back a frame rendered
    before the paint was applied, and the material under test would be scored on
    the previous one's pixels.
    """
    n0 = obs.stats()["frames_received"]
    await hold(drone, settle_s, alt, yaw)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if obs.stats()["frames_received"] - n0 >= fresh:
            img = obs.get_front_native()
            if img is not None:
                return img
        await hold(drone, 0.2, alt, yaw)
    img = obs.get_front_native()
    if img is None:
        print("[warn] no FrontCamera frame arrived at all")
    else:
        print(f"[warn] only {obs.stats()['frames_received'] - n0} new frames in "
              f"{timeout_s:.0f}s -- using the newest available")
    return img


def annotate(img, gt_box, det_box, label: str):
    """Draw the projected box and the detector's box on a copy of the frame.

    The human has to be able to check by eye that the analytic box really lands
    on the car; every number in this survey depends on it.
    """
    from PIL import ImageDraw
    im = img.copy()
    d = ImageDraw.Draw(im)
    if gt_box is not None:
        d.rectangle([gt_box[0], gt_box[1], gt_box[2], gt_box[3]],
                    outline=(255, 255, 0), width=1)
    if det_box is not None:
        d.rectangle([det_box[0], det_box[1], det_box[2], det_box[3]],
                    outline=(0, 255, 0), width=1)
    d.text((4, 4), label, fill=(255, 255, 255))
    d.text((4, 15), "yellow = projected car box, green = OWL-ViT box",
           fill=(200, 200, 200))
    return im


# ------------------------------------------------------------------- assets --

def dump_assets(world, out: Path, asset: str) -> dict:
    """List every asset and write all of it down.

    A previously truncated listing led to the conclusion that no car mesh
    existed in this project, which was wrong — SM_Offroad_Body has been spawning
    and driving for weeks. So: the full list goes to a file, counts are printed,
    and nothing is dropped without saying how much was dropped and where the
    rest is. Absence from this list is also not evidence: the spawn later in
    this run is the real test of whether the mesh is available.
    """
    raw, err = _rpc("list_assets", world.list_assets, ".*")
    names = [asset_name(a) for a in (raw or [])]
    (out / "assets_all.txt").write_text("\n".join(names) + "\n", encoding="utf-8")
    veh = sorted(n for n in names if VEHICLE_RE.search(n))
    mat = sorted(n for n in names if MATERIAL_RE.search(n))
    (out / "assets_vehicle.txt").write_text("\n".join(veh) + "\n", encoding="utf-8")
    (out / "assets_material.txt").write_text("\n".join(mat) + "\n", encoding="utf-8")
    print(f"[assets] list_assets('.*') returned {len(names)} names "
          f"-> {out / 'assets_all.txt'}")
    if err:
        print(f"[assets] the call itself failed ({err}); the list above is empty "
              f"and proves nothing about what exists")
    for label, subset, path in (("vehicle-like", veh, out / "assets_vehicle.txt"),
                                ("material-like", mat, out / "assets_material.txt")):
        print(f"[assets] {len(subset)} {label} -> {path}")
        for n in subset[:60]:
            print(f"           {n}")
        if len(subset) > 60:
            print(f"           ... {len(subset) - 60} more, all of them in {path}")
    hit = [n for n in names if asset in n]
    print(f"[assets] {asset!r}: {'PRESENT as ' + str(hit) if hit else 'NOT in the list'}"
          + ("" if hit else " -- which is not proof it cannot be spawned; the "
                           "spawn below is the test"))
    return {"n_assets": len(names), "vehicle_like": veh, "material_like": mat,
            "asset_present": bool(hit), "list_error": err}


def resolve_materials(requested, discovered, max_auto: int) -> list:
    """Decide which material paths to spend sim time on, and say why.

    "auto" surveys whatever the sim reports as material-like, which is the only
    honest way to find out what this project actually ships. Otherwise the
    curated list is filtered against discovery — but only if discovery found at
    least one of them, because list_assets may not report materials at all, and
    filtering against an empty answer would silently skip the entire survey.
    The control is never dropped.
    """
    disc = list(discovered or [])
    if requested and len(requested) == 1 and requested[0].lower() == "auto":
        picked = [d if d.startswith("/") else d for d in disc][:max_auto]
        if len(disc) > max_auto:
            print(f"[materials] auto mode: {len(disc)} material-like assets found, "
                  f"surveying the first {max_auto} (raise --max-auto for the rest)")
        if CONTROL_MATERIAL not in picked:
            picked.insert(0, CONTROL_MATERIAL)
        print("[materials] auto-mode names are passed to the sim exactly as "
              "list_assets reported them; ones that are not valid material asset "
              "paths will simply fail to apply, and that is a result too")
        return picked
    want = list(requested) if requested else list(DEFAULT_MATERIALS)
    if disc:
        found = [m for m in want if any(m.rsplit("/", 1)[-1] in d for d in disc)]
        if found:
            dropped = [m for m in want if m not in found]
            if dropped:
                print(f"[materials] not in the asset list, skipping: {dropped}")
            want = found
    if CONTROL_MATERIAL not in want:
        want.insert(0, CONTROL_MATERIAL)
    return want


# -------------------------------------------------------------------- survey --

async def survey(args) -> int:
    from projectairsim import Drone, ProjectAirSimClient, World

    out = Path(args.out)
    frames = out / "frames"
    frames.mkdir(parents=True, exist_ok=True)

    poses = []
    for tok in args.poses.split(","):
        alt, rng = tok.split(":")
        poses.append((float(alt), float(rng)))
    spec = moving_car.CarSpec(asset=args.asset)

    det = None if args.no_owlvit else Detector()
    obs = SemanticObs()
    records, spawned, rpc_errors = [], [], []
    client = ProjectAirSimClient()
    client.connect()
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        drone = Drone(client, world, "Drone1")
        client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                         lambda _, m: obs.put_front(m))

        assets = dump_assets(world, out, args.asset)
        materials = resolve_materials(args.materials, assets["material_like"],
                                      args.max_auto)
        # None = leave the mesh in its own materials. On record as rendering dark
        # against dark asphalt, which is why the car is painted at all, so it
        # belongs in the table as the floor every candidate has to beat.
        jobs = [None] + materials
        print(f"[survey] {len(jobs)} materials x {len(poses)} poses = "
              f"{len(jobs) * len(poses)} spawns")

        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())

        for alt, rng in poses:
            # Face East. The drone spawns at (35, -20) and the street along
            # East from there is the one moving_car's routes already verify as
            # >= 10 m clear of buildings, so the car has somewhere legal to
            # stand and the background is the same asphalt the demo flies over.
            yaw = math.pi / 2
            await settle_to(drone, alt, yaw)
            for idx, mat in enumerate(jobs):
                label = mat or "<mesh default>"
                tag = f"{idx:02d}_{slug((mat or 'mesh_default').rsplit('/', 1)[-1])}"
                stem = f"{tag}_a{alt:.0f}r{rng:.0f}"
                print(f"\n[{stem}] {label}")

                kin = drone.get_ground_truth_kinematics()
                p = kin["pose"]["position"]
                psi = quat_yaw(kin["pose"]["orientation"])
                cam = (p["x"] + CAM_FWD_M * math.cos(psi),
                       p["y"] + CAM_FWD_M * math.sin(psi), -p["z"])
                car_xy = (p["x"] + rng * math.cos(psi), p["y"] + rng * math.sin(psi))
                # broadside: the long axis across the line of sight is the most
                # painted surface the camera can be shown, which is the best
                # case for both the colour reading and the detector
                car_h = psi + math.pi / 2

                base_img = await capture(drone, obs, alt, yaw, args.fresh_frames,
                                         args.settle_s)
                if base_img is None:
                    rpc_errors.append(f"{stem}: no baseline frame")
                    continue
                base_img.save(frames / f"{stem}_empty.png")

                name, err = _rpc("spawn_object", world.spawn_object,
                                 f"MatProbe_{idx}", args.asset,
                                 pose_of(car_xy[0], car_xy[1], car_h,
                                         spec.ground_z_ned),
                                 [1.0, 1.0, 1.0] if spec.unit_scale else
                                 [spec.length_m, spec.width_m, spec.height_m],
                                 False)
                if not name:
                    rpc_errors.append(f"{stem}: spawn failed ({err})")
                    records.append({"material": mat, "label": label, "alt_m": alt,
                                    "range_m": rng, "spawned": False,
                                    "spawn_error": err})
                    continue
                spawned.append(name)
                applied, mat_err = (True, None)
                if mat is not None:
                    # --as-texture routes the same candidate list through
                    # set_object_texture_from_packaged_asset instead. Worth its
                    # own mode because set_object_material accepts exactly ONE
                    # material in this build (M_Orange) while the texture call
                    # accepts everything it was offered - which proves nothing on
                    # its own, since "returns True" and "renders differently" are
                    # different claims. This measures the pixels either way.
                    if getattr(args, "as_texture", False):
                        ok, mat_err = _rpc("set_object_texture",
                                           world.set_object_texture_from_packaged_asset,
                                           name, mat)
                    else:
                        ok, mat_err = _rpc("set_object_material",
                                           world.set_object_material, name, mat)
                    applied = bool(ok) and mat_err is None
                    print(f"  material {'applied' if applied else 'REFUSED'}: {mat}")
                back, _ = _rpc("get_object_pose", world.get_object_pose, name)
                if back is not None:
                    t = back.translation
                    bx = getattr(t, "x", None)
                    bx = bx if bx is not None else t["x"]
                    by = getattr(t, "y", None)
                    by = by if by is not None else t["y"]
                    print(f"  spawned {name!r} at ({bx:.1f}, {by:.1f}); "
                          f"wanted ({car_xy[0]:.1f}, {car_xy[1]:.1f})")

                # re-read the pose: the frame is what it is, drift included
                kin = drone.get_ground_truth_kinematics()
                p = kin["pose"]["position"]
                psi = quat_yaw(kin["pose"]["orientation"])
                cam = (p["x"] + CAM_FWD_M * math.cos(psi),
                       p["y"] + CAM_FWD_M * math.sin(psi), -p["z"])
                gt = car_box_px(cam, psi, car_xy, car_h, spec)
                img = await capture(drone, obs, alt, yaw, args.fresh_frames,
                                    args.settle_s)
                if img is None:
                    rpc_errors.append(f"{stem}: no frame with the car")
                    _rpc("destroy_object", world.destroy_object, name)
                    continue
                img.save(frames / f"{stem}.png")

                rgb = np.asarray(img)
                base = np.asarray(base_img)
                rec = {"material": mat, "label": label, "intended": intended_colour(mat),
                       "alt_m": alt, "range_m": rng, "spawned": True,
                       "object": name, "material_applied": applied,
                       "material_error": mat_err,
                       "car_xy": [round(car_xy[0], 2), round(car_xy[1], 2)],
                       "car_heading_deg": round(math.degrees(car_h), 1),
                       "drone_xy_up": [round(p["x"], 2), round(p["y"], 2),
                                       round(-p["z"], 2)],
                       "drone_yaw_deg": round(math.degrees(psi), 1),
                       "frame": str(frames / f"{stem}.png"),
                       "frame_empty": str(frames / f"{stem}_empty.png")}
                if gt is None:
                    rec["error"] = "the car projects behind the image plane"
                    records.append(rec)
                    _rpc("destroy_object", world.destroy_object, name)
                    continue
                gtc = clip_box(gt, FRONT_W, FRONT_H)
                rec["gt_box"] = [round(v, 1) for v in gtc]
                rec["gt_box_px"] = round((gtc[2] - gtc[0]) * (gtc[3] - gtc[1]), 1)
                rec["gt_width_px"] = round(gtc[2] - gtc[0], 1)

                mask, n_px, win = mesh_mask(rgb, base, gtc, args.diff_thresh,
                                            args.mask_margin, args.min_px)
                rec["mesh_px"] = n_px
                rec["visible"] = bool(mask.any())
                rec["colour"] = colour_profile(rgb, mask, gtc)
                if not rec["visible"]:
                    print(f"  INVISIBLE: only {n_px} changed pixels in the box "
                          f"where the car provably is ({rec['gt_width_px']:.0f} px wide)")
                else:
                    hsv = rec["colour"]["hsv"]
                    print(f"  {n_px} mesh px, hue {hsv['hue_deg']:.0f} deg "
                          f"S {hsv['sat']:.0f} V {hsv['val']:.0f} -> reads as "
                          f"{rec['colour']['reads_as']} "
                          f"({rec['colour']['reads_as_frac']:.2f} of mesh px)")

                queries = ["a car"]
                want = intended_colour(mat)
                if want:
                    queries.append(phrase(want))
                reads = rec["colour"]["reads_as"]
                if reads and phrase(reads) not in queries:
                    queries.append(phrase(reads))
                for q in args.queries or []:
                    if q not in queries:
                        queries.append(q)
                if det is not None:
                    rec["owlvit"] = det.run(img, queries, gtc, args.iou_min)
                    rec["owlvit_empty"] = det.run(base_img, queries, gtc,
                                                  args.iou_min)
                    for q in queries:
                        a, b = rec["owlvit"][q], rec["owlvit_empty"][q]
                        print(f"  OWL {q!r:22} at-car {a['score_at_car']} "
                              f"(IoU {a['iou_at_car']})  best {a['best_score']} "
                              f"| empty street best {b['best_score']}")
                    dbox = rec["owlvit"]["a car"].get("box_at_car")
                else:
                    dbox = None
                annotate(img, gtc, dbox, stem).save(frames / f"{stem}_annot.png")
                records.append(rec)
                _rpc("destroy_object", world.destroy_object, name)
                spawned.remove(name)

        for _ in range(600):
            kin = drone.get_ground_truth_kinematics()
            if -kin["pose"]["position"]["z"] <= 1.2:
                break
            await drone.move_by_velocity_async(0.0, 0.0, 1.2, duration=0.3)
            await asyncio.sleep(0.1)
        await (await drone.land_async())
        drone.disarm()
        drone.disable_api_control()
    except Exception as exc:
        print(f"[warn] survey aborted: {type(exc).__name__}: {exc}")
        rpc_errors.append(f"aborted: {type(exc).__name__}: {exc}")
        assets = locals().get("assets", {"n_assets": 0, "vehicle_like": [],
                                         "material_like": [], "asset_present": False,
                                         "list_error": "survey aborted"})
    finally:
        if args.keep:
            print(f"[cleanup] --keep: leaving {len(spawned)} object(s) in the scene")
        else:
            for name in list(spawned):
                _rpc("destroy_object", locals()["world"].destroy_object, name) \
                    if "world" in locals() else None
            if spawned and "world" in locals():
                _rpc("destroy_all_spawned_objects",
                     locals()["world"].destroy_all_spawned_objects)
        client.disconnect()

    (out / "survey.json").write_text(json.dumps(
        {"asset": args.asset, "poses": poses, "detector":
         (None if args.no_owlvit else DETECTOR_ID),
         "colour_min": args.colour_min, "det_thresh": args.det_thresh,
         "assets": assets, "rpc_errors": rpc_errors, "records": records},
        indent=1), encoding="utf-8")
    write_verdict(out, args, poses, records, assets, rpc_errors)
    return 0


# ------------------------------------------------------------------ verdict --

def verdict_of(near, far, args) -> tuple:
    """One word per material, and the numbers that decided it.

    The order of the tests is the order in which a material fails in practice:
    it has to be applied at all, then be visible, then read as the colour its
    name claims, then survive the flight's own colour gate at flight range, and
    only then does the detector score matter.
    """
    if near is None and far is None:
        return "NO DATA", "no frame captured"
    src = near or far
    if src.get("material") is not None and not src.get("material_applied"):
        return "NOT APPLIED", f"sim refused the material ({src.get('material_error')})"
    if not src.get("visible"):
        return "INVISIBLE", f"{src.get('mesh_px', 0)} changed px in the projected box"
    want = src.get("intended")
    reads = (src.get("colour") or {}).get("reads_as")
    frac_far = None
    if far and want:
        frac_far = (far.get("colour") or {}).get("box", {}).get(want)
    at_car = None
    if far and far.get("owlvit"):
        at_car = far["owlvit"].get("a car", {}).get("score_at_car")
    if want is None:
        return "NO COLOUR CLAIM", f"renders as {reads}"
    if reads != want:
        return "WRONG COLOUR", f"named {want}, renders as {reads}"
    if frac_far is None or frac_far < args.colour_min:
        return "TOO WEAK", (f"reads {want} but only {frac_far} of the box at "
                            f"{far['range_m']:.0f} m, gate wants "
                            f"{args.colour_min}")
    if at_car is None or at_car < args.det_thresh:
        return "DETECTOR LOSES IT", (f"colour fine, but OWL-ViT scores "
                                     f"{at_car} on the car at "
                                     f"{far['range_m']:.0f} m")
    return "USABLE", f"reads {want}, gate {frac_far}, detector {at_car}"


def write_verdict(out: Path, args, poses, records, assets, rpc_errors) -> None:
    """The whole point of the run: one table the material choice can be made from."""
    by_mat = {}
    for r in records:
        by_mat.setdefault(r.get("label", "?"), {})[r.get("range_m")] = r
    near_r = min(p[1] for p in poses) if poses else None
    far_r = max(p[1] for p in poses) if poses else None

    lines = [f"# Material survey — {args.asset}", "",
             f"Sim assets listed: {assets.get('n_assets', 0)}  |  "
             f"{args.asset} in the list: {assets.get('asset_present')}  |  "
             f"material-like assets: {len(assets.get('material_like', []))}", "",
             f"Poses (altitude m : range m): "
             + ", ".join(f"{a:.0f}:{r:.0f}" for a, r in poses),
             f"Colour gate threshold {args.colour_min} (demo/follow_vlm.py "
             f"--colour-min), detector threshold {args.det_thresh} "
             f"(--det-thresh).", ""]

    for alt, rng in poses:
        lines += [f"## Pose: {alt:.0f} m altitude, {rng:.0f} m range", "",
                  "| material | intended | mesh px | box px | hue deg | S | V |"
                  " reads as | colour_match(intended) over box | OWL \"a car\""
                  " at car | OWL colour phrase at car | OWL \"a car\" empty"
                  " street |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for label, per in by_mat.items():
            r = per.get(rng)
            if r is None:
                lines.append(f"| `{label}` | | | | | | | | | | | |")
                continue
            col = r.get("colour") or {}
            hsv = col.get("hsv") or {}
            want = r.get("intended")
            gate = col.get("box", {}).get(want) if want else None
            owl = (r.get("owlvit") or {}).get("a car", {})
            empty = (r.get("owlvit_empty") or {}).get("a car", {})
            cq = phrase(want) if want else None
            owlc = (r.get("owlvit") or {}).get(cq, {}) if cq else {}
            lines.append(
                f"| `{label}` | {want or '-'} | {r.get('mesh_px', 0)} | "
                f"{r.get('gt_box_px', '-')} | {hsv.get('hue_deg', '-')} | "
                f"{hsv.get('sat', '-')} | {hsv.get('val', '-')} | "
                f"{col.get('reads_as', '-')} | {gate if gate is not None else '-'} | "
                f"{owl.get('score_at_car', '-')} | "
                f"{owlc.get('score_at_car', '-')} | "
                f"{empty.get('best_score', '-')} |")
        lines.append("")

    rows = []
    for label, per in by_mat.items():
        near, far = per.get(near_r), per.get(far_r)
        verdict, why = verdict_of(near, far, args)
        src = near or far or {}
        col = (src.get("colour") or {})
        gate_far = None
        if far and far.get("intended"):
            gate_far = (far.get("colour") or {}).get("box", {}).get(far["intended"])
        at_car = ((far or {}).get("owlvit") or {}).get("a car", {}).get("score_at_car")
        rows.append({
            "label": label, "verdict": verdict, "why": why,
            "reads_as": col.get("reads_as"), "intended": src.get("intended"),
            "visible": bool(src.get("visible")), "gate_far": gate_far,
            "at_car": at_car,
            "rank": (verdict == "USABLE", bool(src.get("visible")),
                     gate_far or 0.0, at_car or 0.0),
        })
    rows.sort(key=lambda r: r["rank"], reverse=True)

    lines += ["## Ranked verdict", "",
              "| rank | material | verdict | intended | renders as | "
              "colour gate at far range | OWL \"a car\" on the car | why |",
              "|---|---|---|---|---|---|---|---|"]
    for i, r in enumerate(rows, 1):
        lines.append(f"| {i} | `{r['label']}` | **{r['verdict']}** | "
                     f"{r['intended'] or '-'} | {r['reads_as'] or '-'} | "
                     f"{r['gate_far'] if r['gate_far'] is not None else '-'} | "
                     f"{r['at_car'] if r['at_car'] is not None else '-'} | "
                     f"{r['why']} |")
    lines += ["", "USABLE means: the sim accepted the material, the mesh is "
              "visible, it reads as the colour its name claims, it clears the "
              "flight's colour gate at flight range, and OWL-ViT still finds the "
              "car. Anything else names the step that failed.", "",
              "Read the `_annot.png` frames before trusting a row: the yellow "
              "box is where the car was projected to be and every colour number "
              "is measured inside it."]
    if rpc_errors:
        lines += ["", "## Sim calls that failed", ""] + [f"- {e}" for e in rpc_errors]
    (out / "VERDICT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("\n" + "=" * 72)
    for i, r in enumerate(rows, 1):
        print(f"{i:2}. {r['verdict']:18} {r['label']:48} {r['why']}")
    print("=" * 72)
    print(f"[out] {out / 'VERDICT.md'}")
    print(f"[out] {out / 'survey.json'}")
    print(f"[out] frames + annotated frames in {out / 'frames'}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Measure what each candidate material actually renders as, "
                    "and whether OWL-ViT still finds the car once it is painted.")
    ap.add_argument("--as-texture", action="store_true",
                    help="apply each candidate as a TEXTURE rather than a "
                         "material")
    ap.add_argument("--materials", nargs="*", default=None,
                    help="material asset paths to survey. 'auto' surveys the "
                         "material-like names list_assets reports. Default is a "
                         "curated set including the M_Orange control, which is "
                         "always included either way.")
    ap.add_argument("--asset", default=moving_car.CarSpec().asset,
                    help="car mesh to spawn and paint")
    ap.add_argument("--out", default=str(ROOT / "experiments" / "out" / "materials"))
    ap.add_argument("--queries", nargs="*", default=None,
                    help="extra OWL-ViT phrases to score on every frame. 'a car' "
                         "and the phrase for each material's own colour are "
                         "always included.")
    ap.add_argument("--keep", action="store_true",
                    help="leave spawned objects in the scene instead of "
                         "destroying them, to inspect the last one by hand")
    ap.add_argument("--poses", default="4:8,9:22",
                    help="altitude:range pairs, metres. 4:8 fills the frame for "
                         "the colour measurement; 9:22 is the measured flight "
                         "condition where the detector score means something.")
    ap.add_argument("--max-auto", type=int, default=12,
                    help="cap on how many discovered materials --materials auto "
                         "surveys; each one costs a spawn and two captures")
    ap.add_argument("--colour-min", type=float, default=0.10,
                    help="the flight's colour gate (demo/follow_vlm.py); a "
                         "material has to clear it at flight range to be usable")
    ap.add_argument("--det-thresh", type=float, default=0.02,
                    help="the flight's detector threshold")
    ap.add_argument("--iou-min", type=float, default=0.10,
                    help="minimum IoU with the projected box for a detection to "
                         "count as having found the car rather than something else")
    ap.add_argument("--diff-thresh", type=int, default=18,
                    help="per-channel difference that counts as a changed pixel")
    ap.add_argument("--mask-margin", type=int, default=8,
                    help="how far outside the projected box to look for changed "
                         "pixels, absorbing station-keeping drift")
    ap.add_argument("--min-px", type=int, default=40,
                    help="fewer changed pixels than this counts as invisible")
    ap.add_argument("--fresh-frames", type=int, default=3,
                    help="new camera messages to wait for before reading a frame")
    ap.add_argument("--settle-s", type=float, default=0.8)
    ap.add_argument("--no-owlvit", action="store_true",
                    help="colour measurement only; skips loading the detector")
    return asyncio.run(survey(ap.parse_args()))


if __name__ == "__main__":
    sys.exit(main())
