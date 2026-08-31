"""
Follow a named object with vision and language, through the guardrail.

You type "a car". A vision-language model finds that thing in the camera image.
A servo loop keeps it centred and at a chosen distance. The Shield still checks
every action. Nothing in the steering path knows the target's coordinates.

Why this and not AerialVLA
--------------------------
AerialVLA was measured, over 108 controlled forward passes and several flights,
to ignore its object-description slot entirely: correct and wrong colour words
produce indistinguishable actions, and with no coordinate-derived bearing phrase
it emits stop-and-land on 11 of 18 real frames. Flown against a real car mesh it
never moved at all — 12 of 12 inferences returned the same token triple. See
docs/FINDING-what-drives-aerialvla.md.

So the language grounding is done by a model that actually does it. OWL-ViT is an
open-vocabulary detector: text in, boxes out. Measured here at 34-56 ms per frame
against AerialVLA's 11-12 s, which also removes the latency problem that made
closed-loop tracking impossible.

This is still vision-language-action — the words choose the target, the camera
finds it, the controller acts — with the model swapped for one whose language
input reaches the output.

What the controller does
------------------------
    horizontal box offset  -> yaw rate      (turn to centre the target)
    box width vs desired   -> forward speed (hold a standoff distance)
    altitude error         -> climb rate    (hold the cruise height)

The altitude term is deliberate: the earlier scripts had none and leaned on the
Shield, which is a constraint filter making minimal corrections, not a
controller. Flights sagged tens of seconds below the floor as a result.

Run:
    python demo/follow_vlm.py --object "a car" --tag vlmfollow
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import random
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import AuditLogger, Shield, State, load_policy       # noqa: E402
from guardrail import kpi as kpi_mod                                # noqa: E402
from guardrail.manifest import build_manifest, is_kpi_grade         # noqa: E402
from guardrail.geometry import fence_polygon                        # noqa: E402
from guardrail.models import (                                      # noqa: E402
    Action4D, AltitudeEnvelope, ObstacleClearance, PolygonFence,
)
from shapely.geometry import Point                                  # noqa: E402

import city_planner                                                 # noqa: E402
import city_traffic
from recorder import FrameRecorder
import car_trajectory
import pedestrians as people_mod
from target_state import TargetState, want_range_from_width
import moving_car                                                   # noqa: E402
from aerialvla_demo import RateLimiter                              # noqa: E402
from semantic_demo import SemanticObs, quat_yaw                     # noqa: E402

TICK = 0.1
SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_semantic.jsonc"
# Declared in that scene file. The simulator drives this actor along an uploaded
# trajectory; see demo/env_actor_car.jsonc and demo/car_trajectory.py.
ENV_CAR_NAME = "SemCarActor"
DETECTOR_ID = "google/owlvit-base-patch32"


# Hue ranges in OpenCV's 0-179 scale, plus how saturated a pixel must be to count
# as that colour at all. Grey road and pale concrete have low saturation, so the
# saturation floor does most of the rejecting.
COLOUR_HUE = {
    "red": [(0, 10), (170, 179)], "orange": [(8, 24)], "yellow": [(22, 35)],
    "green": [(36, 85)], "blue": [(90, 130)], "purple": [(130, 160)],
}
COLOUR_ACHROMATIC = {"white": "white", "black": "black", "grey": "grey", "gray": "grey"}

# How much absolute colour a pixel must carry to count as its hue, on the 0-255
# scale of max(R,G,B) - min(R,G,B). Replaces a saturation floor, which is chroma
# divided by brightness and therefore collapses in sunlight; see colour_match.
# 40 admits the sunlit taxi (0.174 of its pixels) and still rejects road and
# zebra crossing outright (0.0004).
COLOUR_MIN_CHROMA = 40.0


def colour_word(query: str):
    q = query.lower()
    for w in list(COLOUR_HUE) + list(COLOUR_ACHROMATIC):
        if w in q:
            return w
    return None


# Real-world width of the subject, used to turn an apparent box width into a
# range whenever depth is unusable - which is most of the time, because the
# depth stream quantises to whole metres.
#
# This was a single hardcoded 4.0, a car, applied to whatever was named. The
# 2026-08-19 review asked for a pedestrian in the scene next, and a pedestrian
# is about 0.5 m wide: the same code would have reported a person at roughly
# EIGHT TIMES their true distance, and the aircraft would have flown that far
# in to close the gap. The pedestrian demo is meaningless until the width
# follows the noun.
#
# Longest match wins, so "police car" does not resolve on "car" if a more
# specific entry is ever added. Values are ordinary vehicle and body widths, not
# measurements of the assets - a rough width is enough, since range goes as the
# width and a 20 percent error is a 20 percent range error, not a factor of 8.
SUBJECT_WIDTH_M = {
    "pedestrian": 0.5, "person": 0.5, "human": 0.5, "man": 0.5, "woman": 0.5,
    "cyclist": 0.6, "bicycle": 0.6, "bike": 0.6,
    "motorcycle": 0.8, "motorbike": 0.8, "scooter": 0.8,
    "car": 4.0, "taxi": 4.0, "sedan": 4.0, "suv": 4.4,
    "van": 5.0, "delivery": 5.0, "pickup": 5.4,
    "truck": 6.0, "lorry": 6.0, "bus": 12.0,
}
SUBJECT_WIDTH_DEFAULT = 4.0


def subject_width(query: str) -> tuple[float, str | None]:
    """Real width implied by the words, and the word it came from.

    Returns the default with a None word when nothing matches, so the caller can
    say so out loud rather than let a silent 4.0 look like a decision.
    """
    q = query.lower()
    best = None
    for w in SUBJECT_WIDTH_M:
        if w in q and (best is None or len(w) > len(best)):
            best = w
    if best is None:
        return SUBJECT_WIDTH_DEFAULT, None
    return SUBJECT_WIDTH_M[best], best


# The depth stream is quantised to whole metres (see SemanticObs.get_depth),
# so a margin below 1 m is below the resolution of the signal it tests. A
# 0.9 m true height difference quantises to 1 m about 90% of the time, which
# is why 0.5 m appeared to work; at shallower depression angles the height
# difference shrinks and it stops working. 1.0 m asks for a difference the
# stream can actually represent.
def object_mask_from_depth(depth, box, margin_m: float = 1.0, band: float = 2.0,
                           min_frac: float = 0.05, max_frac: float = 0.90):
    """Which pixels inside the box are the OBJECT rather than the ground?

    Returns a boolean mask over the box, or None when the depth cannot support
    the judgement - in which case the caller must fall back rather than guess.

    WHY THIS IS NEEDED. `colour_match` used to measure the fraction of the
    BOUNDING BOX that is the named colour, which silently includes whatever the
    object is standing on. Measured on two scheduled stops of the same flight, at
    the same range (11.3 m against 11.4 m) and the same altitude:

        plain asphalt     yellow 0.148-0.172   -> passes the 0.10 gate
        zebra crossing    yellow 0.027-0.042   -> REJECTED, 96/138 ticks lost

    The detector had found the car both times. The white stripes diluted the
    yellow fraction four-fold and the gate threw the car away. That is the
    "kehilangan object padahal object ada di depannya" at the intersection.

    WHY A DEPTH BAND DOES NOT WORK. The obvious mask - pixels near the object's
    median depth - keeps the stripes, because they are underneath and around the
    car at almost exactly its range.

    What separates them is HEIGHT. The car is raised off the road, and with the
    camera pitched down a point 1.5 m up is about 1.5*sin(depression) nearer -
    roughly 0.9 m at the angles flown here. So an object pixel is one that is
    measurably NEARER than the road surface at the same image row.

    The road depth per row is estimated from the depth frame itself, as the
    median over a horizontal band either side of the box. That is
    self-calibrating: it needs no altitude and no pitch, and it tolerates both
    being wrong.
    """
    if depth is None:
        return None
    depth = np.asarray(depth)
    if depth.ndim != 2:
        return None
    h, w = depth.shape
    x0, y0, x1, y1 = [int(v) for v in box]
    x0, x1 = max(0, x0), min(w, x1)
    y0, y1 = max(0, y0), min(h, y1)
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None                       # too few pixels to say anything

    crop = depth[y0:y1, x0:x1].astype(float)
    # Same sentinel filtering as range_from_depth: the renderer reports "no hit"
    # as a huge value rather than NaN, so both have to go.
    ok = np.isfinite(crop) & (crop > 0.1) & (crop < 1000.0)
    if int(ok.sum()) < 16:
        return None

    pad = int(max(4, band * (x1 - x0)))
    lx0, rx1 = max(0, x0 - pad), min(w, x1 + pad)
    mask = np.zeros(crop.shape, bool)
    for i, row in enumerate(range(y0, y1)):
        side = np.concatenate([depth[row, lx0:x0], depth[row, x1:rx1]])
        side = side[np.isfinite(side) & (side > 0.1) & (side < 1000.0)]
        if side.size < 8:
            continue                      # no road visible on this row
        ground = float(np.median(side))
        mask[i] = ok[i] & (crop[i] < ground - margin_m)

    frac = float(mask.mean())
    # Both extremes mean the estimate failed rather than that the object is
    # tiny or enormous: nothing standing proud of the road, or the "road"
    # reference itself being the object. Fall back instead of trusting it.
    if frac < min_frac or frac > max_frac:
        return None
    return mask


def colour_match(img, box, word, depth=None, stats=None,
                 legacy_sat: bool = False) -> float:
    """Fraction of pixels inside the box that really are the named colour.

    With `depth`, the fraction is taken over the OBJECT's pixels rather than the
    whole box - see `object_mask_from_depth` for why that matters and for the
    measurement that forced it. Without it, or when the depth cannot support a
    mask, the behaviour is exactly what it always was.

    The units do not change either way: this is still "what fraction of the
    thing is the named colour", so `--colour-min` keeps its meaning and every
    previously measured discrimination result stays comparable.

    The detector grounds the noun; this grounds the adjective. Without it the
    colour word does nothing measurable — "a blue car" and "an orange car"
    scored identically against the same orange car — and a confident detection
    on the wrong object is indistinguishable from the right one. A false lock
    cost a whole flight: the aircraft centred a city object at the right apparent
    size and held station on it 60 m from the actual car.
    """
    import cv2
    if word is None:
        return 1.0
    x0, y0, x1, y1 = [int(max(0, v)) for v in box]
    if x1 - x0 < 2 or y1 - y0 < 2:
        return 0.0
    crop = np.asarray(img)[y0:y1, x0:x1]
    if crop.size == 0:
        return 0.0
    hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    if word in COLOUR_ACHROMATIC:
        kind = COLOUR_ACHROMATIC[word]
        if kind == "white":
            m = (s < 60) & (v > 170)
        elif kind == "black":
            m = v < 60
        else:
            m = (s < 60) & (v >= 60) & (v <= 170)
    else:
        m = np.zeros(h.shape, bool)
        for lo, hi in COLOUR_HUE[word]:
            m |= (h >= lo) & (h <= hi)
        # ABSOLUTE CHROMA, NOT SATURATION. This is the intersection dropout.
        #
        # HSV saturation is chroma divided by brightness, so the brighter a
        # surface is lit the more absolute colour it needs to reach the same S.
        # The old floor `s > 90` was tuned on a car in shade and threw the SAME
        # CAR away in sunlight. Measured on the two scheduled stops of one
        # flight, on the car's own pixels:
        #
        #     stop 1, shade    hue-matched 0.321, of which 56.7% pass s>90 -> 0.182 PASS
        #     stop 2, sunlit   hue-matched 0.213, of which  8.1% pass s>90 -> 0.017 FAIL
        #
        # Median saturation of the yellow pixels fell from 99 to 69 purely
        # because the sun came out. The detector was scoring that car 0.17-0.30,
        # its strongest of the flight, and 25 consecutive boxes were rejected by
        # this one comparison while the taxi filled the frame.
        #
        # Chroma = max-min of RGB does not deflate under bright light, and for
        # uint8 HSV it is exactly s*v/255, so it costs no extra conversion.
        # Measured with a floor of 40: sunlit car 0.174, shaded car 0.285, and
        # the zebra crossing and road it stands on 0.0004 and 0.0000. The gate
        # still rejects the background completely; it just stops rejecting the
        # target for being well lit.
        if legacy_sat:
            m &= (s > 90) & (v > 50)      # the old floor, for the A/B
        else:
            m &= (s.astype(np.float32) * v / 255.0 > COLOUR_MIN_CHROMA) & (v > 50)

    mask = object_mask_from_depth(depth, (x0, y0, x1, y1)) if depth is not None else None
    if mask is not None and mask.shape == m.shape:
        if stats is not None:
            stats["masked"] = stats.get("masked", 0) + 1
        return float(m[mask].mean()) if mask.any() else 0.0
    if stats is not None:
        stats["whole_box"] = stats.get("whole_box", 0) + 1
    return float(m.mean())


def range_from_depth(depth, det, shrink: float = 0.35):
    """Metres to the detected object, from the depth image. None if unusable.

    The detection box comes from the Scene capture and indexes straight into the
    depth capture, which is why both are configured at the same resolution and
    the same field of view.

    Two deliberate choices:

    * **Shrink the box before sampling.** A bounding box always contains
      background — sky above a car, road beside it — and background is usually
      much further away than the object. Sampling the middle 35% keeps the
      window on the object itself.
    * **Median, not mean.** Even a shrunken window catches the odd background
      pixel, and one sky pixel at 5 km would drag a mean into uselessness. The
      median ignores it.

    Why this exists at all: the radial servo used apparent box width, which is a
    fine range proxy for a car (about the same width from any angle) and a bad
    one for a building. On a 50 x 50 m block apparent width swings by root-2
    between face-on and corner-on, so circling made the width servo command
    reverse from the aspect change alone and the orbit radius spiralled from
    37.7 m to 178 m.
    """
    if depth is None or det is None:
        return None
    cx, cy, bw, bh = float(det[0]), float(det[1]), float(det[2]), float(det[3])
    h, w = depth.shape[:2]
    half_w, half_h = max(1.0, bw * shrink / 2), max(1.0, bh * shrink / 2)
    x0, x1 = int(max(0, cx - half_w)), int(min(w, cx + half_w + 1))
    y0, y1 = int(max(0, cy - half_h)), int(min(h, cy + half_h + 1))
    if x1 <= x0 or y1 <= y0:
        return None
    win = depth[y0:y1, x0:x1]
    # The renderer reports "no hit" as a huge value rather than as NaN, so both
    # have to be filtered or the median is meaningless.
    good = win[np.isfinite(win) & (win > 0.1) & (win < 1000.0)]
    if good.size < 4:
        return None
    return float(np.median(good))


def implied_width_m(det, rng_m: float, hfov_deg: float = 90.0):
    """How wide the detected thing must physically be, given its range.

    A box is an angle. With a range, that angle becomes a size — and a size can
    be checked against what the named object actually is. This is the first
    signal in the system that can say a detection is IMPLAUSIBLE rather than
    merely low-scoring.
    """
    if det is None or rng_m is None or rng_m <= 0:
        return None
    bw, W = float(det[2]), float(det[5])
    half = math.radians(hfov_deg / 2.0) * (bw / W)
    return 2.0 * rng_m * math.tan(half)


def implied_range_from_width(det, object_width_m: float = 4.0,
                            hfov_deg: float = 90.0):
    """Range implied by an apparent box width - the inverse of implied_width_m.

    The fallback for when depth is unusable. Noisier than depth (it inherits the
    box's width jitter and assumes the object's real width), which is precisely
    why it feeds an ESTIMATOR now rather than the controller directly.
    """
    if det is None:
        return None
    bw, W = float(det[2]), float(det[5])
    if bw <= 0 or W <= 0:
        return None
    half = math.radians(hfov_deg / 2.0) * (bw / W)
    if half <= 1e-4:
        return None
    return (object_width_m / 2.0) / math.tan(half)


# Physical width in metres that a phrase is allowed to imply. Deliberately wide:
# the job is to reject a 40 m "car", not to measure one.
PLAUSIBLE_WIDTH_M = {
    "car": (1.0, 8.0), "truck": (2.0, 14.0), "bus": (2.0, 16.0),
    "van": (1.5, 10.0), "person": (0.2, 1.5), "traffic light": (0.1, 2.0),
    "lamp post": (0.1, 2.0), "tree": (0.5, 20.0), "building": (5.0, 200.0),
}


def plausible_noun(query: str):
    q = query.lower()
    for noun in PLAUSIBLE_WIDTH_M:
        if noun in q:
            return noun
    return None


def search_sweep_rate(elapsed_s: float, sweep_deg: float, period_s: float,
                      cap: float = 1.1) -> float:
    """Yaw rate for the search sweep, rad/s.

    A cosine, so the ANGLE is a sine about the bearing the target was last seen
    on and the net rotation over a whole period is zero. That property is the
    whole point and is asserted in the tests: the previous implementation used a
    constant sign, so the nose rotated away from a bearing that was still
    correct and never came back -- 292 degrees on one measured episode, which is
    the "360 manoeuvre" seen on the demo video.

    `sweep_deg` is a half-amplitude and should stay inside the camera's 45 deg
    horizontal half-FOV, so the last known bearing never leaves the frame.
    """
    w = 2.0 * math.pi / max(0.5, period_s)
    return float(np.clip(math.radians(sweep_deg) * w * math.cos(w * elapsed_s),
                         -cap, cap))


def presence_verdict(det, rng_m, query: str, colour_min: float,
                     frame_frac_max: float = 0.85):
    """PRESENT / ABSENT / UNSURE for the thing the operator named.

    Acknowledged limitation number two has always been that this system cannot
    say the object is not there: with no car in the scene the raw detector still
    fires on roughly three quarters of frames, and only the colour gate suppresses
    the follow. The orbit control arm made the cost plain — subject retention
    scored 1.000 on BOTH arms, including the one that flew 200 m from any traffic
    light — so detection presence separated nothing at all.

    Three checks, each rejecting a different way of being wrong, and none of them
    a confidence threshold:

    * **Colour.** Already there, and still the strongest: a box whose pixels are
      not the named colour is not the named object.
    * **Scale.** A box covering most of the frame is a wall, not an object. The
      orbit flights returned a median box of 395 px in a 400 px frame for
      "a building" and every downstream stage treated it as a target.
    * **Implied size.** With depth, a box angle becomes a physical width. A "car"
      that must be 40 m across is not a car. This one is new and is the only
      check that uses the range signal for anything other than control.

    Returns (verdict, reason). UNSURE when depth is unavailable and the cheaper
    checks pass — honest about not knowing rather than defaulting to PRESENT.
    """
    if det is None:
        return "ABSENT", "no detection"
    bw, W = float(det[2]), float(det[5])
    if bw / W > frame_frac_max:
        return "ABSENT", f"box is {bw / W:.0%} of frame — a wall, not an object"
    if colour_word(query) is not None and float(det[7]) < colour_min:
        return "ABSENT", f"colour {float(det[7]):.2f} < {colour_min:.2f}"
    noun = plausible_noun(query)
    w_m = implied_width_m(det, rng_m)
    if noun is not None and w_m is not None:
        lo, hi = PLAUSIBLE_WIDTH_M[noun]
        if not (lo <= w_m <= hi):
            return "ABSENT", (f"implies {w_m:.1f} m wide at {rng_m:.0f} m; "
                              f"a {noun} is {lo}-{hi} m")
    if w_m is None:
        return "UNSURE", "no range — size not checked"
    return "PRESENT", f"{w_m:.1f} m wide at {rng_m:.0f} m"


def appearance(img, box, h_bins: int = 8, s_bins: int = 4):
    """A small HSV histogram of the box interior — what the thing LOOKS like.

    The gap this closes is the ceiling the absence work hit. Four geometric checks
    reach 0.75 ABSENT with no target against 0.40 with one, and cannot do better,
    because in a dense city there really are white, car-sized, car-distance
    objects — geometrically they ARE a car. Position identity (`TargetLock`) and
    size plausibility both say "consistent"; nothing says "that is a DIFFERENT
    white thing".

    Deliberately coarse: 8 hue by 4 saturation is 32 numbers. The job is to tell
    one object from another across a few seconds of the same flight, not to
    re-identify it tomorrow under different light. A big descriptor would mostly
    encode illumination.

    The middle 60% of the box is sampled for the same reason `range_from_depth`
    shrinks its window — a bounding box always contains background, and here the
    background is what makes two different objects look alike.
    """
    import cv2
    if img is None or box is None:
        return None
    x0, y0, x1, y1 = [int(max(0, v)) for v in box]
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hw, hh = (x1 - x0) * 0.3, (y1 - y0) * 0.3
    crop = np.asarray(img)[int(cy - hh):int(cy + hh) + 1,
                           int(cx - hw):int(cx + hw) + 1]
    if crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)
    h = (hsv[..., 0].astype(int) * h_bins // 180).clip(0, h_bins - 1)
    s = (hsv[..., 1].astype(int) * s_bins // 256).clip(0, s_bins - 1)
    hist = np.bincount((h * s_bins + s).ravel(),
                       minlength=h_bins * s_bins).astype(float)
    total = hist.sum()
    return hist / total if total > 0 else None


def appearance_similarity(a, b) -> float:
    """Histogram intersection, 0..1. 1 is identical."""
    if a is None or b is None:
        return 1.0            # no opinion rather than a false accusation
    return float(np.minimum(a, b).sum())


class PresenceMonitor:
    """presence_verdict plus the one check that needs memory: does the thing keep
    the same physical size?

    Implied width is a physical property, so for a real object it is CONSTANT
    while range and apparent width both change. For a detector wandering between
    unrelated bits of city it is not. That makes instability a signal no single
    frame can provide, and measurement says it is the best one available:

        arm            implied width   range     rolling CV (1 s)   CV > 0.35
        car present        3.4 m       29.8 m         0.406            0.56
        NO car at all      5.4 m       69.0 m         0.615            0.84

    Nothing here is a confidence score — every check is geometric or photometric,
    which is the point: the detector's own confidence is exactly what cannot tell
    absence from presence.

    READ THE OUTPUT AS A FLIGHT-LEVEL INDICATOR, NOT A PER-TICK VERDICT.

    A threshold sweep over both logs found the ceiling of this whole approach:

        cv_max   width band   ABSENT with a car   ABSENT with no car   gap
         0.35      1.0-8.0          0.40                0.75          0.35
         0.55      1.0-8.0          0.24                0.45          0.21
         none      1.0-8.0          0.19                0.31          0.13

    The defaults are that optimum. But 0.40 means the check calls ABSENT on two
    ticks in five of a flight that tracked its target 100% of the time within
    30 m — so a controller must NOT gate on it tick by tick. Over a whole flight
    75% against 40% does separate the conditions, and that is the honest claim:
    the first signal in this system that responds to absence at all, at aggregate
    resolution only.
    """

    def __init__(self, query: str, colour_min: float, window: int = 10,
                 cv_max: float = 0.35, appear_min: float = 0.0):
        self.query = query
        self.colour_min = colour_min
        self.window = window
        self.cv_max = cv_max
        # How much the thing may change appearance and still be the same thing.
        #
        # DEFAULT 0 — DISABLED, because it was measured and it does not help.
        # Flown as a matched present/absent pair and swept offline:
        #
        #     appear_min   ABSENT with car   ABSENT no car   gap
        #        0.00 (off)     0.37              0.67       0.30
        #        0.40           0.49              0.85       0.36
        #        0.55           0.68              0.96       0.28
        #
        # The best it ever adds is 0.01 over geometry alone (0.36 against 0.35),
        # and it buys that by raising BOTH arms together rather than separating
        # them. The reason is resolution, not concept: at this range the box is
        # 22 px wide, the middle-60% sample is about 13 x 9 px, and a 32-bin
        # histogram from ~126 pixels is 4 pixels per bin. That is noise with a
        # shape, not a fingerprint — present and absent similarity distributions
        # overlap heavily (median 0.679 against 0.515, p10 0.341 against 0.259).
        #
        # Kept, tested and opt-in: the machinery is correct and would work on a
        # target that fills more of the frame, which is the closer-range or
        # higher-resolution case. It is simply not usable here.
        self.appear_min = appear_min
        self._w: list[float] = []
        self._ref = None            # appearance of the instance being followed
        self._ref_hits = 0
        self.last_sim = None
        self.counts = {"PRESENT": 0, "ABSENT": 0, "UNSURE": 0}

    def update(self, det, rng_m, app=None):
        verdict, why = presence_verdict(det, rng_m, self.query, self.colour_min)
        # Appearance identity. Geometry can only say "consistent with a car";
        # this is the only check that can say "a DIFFERENT car-like thing".
        if verdict == "PRESENT" and app is not None and self.appear_min > 0:
            if self._ref is None:
                self._ref = app
            else:
                sim = appearance_similarity(self._ref, app)
                self.last_sim = sim
                if sim < self.appear_min:
                    verdict = "ABSENT"
                    why = f"looks different: {sim:.2f} similarity to the target"
                else:
                    # Drift slowly toward the current look, so gradual lighting
                    # change is tolerated but a jump to another object is not.
                    self._ref = 0.9 * self._ref + 0.1 * app
                    self._ref_hits += 1
        w_m = implied_width_m(det, rng_m)
        if w_m is not None:
            self._w.append(w_m)
            del self._w[:-self.window]
        if verdict == "PRESENT" and len(self._w) >= self.window:
            mean = sum(self._w) / len(self._w)
            if mean > 0.1:
                var = sum((v - mean) ** 2 for v in self._w) / len(self._w)
                cv = math.sqrt(var) / mean
                if cv > self.cv_max:
                    verdict = "ABSENT"
                    why = (f"size unstable: {cv:.2f} CV over {self.window} ticks "
                           f"— not one physical object")
        self.counts[verdict] = self.counts.get(verdict, 0) + 1
        return verdict, why


class TargetLock:
    """Binds the controller to ONE instance of the named class, not to whichever
    instance the detector happens to like this tick.

    The gap this closes. "a building" names a KIND, and the map has nine city
    blocks; "a car" names a kind, and the traffic scene has four. The detector
    answers "where is something of this kind", the controller centres whatever
    box it is handed, and nothing ties one tick's answer to the last. Measured on
    the orbit flights: box-centre discontinuities over 80 px occurred 24, 5 and 8
    times, so the aircraft was chasing whichever building was most salient at
    that moment, and that walks across the map.

    For the car this was solved by accident — colour supplied INSTANCE
    persistence on top of CLASS detection. Nothing supplies it for a building.

    The lock predicts where the held instance should now appear, using the
    aircraft's own yaw change (which moves every object in frame by a known
    number of pixels) and accepts the nearest candidate to that prediction. A
    candidate that is too far from the prediction is a DIFFERENT object, and
    taking it would be a silent target switch.
    """

    def __init__(self, hfov_deg: float = 90.0, gate_frac: float = 0.28,
                 hold_s: float = 2.0):
        self.hfov_deg = hfov_deg
        self.gate_frac = gate_frac      # max jump, as a fraction of image width
        self.hold_s = hold_s            # how long a lock survives with no match
        self.cx = None
        self.last_yaw = None
        self.last_t = None
        self.n_locked = 0
        self.n_switched = 0
        self.n_rejected = 0

    def _predict(self, img_w: int, yaw: float) -> float:
        """Where the held instance should be now, given how far the nose turned.

        A yaw of d radians slides a distant object across the frame by
        d / hfov * width pixels, in the opposite direction to the turn. Without
        this the gate would reject the true target every time the aircraft
        turned, which is precisely when it is tracking hardest.
        """
        if self.cx is None or self.last_yaw is None:
            return None
        d = math.atan2(math.sin(yaw - self.last_yaw), math.cos(yaw - self.last_yaw))
        px_per_rad = img_w / math.radians(self.hfov_deg)
        return self.cx - d * px_per_rad

    def select(self, candidates, img_w: int, yaw: float, now: float):
        """Pick the candidate that is the held instance. `candidates` are the
        detector's boxes, best-scoring first; each is the usual 8-tuple.

        Returns (chosen, switched). `switched` is True when the lock was dropped
        and re-acquired on a different object — the event worth logging, because
        it is the moment the mission silently changes target.
        """
        if not candidates:
            return None, False
        stale = self.last_t is None or (now - self.last_t) > self.hold_s
        pred = None if stale else self._predict(img_w, yaw)
        if pred is None:
            chosen = candidates[0]
            switched = self.cx is not None
            self.n_switched += int(switched)
        else:
            gate = self.gate_frac * img_w
            near = [(abs(float(c[0]) - pred), c) for c in candidates]
            near.sort(key=lambda p: p[0])
            if near[0][0] <= gate:
                chosen = near[0][1]
                self.n_locked += 1
                switched = False
            else:
                # Nothing where the held instance should be. Re-acquire, and say
                # so: this is a target switch, not a continuation.
                self.n_rejected += len(candidates)
                chosen = candidates[0]
                switched = True
                self.n_switched += 1
        self.cx = float(chosen[0])
        self.last_yaw = yaw
        self.last_t = now
        return chosen, switched

    def stats(self) -> dict:
        return {"locked": self.n_locked, "switched": self.n_switched,
                "rejected_candidates": self.n_rejected}


class FenceGuard:
    """Lets the controller see the no-fly zones, so it can stop before them.

    Without this the servo commands "go to the car" every tick and the Shield
    refuses it every tick. Neither changes its mind, so the aircraft chatters
    against the boundary — measured at 659 corrections in 795 ticks, and it looks
    exactly as unsafe as it is.

    A real aircraft knows its own geofence; that is mission data, not target
    data. Nothing here reveals where the car is — only where the aircraft may
    not go. It brakes smoothly on approach and holds at a standoff, while yaw
    keeps tracking so the target stays in view.
    """

    def __init__(self, policy, brake_m: float = 12.0, stand_off_m: float = 3.0,
                 obstacle_map: dict | None = None, min_clearance_m: float = 5.0,
                 street_mask: dict | None = None):
        from guardrail.geometry import fence_polygon
        from guardrail.models import PolygonFence
        self.polys = [fence_polygon(f).buffer(f.margin_m)
                      for f in policy.by_type(PolygonFence)]
        self.brake_m = brake_m
        self.stand_off_m = stand_off_m
        # A detour has to stay on the ROAD, not merely outside the fence.
        #
        # Without this, slide() answers a question narrower than the one being
        # asked. On follow_car_nfz.yaml — a fence spanning the whole corridor,
        # deliberately, so that there IS no way past — it found one anyway by
        # routing around the fence's eastern END at x > 55, which is off the
        # street entirely. Measured: fence_mode was `skirt` on 378 of 498 ticks
        # against `hold` on 442 of 552 before, and Shield interventions went from
        # 0 to 298 as the aircraft was pushed into building clearance.
        #
        # The Shield caught every one of those, which is the system working. But
        # the controller should not be proposing them.
        self.occ = obstacle_map
        self.min_clearance_m = min_clearance_m
        # ...and "on the road" has to be ASKED, not inferred from the absence of
        # obstacles. That inference held only while the obstacle map was built
        # over 15-55 m AGL and so contained nothing but buildings, making every
        # road free by construction. Rebuilt over the flight band it fails both
        # ways: a canopy over a road is occupied at cruise and perfectly
        # drivable, and 461 low structures that had been blocked became free, so
        # detours over rooftops started scoring as legal road again.
        self.street = street_mask

    def clearance(self, px: float, py: float, cap_m: float) -> float:
        """Distance to the nearest mapped obstacle, searched no further than `cap_m`.

        Returns `cap_m` when nothing is within it, so callers can treat the cap
        as "far enough to be uninteresting" without a special case. A local
        scan rather than a distance transform, for the same reason
        `_clear_of_obstacles` uses one: this runs inside the control loop.
        """
        if not self.occ:
            return cap_m
        occ, res = self.occ["occ"], self.occ["res"]
        ox, oy = self.occ["ox"], self.occ["oy"]
        n, m = occ.shape
        r = int(math.ceil(cap_m / res))
        i0 = int(round((px - ox) / res))
        j0 = int(round((py - oy) / res))
        best = cap_m
        for i in range(max(0, i0 - r), min(n, i0 + r + 1)):
            for j in range(max(0, j0 - r), min(m, j0 + r + 1)):
                if occ[i, j]:
                    d = math.hypot(ox + i * res - px, oy + j * res - py)
                    if d < best:
                        best = d
        return best

    def _obstacle_gate(self, x: float, y: float, vx: float, vy: float):
        """The building half of `gate`, shaped exactly like the fence half.

        Brakes for a PREDICTED incursion rather than a shrinking distance, so
        flying parallel to a wall - which is most of a street - is not braked.
        The urgency then comes from how close the obstacle is right now.

        The ring is the policy's own `min_clearance_m`, the same number the
        Shield enforces. The controller stopping at the same line the Shield
        would defend is the point: the Shield becomes a backstop instead of the
        only thing steering.
        """
        if not self.occ:
            return 1.0, None, False
        speed = math.hypot(vx, vy)
        if speed < 1e-3:
            return 1.0, None, False
        ring = self.min_clearance_m
        ux, uy = vx / speed, vy / speed
        horizon = max(self.brake_m, speed * 3.0)

        # URGENCY IS HOW FAR AHEAD THE INCURSION IS, NOT HOW CLOSE THE WALL IS.
        #
        # The fence half can scale by the current distance because a fence is a
        # region you approach and then leave. Buildings are not like that: they
        # line the street continuously, so current clearance sits at 4-5 m for
        # the whole flight. Scaling by it throttled the aircraft to 11 % of
        # commanded speed twelve metres before anything was in the way, which is
        # precisely how the gap flight lost its car - "slower than the target for
        # 537 of 552 ticks, so it could not keep up no matter which side it
        # chose".
        #
        # Distance-to-incursion has neither problem. Flying parallel to a wall
        # never enters the ring and is never braked; flying at one brakes in
        # proportion to how soon.
        d_now = self.clearance(x, y, self.brake_m)
        a_hit = None
        for a in np.linspace(0.0, horizon, 12)[1:]:
            c = self.clearance(x + ux * a, y + uy * a, self.brake_m)
            # CLOSING, not merely inside. Both halves are needed. Without the
            # ring test, any approach at all would brake. Without the "nearer
            # than now" test, an aircraft already inside the ring and flying
            # OUT of it brakes hardest exactly when it is escaping - measured
            # at scale 0.09 while retreating south from the canopy, which would
            # pin it against the obstacle it was leaving.
            if c <= ring and c < d_now - 1e-6:
                a_hit = float(a)
                break
        if a_hit is None:
            return 1.0, None, False

        k = a_hit / self.brake_m
        return float(np.clip(k, 0.0, 1.0)), d_now, k < 0.35

    def gate(self, x: float, y: float, vx: float, vy: float):
        """Scale a commanded velocity down as it closes on a fence.

        Returns (scale, distance_to_fence, blocked). `scale` is 1 when clear and
        0 at the stand-off, so the approach is a smooth deceleration rather than
        a wall.
        """
        from shapely.geometry import Point
        o_scale, o_d, o_blocked = self._obstacle_gate(x, y, vx, vy)
        if not self.polys:
            return o_scale, o_d, o_blocked
        p = Point(x, y)
        d = min(poly.distance(p) for poly in self.polys)
        speed = math.hypot(vx, vy)
        if speed < 1e-3:
            return 1.0, d, d <= self.stand_off_m

        # Brake for a predicted INCURSION, not for a shrinking distance.
        #
        # Comparing the distance 2 m ahead against the distance now brakes for any
        # motion that closes on the fence, including motion that passes cleanly by
        # it. Inside the gap of follow_car_gap.yaml, flying north up x = 47, the
        # nearest fence point is the corner at (43, 1) and that corner does get
        # nearer — so the gate throttled a trajectory that never enters the zone.
        #
        # Measured on v2_gap: the aircraft found the gap (277 of 552 ticks at
        # x > 43, reaching x = 50.3) and was still throttled to 1.62-1.68 m/s in
        # `near` and `skirt` against a car doing 2.0. Slower than the target for
        # 537 of 552 ticks, so it could not keep up no matter which side it chose.
        # That, not the side choice, is why the gap flight lost the car.
        #
        # Forecasting the actual path answers the right question: does this
        # velocity, held, put the aircraft inside the stand-off within the
        # lookahead? Same idea the Shield uses, and it leaves a parallel pass
        # unbraked.
        ux, uy = vx / speed, vy / speed
        horizon = max(self.brake_m, speed * 3.0)
        d_min = d
        for a in np.linspace(0.0, horizon, 8)[1:]:
            q = Point(x + ux * a, y + uy * a)
            d_min = min(d_min, min(poly.distance(q) for poly in self.polys))
            if d_min <= self.stand_off_m:
                break
        if d_min > self.stand_off_m:
            # Fence clear; the buildings may still have something to say.
            return (o_scale, d if o_d is None else o_d, o_blocked)
        # It does close inside the stand-off somewhere ahead; how urgently is
        # still governed by how far away the fence is right now.
        if d <= self.stand_off_m:
            return 0.0, d, True
        if d >= self.brake_m:
            return min(1.0, o_scale), d, o_blocked
        k = (d - self.stand_off_m) / (self.brake_m - self.stand_off_m)
        f_scale = float(np.clip(k, 0.0, 1.0))
        # Whichever hazard is more urgent governs. Taking the minimum cannot
        # relax the fence behaviour that the fenced policies are tested on.
        return min(f_scale, o_scale), d, (k < 0.35) or o_blocked

    # reach_m stays 14. The detour around follow_car_gap.yaml's fence is 15.0 m
    # once street furniture is in the obstacle map, so 14 misses it - but raising
    # it to 20 lets a guard WITHOUT a street mask find a way around a
    # corridor-spanning fence's END, which is the off-road regression
    # test_a_detour_must_stay_on_the_road exists to catch. The reach cannot move
    # until every caller supplies the street mask.
    # See docs/FINDING-the-occupancy-map-was-looking-elsewhere.md.
    def slide(self, x: float, y: float, vx: float, vy: float, probe_m: float = 8.0,
              reach_m: float = 14.0, step_m: float = 1.0):
        """Which way to sidestep, and how far the detour has to be.

        Braking alone is safe but passive: the aircraft stops at the boundary and
        the target drives away. If the fence does not span the whole corridor
        there is a way past, and this looks for it.

        Returns (ux, uy, cost_m): a unit vector and how far sideways the aircraft
        must travel before the way ahead opens. (0, 0, inf) means neither side
        opens within `reach_m`, and stopping really is the only legal answer.

        WHY IT MEASURES A DETOUR LENGTH RATHER THAN A CLEARANCE

        The first version scored each side by the fence distance at a single
        probe point. That is symmetric information — it says how far the fence is
        on the left and on the right — and it cannot tell which side the aircraft
        can actually get PAST on. Flown against follow_car_gap.yaml, whose fence
        covers x 26..42 and leaves 7 m of road at x 43..50, the aircraft slid WEST
        to x = 30.9. Wrong side entirely: west is the closed end.

        So each side is now scored by the smallest lateral displacement after
        which the forward direction is clear. That is exactly the question — how
        big is the detour — and the side with the shorter answer wins. On the gap
        policy the eastward answer is finite and the westward one is infinite.
        """
        from shapely.geometry import Point
        speed = math.hypot(vx, vy)
        # A BUILDING IS AS GOOD A REASON TO SIDESTEP AS A FENCE.
        #
        # This used to return "no opinion" whenever the policy declared no
        # no-fly zone, and the demo policy declares none - so on every tracking
        # flight the whole of this function was dead code. Everything below
        # already consults the occupancy grid and the street mask; only the
        # gate at the top was fence-shaped.
        #
        # What that cost is on record. Chasing the car north along x = 38, the
        # flight-band map is BLOCKED at (38, 22) - a canopy, road underneath,
        # solid at cruise - and the clearance reachable on that line falls to
        # 0.0 m. Five metres east, at x = 43, it is 5.0 m. The controller could
        # not see that, so it commanded 4 m/s due north into the canopy for
        # forty consecutive ticks and the Shield turned every one of them away.
        if speed < 1e-3 or (not self.polys and not self.occ):
            return 0.0, 0.0, float("inf")
        ux, uy = vx / speed, vy / speed
        lx, ly = -uy, ux                      # left of the commanded heading

        def _on_street(px: float, py: float) -> bool:
            """Is this point on a road at all? Unmapped counts as not a road."""
            if not self.street:
                return True
            st, res = self.street["street"], self.street["res"]
            # round, matching the grid convention in city_planner.py and the
            # obstacle lookups a few lines below. Truncating read the mask a
            # metre off and disagreed with is_street() on 9.4 % of points -
            # which started to matter the moment slide() began running on
            # unfenced policies, i.e. on every tracking flight.
            i = int(round((px - self.street["ox"]) / res))
            j = int(round((py - self.street["oy"]) / res))
            if not (0 <= i < st.shape[0] and 0 <= j < st.shape[1]):
                return False
            return bool(st[i, j])

        def _clear_of_obstacles(px: float, py: float) -> bool:
            """Far enough from every MAPPED obstacle. Cheap Chebyshev scan of the
            occupancy grid rather than a distance transform, because slide() runs
            inside the 10 Hz control loop."""
            if not self.occ:
                return True
            occ, res = self.occ["occ"], self.occ["res"]
            ox, oy = self.occ["ox"], self.occ["oy"]
            r = int(math.ceil(self.min_clearance_m / res))
            i0 = int(round((px - ox) / res))
            j0 = int(round((py - oy) / res))
            n, m = occ.shape
            for i in range(max(0, i0 - r), min(n, i0 + r + 1)):
                for j in range(max(0, j0 - r), min(m, j0 + r + 1)):
                    if occ[i, j] and math.hypot(ox + i * res - px,
                                                oy + j * res - py) < self.min_clearance_m:
                        return False
            return True

        def _ahead_clear(px: float, py: float) -> bool:
            pts = [(px + ux * a, py + uy * a)
                   for a in (probe_m * 0.5, probe_m, probe_m * 1.5)]
            if self.polys and not all(
                    min(poly.distance(Point(*q)) for poly in self.polys)
                    >= self.stand_off_m for q in pts):
                return False
            return all(_clear_of_obstacles(*q) and _on_street(*q) for q in pts)

        # No blockage, no detour. Without this the probe reaches past nothing
        # when the fence is still far away, BOTH sides score the minimum cost,
        # and the tie is broken by iteration order -- which silently picked WEST,
        # the closed end of the gap policy, for the whole distant approach.
        # "Which way round" is only a question once there is something in the way.
        if _ahead_clear(x, y):
            return 0.0, 0.0, float("inf")

        best_cost, bx, by = float("inf"), 0.0, 0.0
        n = max(1, int(reach_m / step_m))
        for sgn in (1.0, -1.0):
            for k in range(1, n + 1):
                off = k * step_m
                px = x + lx * sgn * off
                py = y + ly * sgn * off
                # Standing here must itself be legal, with the stand-off kept.
                if self.polys and min(poly.distance(Point(px, py))
                                      for poly in self.polys) < self.stand_off_m:
                    continue
                if not (_clear_of_obstacles(px, py) and _on_street(px, py)):
                    continue          # outside the fence but off the street
                # ...and the way ahead from here must be open for a real distance,
                # not merely one step: a one-step gap is a corner, not a route.
                if _ahead_clear(px, py):
                    if off < best_cost:
                        best_cost, bx, by = off, lx * sgn, ly * sgn
                    break                     # shortest detour on this side
        return bx, by, best_cost


def annotate(img, det, hud: dict):
    """Draw what the drone is actually seeing and deciding, for the demo.

    A raw camera frame proves nothing to a viewer — the whole claim is that a
    word picked the box and the box drove the aircraft, so both have to be on
    screen at once.
    """
    from PIL import ImageDraw
    im = img.copy()
    d = ImageDraw.Draw(im)
    W, H = im.size
    if det is not None:
        cx, cy, bw, bh = det[0], det[1], det[2], det[3]
        x0, y0, x1, y1 = cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2
        d.rectangle([x0, y0, x1, y1], outline=(0, 255, 0), width=2)
        d.line([(cx, 0), (cx, H)], fill=(0, 255, 0, 90))
        tag = f"{hud['query']}  p={det[4]:.3f}"
        if len(det) > 7:
            tag += f"  colour={det[7]:.2f}"
        d.text((max(2, x0), max(2, y0 - 11)), tag, fill=(0, 255, 0))
    d.line([(W / 2, H / 2 - 8), (W / 2, H / 2 + 8)], fill=(255, 255, 255))
    d.line([(W / 2 - 8, H / 2), (W / 2 + 8, H / 2)], fill=(255, 255, 255))
    lines = [
        f"t {hud['t']:5.1f}s   alt {hud['alt']:4.1f} m",
        f"bearing {hud['brg']:+6.1f}deg   fwd {hud['fwd']:+4.1f} m/s",
        (f"separation {hud['sep']:5.1f} m" if hud.get("sep") is not None
         else "separation n/a"),
        ("NFZ - SKIRTING AROUND" if hud.get("fence_mode") == "skirt"
         else "NFZ AHEAD - HOLDING" if hud.get("fence_hold")
         else (f"no-fly zone {hud['fence_d']:.0f} m ahead"
               if hud.get("fence_d") is not None and hud["fence_d"] < 20
               else "GUARDRAIL: correcting" if hud["shield"] else "GUARDRAIL: clear")),
        {"track": "TARGET LOCKED", "coast": "COASTING on last motion",
         "search": "SEARCHING ...", "scan": "SCANNING for target"}.get(
            hud.get("mode"), "SCANNING for target"),
    ]
    for i, ln in enumerate(lines):
        d.text((6, 6 + 11 * i), ln,
               fill=((255, 90, 90) if ("HOLDING" in ln or "correcting" in ln)
                     else (120, 220, 255) if "SKIRTING" in ln
                     else (255, 200, 90) if "no-fly zone" in ln
                     else (255, 255, 255)))
    return im


class Grounder:
    """Open-vocabulary detector in a background thread: text in, box out.

    Publishes the latest detection; the control loop reads it without blocking.
    Detection scores for a small distant object are low in absolute terms
    (0.03-0.07 measured at 22 m), so the box is accepted on a low threshold and
    filtered by continuity instead: a detection far from the last accepted one is
    rejected unless nothing has been seen for a while. That rejects the
    occasional confident tree without needing a confident car.
    """

    def __init__(self, obs: SemanticObs, query: str, thresh: float = 0.02,
                 jump_frac: float = 0.35, log_path: Path | None = None,
                 colour_min: float = 0.10, lock: "TargetLock | None" = None,
                 colour_mask: bool = True, legacy_sat: bool = False):
        # Measure the colour on the object's pixels rather than the whole box.
        # `mask_stats` records which path actually ran, because a fix that
        # silently never engages is the failure mode to watch for here - the
        # start-heading fault failed silently for an unknown number of runs.
        self.colour_mask = bool(colour_mask)
        self.legacy_sat = bool(legacy_sat)
        self.mask_stats: dict = {"masked": 0, "whole_box": 0}
        self.obs = obs
        # Instance persistence. None keeps the historical behaviour exactly:
        # take the best-ranked candidate every tick and never ask whether it is
        # the same object as last tick.
        self.lock = lock
        self.query = query
        self.thresh = thresh
        self.jump_frac = jump_frac
        self.colour = colour_word(query)
        self.colour_min = colour_min
        if self.colour:
            print(f"[grounder] colour prior: {self.colour!r} "
                  f"(a box must be >={colour_min:.0%} that colour to qualify)")
        self.log_path = log_path
        self._lock = threading.Lock()
        self._stop = False
        self._seq = 0
        self._det = None          # (cx, cy, w, h, score, W, H, colour)
        self._app = None          # HSV histogram of the chosen box
        self._t = 0.0             # last time the detector RAN
        self._t_det = 0.0         # last time it actually FOUND the target
        self.drop_after = 8.0     # forget the box entirely after this long
        self._infer_ms = 0.0
        self._pre_ms = 0.0
        self._fwd_ms = 0.0
        self._n_seen = 0
        self._n_miss = 0
        self._thread = threading.Thread(target=self._worker, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True

    def _worker(self):
        import torch
        from transformers import OwlViTProcessor, OwlViTForObjectDetection
        t0 = time.time()
        proc = OwlViTProcessor.from_pretrained(DETECTOR_ID)
        model = OwlViTForObjectDetection.from_pretrained(DETECTOR_ID).to("cuda").eval()
        print(f"[grounder] {DETECTOR_ID} loaded in {time.time()-t0:.0f}s "
              f"(VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB)", flush=True)
        queries = [[self.query]]
        last = None
        while not self._stop:
            img = self.obs.get_front_native()
            if img is None:
                time.sleep(0.05)
                continue
            # One depth frame per inference, not per candidate box. It indexes
            # straight into the scene image because both captures are configured
            # at the same resolution and the same field of view - the same
            # property range_from_depth already relies on.
            dep = self.obs.get_depth() if self.colour_mask else None
            W, H = img.size
            # Split, because the total alone cannot say WHY it is slow. Offline
            # on an idle GPU this whole block is 66 ms (26.7 CPU preprocessing,
            # 36.2 forward) at the real 400x225 input - see
            # experiments/profile_owlvit.py. In flight it measures 287 ms, which
            # is 4.3x slower and is NOT explained by capture resolution: it
            # barely moved across a 2.4x change in Chase pixels and not at all
            # across a 1.8x change in window pixels.
            #
            # So the slowdown comes from sharing the machine with the simulator,
            # and the two halves point at different culprits. Preprocessing is
            # CPU and would be starved by the recorder's JPEG encoding, the
            # control loop and msgpack deserialisation. The forward pass is GPU
            # and would be starved by Unreal's rendering. Logging them apart is
            # the difference between knowing and guessing.
            t = time.time()
            inputs = proc(text=queries, images=img, return_tensors="pt").to("cuda")
            t_pre = time.time()
            with torch.no_grad():
                out = model(**inputs)
            torch.cuda.synchronize()
            t_fwd = time.time()
            pre_ms = (t_pre - t) * 1000
            fwd_ms = (t_fwd - t_pre) * 1000
            ms = (t_fwd - t) * 1000
            res = proc.post_process_object_detection(
                out, threshold=0.0,
                target_sizes=torch.tensor([[H, W]]).to("cuda"))[0]
            sc, bx = res["scores"], res["boxes"]
            det, switched, cands = None, False, []
            if len(sc):
                # Score every plausible box, do not just take the detector's top
                # one. Ranking by detector score alone is what let a city object
                # win and hold the aircraft 60 m from the car.
                for i in sc.argsort(descending=True)[:12]:
                    s = float(sc[i])
                    if s < self.thresh:
                        break
                    x0, y0, x1, y1 = [float(v) for v in bx[i].tolist()]
                    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                    if last is not None and (time.time() - last[1]) < 1.5:
                        if abs(cx - last[0]) > self.jump_frac * W:
                            continue          # too far from where it just was
                    cm = colour_match(img, (x0, y0, x1, y1), self.colour,
                                      depth=dep, stats=self.mask_stats,
                                      legacy_sat=self.legacy_sat)
                    if self.colour is not None and cm < self.colour_min:
                        continue              # right shape, wrong colour
                    cands.append((cx, cy, x1 - x0, y1 - y0, s, W, H, cm))
            if cands:
                # Rank by score-and-colour, then let the instance lock decide
                # WHICH of the survivors is the one we were already following.
                # Ranking alone answers "is this the right kind of thing"; it has
                # nothing to say about "is this the same one", and with several
                # identical vehicles or nine city blocks those are different
                # questions.
                cands.sort(key=lambda c: -(c[4] * (0.25 + 0.75 * c[7])))
                if self.lock is None:
                    det = cands[0]
                else:
                    yaw_now = (self.obs.pose[2] if getattr(self.obs, "pose", None)
                               else 0.0)
                    det, switched = self.lock.select(cands, W, yaw_now, time.time())
            app = None
            if det is not None:
                x0 = det[0] - det[2] / 2, det[1] - det[3] / 2
                app = appearance(img, (x0[0], x0[1],
                                       det[0] + det[2] / 2, det[1] + det[3] / 2))
            with self._lock:
                self._seq += 1
                self._infer_ms = ms
                self._pre_ms = pre_ms
                self._fwd_ms = fwd_ms
                # `_t` is when the detector last RAN. `_t_det` is when it last
                # actually FOUND something. Conflating the two was a real bug:
                # the control loop aged the detection against `_t`, which
                # refreshes on every frame including misses, so a stale box was
                # treated as fresh forever. The aircraft chased a box that was no
                # longer there and the HUD kept reporting TARGET LOCKED.
                self._t = time.time()
                if det is not None:
                    self._det = det
                    self._app = app
                    self._t_det = time.time()
                    self._n_seen += 1
                    last = (det[0], time.time())
                else:
                    self._n_miss += 1
                    # drop the box once it is unusably old, so nothing downstream
                    # can accidentally act on it
                    if self._t_det and time.time() - self._t_det > self.drop_after:
                        self._det = None
            if self.log_path is not None:
                rec = {"seq": self._seq, "t": self._t, "infer_ms": round(ms, 1),
                       "query": self.query,
                       "switched": bool(switched),
                       "det": (None if det is None else
                               {"cx": round(det[0], 1), "cy": round(det[1], 1),
                                "w": round(det[2], 1), "h": round(det[3], 1),
                                "score": round(det[4], 4), "colour": round(det[7], 3),
                                "img_w": det[5], "img_h": det[6]})}
                with self.log_path.open("a", encoding="utf-8") as fh:
                    fh.write(json.dumps(rec) + "\n")

    def latest(self) -> dict:
        with self._lock:
            return {"seq": self._seq, "t": self._t, "t_det": self._t_det,
                    "det": self._det, "app": self._app, "infer_ms": self._infer_ms,
                    "pre_ms": self._pre_ms, "fwd_ms": self._fwd_ms,
                    "n_seen": self._n_seen, "n_miss": self._n_miss}


def servo(det, img_w: int, yaw_gain: float, want_w_frac: float,
          speed_max: float, hfov_deg: float = 90.0):
    """Box in the image -> (yaw rate rad/s, forward speed m/s, bearing error rad).

    Bearing comes from the horizontal offset scaled by the real horizontal FOV,
    so the yaw command is in true angular units rather than arbitrary pixels.
    Forward speed closes on a target apparent width: too small means too far, so
    accelerate; too large means too close, so back off.
    """
    cx, _cy, bw, _bh, _s, W, _H = det[:7]
    off = (cx - W / 2) / (W / 2)                 # -1 left .. +1 right
    bearing = math.radians(hfov_deg / 2.0) * off
    yaw_rate = yaw_gain * bearing
    w_frac = bw / W
    err = (want_w_frac - w_frac) / max(1e-3, want_w_frac)
    fwd = float(np.clip(err * speed_max, -0.4 * speed_max, speed_max))
    # do not charge forward while the target is far off to one side
    fwd *= max(0.0, math.cos(bearing))
    return yaw_rate, fwd, bearing


async def fly(args) -> int:
    from projectairsim import Drone, EnvActor, ProjectAirSimClient, World

    # Resolve the subject's real width from what was actually named, unless the
    # command line said otherwise. Printed either way: a width the flight chose
    # for itself has to be visible, because it scales every range estimate.
    # The word the width came from is also what a SubjectStandoff rule matches
    # on, so "a pedestrian" selects both the 0.5 m width and the 10 m stand-off
    # from one phrase rather than two flags that can disagree.
    subject_class = subject_width(args.object)[1]

    if args.object_width_m is None:
        args.object_width_m, matched = subject_width(args.object)
        if matched:
            print(f"[subject]  width prior: {args.object_width_m:.2f} m "
                  f"(from {matched!r} in {args.object!r})")
        else:
            print(f"[subject]  WARNING no width known for {args.object!r}; "
                  f"falling back to {args.object_width_m:.2f} m, a car. If the "
                  f"subject is not car-sized, every range estimate is wrong by "
                  f"the ratio - pass --object-width-m.")
    else:
        print(f"[subject]  width {args.object_width_m:.2f} m (from the command line)")

    # Seed every RNG the flight can reach. WP4 requires random_seed in the
    # determinism manifest, and a manifest that records a seed nothing honours
    # would be worse than no manifest at all.
    random.seed(args.seed)
    np.random.seed(args.seed & 0xFFFFFFFF)

    out = ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)
    for f in ("flight_log.jsonl", "detections.jsonl"):
        if (out / f).exists():
            (out / f).unlink()

    policy = load_policy(args.policy)
    cmap = city_planner.load_occ(args.citymap)
    smap = None
    if policy.by_type(ObstacleClearance) and cmap is not None:
        smap = {"occ": cmap["occ"], "res": cmap["res"],
                "ox": cmap["ox"], "oy": cmap["oy"]}
    shield = Shield(policy, lookahead_s=3.0, dt=0.5, obstacle_map=smap)
    clr = policy.by_type(ObstacleClearance)

    # Where the roads are, which is a different question from where the
    # obstacles are - see demo/build_street_mask.py. The detour search needs
    # both: outside the fence, clear of obstacles, AND on a road.
    street = None
    _sm = Path(args.citymap).parent / "street.npz"
    if _sm.is_file():
        from build_street_mask import load_street
        street = load_street(_sm)
        print(f"[fence] street mask loaded: {int(street['street'].sum())} road cells")
    else:
        print(f"[fence] WARNING no street mask at {_sm}; detours will be judged "
              f"on obstacles alone, which lets slide() route off-road. "
              f"Build it with: python demo/build_street_mask.py")

    fence = FenceGuard(policy, brake_m=args.fence_brake,
                       stand_off_m=args.fence_standoff, obstacle_map=smap,
                       min_clearance_m=(clr[0].min_clearance_m if clr else 5.0),
                       street_mask=street)

    audit = AuditLogger(out / "audit.jsonl", policy)   # the POLICY, so a hot-applied rule restamps the hash
    if fence.polys:
        print(f"[fence] {len(fence.polys)} no-fly zone(s) known to the controller: "
              f"brake from {args.fence_brake:.0f} m, hold at {args.fence_standoff:.0f} m")

    print(f"[policy]  {policy.policy_id} {policy.policy_hash}")
    print(f"[follow]  query = {args.object!r}")
    print("[follow]  the ONLY steering input is where the detector puts the box; "
          "no target coordinates reach the controller")

    obs = SemanticObs()
    rows, traj, n_touched = [], [], 0
    car = None
    traffic = None
    env_car = None
    people = None
    n_absent = 0
    # Initialised at function scope, not inside the `async with`. When the sim
    # fails to connect the block raises before its own initialisers run, and the
    # metrics section then dies with UnboundLocalError - which buries the real
    # error under a confusing one. A failed flight should report zeros.
    tick = 0
    nfz_hold_ticks = 0
    guard_hold_ticks = 0        # any hold: fence OR building

    presence = PresenceMonitor(args.object, args.colour_min)
    rng_f = None            # low-passed range, for the orbit radial term
    orbit_fwd_prev = 0.0
    client = ProjectAirSimClient()
    client.connect()
    grounder = None
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)

        # MAKE THE SIMULATOR WINDOW SHOW THE SIMULATION.
        #
        # The reported symptom was "window berukuran kecil dan tidak menampilkan
        # simulasi" - a small window, blank white, no scene. It is not a
        # rendering fault and it is not the depth captures being expensive,
        # which is what I wrongly blamed first.
        #
        # `SwitchStreamingView` binds the game window to "the next available
        # camera with streaming-enabled=true". This robot declares exactly two,
        # and the FIRST is the FrontCamera DEPTH stream at 400x225. So the main
        # view is a depth image: the window resizes itself to 400x225 and shows
        # near-white, because everything in the scene is far away and depth maps
        # to white. Exactly the reported symptom, and it only appears once a
        # client loads the scene - an idle simulator still shows the level,
        # which is why this looked fixed when it was not.
        #
        # The depth stream cannot simply be turned off: `streaming-enabled: true`
        # on it is load-bearing for the range signal (with it false the depth
        # arrives as all zeros, silently). So advance the view instead. One
        # switch lands on the Chase camera, which is the third-person view of
        # the aircraft - the thing worth watching anyway.
        for _ in range(max(0, args.view_switch)):
            try:
                world.switch_streaming_view()
            except Exception as e:
                print(f"[view] could not switch the main view "
                      f"({type(e).__name__}: {e})")
                break
        if args.view_switch:
            print(f"[view] simulator window switched {args.view_switch}x -> "
                  f"showing the Chase camera, not the depth stream")

        drone = Drone(client, world, "Drone1")
        client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                         lambda _, m: obs.put_front(m))
        client.subscribe(drone.sensors["DownCamera"]["scene_camera"],
                         lambda _, m: obs.put_down(m))
        # Range signal. Optional by design: an older robot config has no depth
        # capture on FrontCamera, and every mission except the orbit works
        # perfectly well on apparent width, so a missing stream degrades to the
        # width servo rather than failing the flight.
        have_depth = False
        try:
            client.subscribe(drone.sensors["FrontCamera"]["depth_camera"],
                             lambda _, m: obs.put_depth(m))
            have_depth = True
            print("[range] FrontCamera depth stream subscribed")
        except Exception as exc:
            print(f"[range] no depth stream ({type(exc).__name__}) — "
                  "falling back to apparent box width")
        view_dir = None
        recorder = None
        if args.save_view:
            try:
                client.subscribe(drone.sensors["Chase"]["scene_camera"],
                                 lambda _, m: obs.put_chase(m))
                view_dir = out / "view"
                (view_dir / "tps").mkdir(parents=True, exist_ok=True)
                (view_dir / "fpv").mkdir(parents=True, exist_ok=True)
                recorder = FrameRecorder(obs, view_dir, annotate,
                                         hz=args.record_hz,
                                         height=args.record_height)
                recorder.start()
                print(f"[view] recording {args.record_hz:.0f} Hz at "
                      f"{args.record_height}p on its own thread -> {view_dir}")
            except Exception as exc:
                print(f"[view] no Chase camera in this config ({type(exc).__name__})")

        if not args.no_car:
            # Any FIXED route gets the two stops, not just --straight. The
            # stop-and-go is the clearest evidence the aircraft is tracking the
            # car rather than flying down the same street: a follower has to
            # stop too, and pull away when the car does. Gating it on --straight
            # alone silently dropped it the moment --route was introduced.
            stops = ([(0.30, args.car_stop_s), (0.62, args.car_stop_s)]
                     if (args.route or args.straight) and args.car_stop_s > 0
                     else None)
            if args.traffic > 0:
                # Several vehicles, same mesh, only the target painted. `car`
                # stays a MovingCar (the fleet's target), so every downstream
                # use — ground-truth separation, the HUD, the metrics — is
                # unchanged and the traffic is purely additive.
                traffic = city_traffic.Traffic(world, fleet=city_traffic.default_fleet(
                    n_background=args.traffic, speed=args.car_speed,
                    target_stops=stops, bg_every=args.traffic_every,
                    mode=args.traffic_mode,
                    models=city_traffic.glb_dir(args.glb_dir),
                    route_name=(args.route or "straight")))
                traffic.spawn()
                car = traffic.target
            else:
                # The lone subject gets the same upgrade as the fleet when the
                # models are installed. It is the same detector and the same
                # colour gate, so the measured gain applies here too: the glTF
                # taxi scores 0.108 as "a yellow car" with colour_match 0.317,
                # against 0.047 / 0.119 for the orange buggy at the same pose.
                spec = None
                d = city_traffic.glb_dir(args.glb_dir)
                if d is not None and (d / city_traffic.GLB_TARGET[0]).is_file():
                    spec = moving_car.CarSpec()
                    spec.glb_path = str(d / city_traffic.GLB_TARGET[0])
                    spec.materials = []
                    spec.desc_match = f"a {city_traffic.GLB_TARGET[1]} car"
                    spec.desc_mismatch = "a red car"
                if args.park_at:
                    px, py = [float(v) for v in args.park_at.split(",")]
                    # A one-metre route driven once: the car reaches the far end
                    # almost immediately and stays there. pose_at clamps, and
                    # update() now skips the no-op teleport, so a parked subject
                    # costs nothing per tick.
                    car = moving_car.MovingCar(
                        world, speed_mps=0.5, route=[(px, py), (px, py + 1.0)],
                        one_shot=True, phase_s=0.0, spec=spec)
                else:
                    # --route wins; --straight is kept so every existing
                    # command line and every recorded flight keeps its meaning.
                    route_name = args.route or ("straight" if args.straight else None)
                    fixed = moving_car.ROUTES.get(route_name) if route_name else None
                    car = moving_car.MovingCar(
                        world, speed_mps=args.car_speed,
                        route=fixed,
                        one_shot=fixed is not None,
                        phase_s=(0.0 if fixed is not None else 10.0),
                        stops=stops, spec=spec)
                    if route_name:
                        print(f"[car] route '{route_name}': "
                              f"{car.path.total:.0f} m, {car.lap_time:.0f} s to drive")

                # Prefer the environment actor: the simulator interpolates the
                # whole route at render rate, instead of the client teleporting
                # once per control tick. Teleporting sampled a continuous motion
                # model at 8.69 Hz in 54 cm steps against a 15.57 Hz capture, so
                # 44 % of frames during motion repeated a position - the whole of
                # the "choppy" complaint.
                #
                # Falls back to spawning if the actor is not in the scene, so a
                # scene config without it still flies.
                if args.car_mode != "spawn":
                    try:
                        env_car = EnvActor(client, world, ENV_CAR_NAME)
                        info = car_trajectory.upload(world, car,
                                                     seconds=args.max_s + 5.0)
                        car.actual_name = None      # nothing to teleport or destroy
                        print(f"[car] driven by the simulator: {info['samples']} "
                              f"samples over {info['seconds']:.0f}s at "
                              f"{info['sample_hz']:.0f} Hz, one upload")
                    except Exception as exc:
                        env_car = None
                        print(f"[car] env actor {ENV_CAR_NAME!r} unavailable "
                              f"({type(exc).__name__}: {exc}); falling back to "
                              f"per-tick teleport, which will look choppier")
                        car.spawn()
                else:
                    car.spawn()

        # Scenery. Spawned once and, for all but a couple of them, never touched
        # again - a standing figure costs no per-tick RPC, so it cannot take
        # anything from the detector. Default 0 so every recorded flight and every
        # existing command line keeps its meaning.
        if args.pedestrians > 0 and street is not None:
            import numpy as _np
            _b = Path(args.citymap).parent / "occ_day_highband_15to55.npz"
            _bld = _np.load(_b)["occ"] if _b.is_file() else None
            if _bld is None:
                print("[people] no building mask; cannot tell a pavement from a road, "
                      "so nobody is placed")
            else:
                _route = moving_car.ROUTES.get(args.route or "turn") or []
                _samp = [(x, y) for x, y in _route]
                people = people_mod.Pedestrians(
                    world, street, _bld, count=args.pedestrians,
                    walking=args.pedestrians_walking, seed=args.seed,
                    people_dir=args.people_dir, avoid=_samp)
                people.spawn()


            # DOES THE QUESTION MATCH THE SUBJECT?
            #
            # This exact mismatch is what made detection worse rather than
            # better once already: the target mesh was changed to SKM_SportsCar
            # for its stronger noun score, M_Orange renders WHITE on that mesh,
            # and the query still said "orange". Nothing failed, nothing warned,
            # and the run produced a plausible number for a scene where the
            # colour gate could never fire. Never silent again.
            want = colour_word(args.object)
            have = colour_word(car.spec.desc_match)
            if want and have and want != have:
                print(f"[follow] *** the query asks for {want.upper()} but the "
                      f"subject is {have.upper()} ({car.spec.desc_match}). The "
                      f"colour gate cannot pass. Use --object "
                      f"{car.spec.desc_match!r} ***")
            elif want and not have:
                print(f"[follow] note: query asks for {want.upper()}; the "
                      f"subject makes no colour claim ({car.spec.desc_match!r})")

            # let the actor settle before the first teleport; it is briefly
            # not movable straight after spawning
            for _ in range(10):
                await asyncio.sleep(0.4)
                before = car.pos
                car.update(0.5)
                if car.pos != before or getattr(car, "_fail_streak", 0) == 0:
                    break
            car.update(0.0)

        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        for _ in range(400):
            kin = drone.get_ground_truth_kinematics()
            if -kin["pose"]["position"]["z"] >= args.cruise_alt - 0.5:
                break
            await drone.move_by_velocity_async(0.0, 0.0, -2.0, duration=0.2)
            await asyncio.sleep(0.1)

        # POINT DOWN THE STREET BEFORE HANDING OVER, AND PROVE IT WORKED.
        #
        # This was silently broken, and it was corrupting every flight
        # comparison. The old version commanded
        # `move_by_velocity_async(0,0,0, yaw_is_rate=False, yaw=psi0)`, which
        # does not turn the aircraft, and then simply carried on. Measured
        # across two runs of the same demo:
        #
        #     run A   psi0 =  60.3 deg, car at bearing  83.2 -> +22.9 deg, in frame
        #     run B   psi0 = 135.2 deg, car at bearing  83.2 -> -52.1 deg, OUTSIDE
        #                                                       the 45 deg half-FOV
        #
        # Run B never saw the car at all for 11.5 s, by which time it had driven
        # 23 m away, and the flight never recovered: hit rate 0.331 against
        # 0.740, mean separation 54.8 m against 17.5 m. That is a bigger effect
        # than any change actually being tested, so without this fix an A/B on
        # this demo measures the takeoff lottery, not the change.
        #
        # The heading is now a SCENE CONSTANT - the direction of the road - and
        # not derived from the car. The old code read `car.pos`, which is target
        # ground truth; the demo's whole claim is that the only steering input
        # is where the detector puts the box, so pointing the nose using the
        # answer was worth removing on its own.
        psi0 = math.radians(args.start_heading_deg)
        await (await drone.rotate_to_yaw_async(yaw=psi0))
        await asyncio.sleep(0.5)
        got = quat_yaw(drone.get_ground_truth_kinematics()["pose"]["orientation"])
        err = abs((psi0 - got + math.pi) % (2 * math.pi) - math.pi)
        start_heading_err_deg = math.degrees(err)
        print(f"[flight] start heading {math.degrees(got):.1f} deg "
              f"(asked {args.start_heading_deg:.1f})")
        if err > math.radians(10):
            print(f"[flight] *** the aircraft did not take up the start heading "
                  f"({math.degrees(err):.0f} deg off). Whether the subject is in "
                  f"the first frames is now luck, and this run is not comparable "
                  f"with others ***")

        lock = TargetLock(gate_frac=args.lock_gate) if args.lock_target else None
        grounder = Grounder(obs, args.object, thresh=args.det_thresh,
                            log_path=out / "detections.jsonl",
                            colour_min=args.colour_min, lock=lock,
                            colour_mask=args.colour_mask,
                            legacy_sat=args.colour_legacy_sat)
        grounder.start()
        for _ in range(600):                      # wait for the detector to load
            if grounder.latest()["seq"] > 0:
                break
            await asyncio.sleep(0.5)
        print(f"[flight] cruise {args.cruise_alt:.0f} m — following {args.object!r}")

        limiter = RateLimiter(args.dv_h, args.dv_z)

        # Start the car with the MISSION clock, not at scene setup. The
        # trajectory plays from the instant it is bound, so binding it earlier
        # had the car driving through arming and the climb to cruise.
        if env_car is not None:
            car_trajectory.start(env_car)
            print("[car] trajectory playback started with the mission clock")

        t0, last_seen = time.time(), 0.0
        last_bearing, brg_rate, last_cmd, mode = 0.0, 0.0, (0.0, 0.0), "hold"
        # The presence verdict is computed further down the tick, so the search
        # logic uses the PREVIOUS tick's. That is the honest signal anyway: it is
        # what the system last knew about whether the object is really out there.
        start_heading_err_deg = None
        last_verdict = "UNSURE"
        # Servo on an ESTIMATE of the target rather than the latest box. See
        # demo/target_state.py for the measurements that forced this.
        estimator = TargetState() if args.target_estimator else None
        want_range = (args.want_range if args.want_range > 0
                      else want_range_from_width(args.want_width,
                                                 args.object_width_m))

        # --want-width is an ANGULAR target, so the stand-off it asks for scales
        # with the subject's real width. 0.16 was tuned against a 4 m car and
        # gives 15.8 m; the same flag against a 0.5 m pedestrian asks for 2.0 m,
        # which is not a small adjustment but a different mission.
        #
        # FrontCamera is pitched 20 deg down with a 29.4 deg vertical half-FOV,
        # so it sees the ground from about 0.86 x altitude outward. Inside that
        # the subject is under the aircraft and out of frame, and the tracker
        # would be closing on something it can no longer see.
        blind_m = 0.86 * args.cruise_alt
        if want_range < blind_m:
            print(f"[flight] WARNING stand-off {want_range:.1f} m is inside the "
                  f"camera's near blind spot ({blind_m:.1f} m at {args.cruise_alt:.0f} m "
                  f"altitude): the subject leaves frame before the aircraft gets "
                  f"there. --want-width {args.want_width} was calibrated for a 4 m "
                  f"car; for a {args.object_width_m:.2f} m subject set --want-range "
                  f"directly, or fly lower.")

        last_est_seq = None
        if estimator is not None:
            print(f"[flight] target estimator ON, stand-off {want_range:.1f} m "
                  f"(from --want-width {args.want_width} and a "
                  f"{args.object_width_m:.2f} m subject)")
        while time.time() - t0 < args.max_s:
            tick += 1
            kin = drone.get_ground_truth_kinematics()
            p = kin["pose"]["position"]
            yaw = quat_yaw(kin["pose"]["orientation"])
            state = State(x=p["x"], y=p["y"], up=-p["z"])
            obs.put_pose(p["x"], p["y"], yaw)
            if people is not None:
                # Only the pacing few cost anything here; the standing majority
                # is skipped without an RPC. Scenery is never allowed to fail a
                # flight, so update() swallows its own errors.
                people.update(time.time() - t0)
            if traffic is not None:
                traffic.update(time.time() - t0, tick)
            elif car is not None and env_car is None and tick % 2 == 0:
                # Only when the client owns the motion. With an env actor the
                # simulator is already interpolating the uploaded trajectory,
                # and teleporting on top of it would fight the playback.
                car.update(time.time() - t0)
            elif car is not None:
                # Keep ground truth current for scoring without an RPC: pose_at
                # is a pure function of time and describes exactly the path that
                # was uploaded, so the metrics and the simulator agree.
                car.pos = car.pose_at(time.time() - t0)[:2]

            g = grounder.latest()
            # age against the last DETECTION, not the last inference
            det = g["det"]
            age = (time.time() - g["t_det"]) if g["t_det"] else 1e9
            # FEED THE ESTIMATOR, ONCE PER NEW DETECTION.
            #
            # Only measurements go in: the box centre, the depth range, and the
            # aircraft's own pose. No target ground truth, ever - tgt_x/tgt_y in
            # the log are for scoring and must not reach this.
            if estimator is not None and det is not None and g["seq"] != last_est_seq:
                last_est_seq = g["seq"]
                b_meas = math.radians(45.0) * ((det[0] - det[5] / 2) / (det[5] / 2))
                r_meas = range_from_depth(obs.get_depth(), det) if have_depth else None
                if r_meas is None:
                    r_meas = implied_range_from_width(det, args.object_width_m)
                if r_meas:
                    estimator.update(time.time(), state.x, state.y, yaw,
                                     b_meas, float(r_meas))

            est_obs = (estimator.observe(time.time(), state.x, state.y, yaw)
                       if estimator is not None else None)

            if est_obs is not None:
                # SERVO ON THE ESTIMATE, NOT ON THE LAST BOX.
                #
                # The forward channel used apparent box width, which jitters
                # p95 37.7% between detections in traffic and pushed the raw
                # command 1.897 m/s in a single 0.1 s tick, clipped by the slew
                # limiter on 20% of ticks. Replayed on the recorded flights,
                # servoing on the estimated range instead drops that p95 from
                # 0.6375 to 0.0991 - and the estimate is available on 100% of
                # ticks rather than the 38-55% that carried a fresh box.
                bearing, rng_est = est_obs
                yaw_rate = args.yaw_gain * bearing
                # Proportional term plus the target's own opening rate fed
                # FORWARD. Without the feedforward the loop settles at a lag
                # error - a 2 m/s car held 23.4 m against a 15.8 m stand-off,
                # exactly the (r - want)*gain = 2.0 fixed point. An integrator
                # would also fix it and would wind up every time the Shield
                # overrides the actuator; the estimator already knows the
                # target's velocity, so no memory is needed.
                ff = (estimator.range_rate(state.x, state.y)
                      if args.range_feedforward else 0.0)
                fwd = float(np.clip((rng_est - want_range) * args.range_gain + ff,
                                    -0.4 * args.speed_max, args.speed_max))
                fwd *= max(0.0, math.cos(bearing))
                if last_seen:
                    dt = max(1e-3, time.time() - last_seen)
                    brg_rate = 0.6 * brg_rate + 0.4 * ((bearing - last_bearing) / dt)
                last_seen, last_bearing = time.time(), bearing
                last_cmd = (yaw_rate, fwd)
                mode, seen = "track", True
            elif det is not None and age < args.det_max_age:
                yaw_rate, fwd, bearing = servo(
                    det, det[5], args.yaw_gain, args.want_width, args.speed_max)
                # remember what it was doing, so a gap can be coasted through
                if last_seen:
                    dt = max(1e-3, time.time() - last_seen)
                    brg_rate = 0.6 * brg_rate + 0.4 * ((bearing - last_bearing) / dt)
                last_seen, last_bearing = time.time(), bearing
                last_cmd = (yaw_rate, fwd)
                mode, seen = "track", True
            else:
                # Detection drops on roughly a quarter of ticks — the car leaves
                # frame on a corner, or the detector simply misses. Freezing on
                # every gap meant the drone stopped dead and fell behind, so it
                # coasts on what the target was doing, then searches, and only
                # then gives up.
                seen = False
                if not last_seen:
                    # Never acquired yet. Sweep — do not sit still waiting for a
                    # target to wander into frame. This was the failure mode the
                    # first time: the aircraft held its launch heading for the
                    # whole flight while the car drove a lap behind it.
                    yaw_rate, fwd, bearing = args.search_rate, 0.0, 0.0
                    mode = "search"
                    lost = 0.0
                elif (lost := time.time() - last_seen) < args.coast_s:
                    # keep turning the way the target was moving, decaying
                    k = 1.0 - lost / args.coast_s
                    bearing = last_bearing + brg_rate * lost
                    yaw_rate = float(np.clip(args.yaw_gain * bearing, -1.1, 1.1))
                    fwd = last_cmd[1] * k
                    mode = "coast"
                elif lost < args.coast_s + args.search_s:
                    # A BOUNDED SWEEP AROUND THE LAST BEARING, AND KEEP MOVING.
                    #
                    # What this replaced was `yaw_rate = side * search_rate` --
                    # a constant sign, so despite the comment calling it a sweep
                    # the nose just rotated and never came back. Measured on
                    # demo_traffic: one lost-lock episode swept 292 degrees,
                    # which is the "manuver 360" seen on the video.
                    #
                    # It is the worst possible response to why the lock is
                    # actually lost here. All three episodes across two flights
                    # happened at the SAME place, with the target dead ahead
                    # (aspect 0.4-2.9 deg) and the presence monitor still saying
                    # PRESENT: the car is behind a street tree at the
                    # intersection, roughly where it stops. Turning away from a
                    # bearing that is still correct cannot help.
                    #
                    # Two things do. A sweep that oscillates about the last
                    # bearing keeps re-crossing where the target really is, and
                    # integrates to zero net rotation so the nose does not walk
                    # off. And creeping FORWARD changes the parallax, which is
                    # what actually moves a tree out of the line of sight --
                    # while also holding the range. Standing still cost 9 m of
                    # separation in one 16 s episode (26.3 -> 35.5 m), shrinking
                    # the target and making the reacquisition harder the longer
                    # it took: positive feedback.
                    if args.search_legacy_spin:
                        side = 1.0 if last_bearing >= 0 else -1.0
                        yaw_rate, bearing = side * args.search_rate, 0.0
                    else:
                        yaw_rate = search_sweep_rate(lost - args.coast_s,
                                                     args.search_sweep_deg,
                                                     args.search_period_s)
                        bearing = last_bearing
                    # Creep only while the system still believes the object is
                    # out there. On the --no-car control the verdict is ABSENT
                    # and the aircraft must not go wandering up the street.
                    fwd = (abs(last_cmd[1]) * args.search_creep
                           if last_verdict != "ABSENT" else 0.0)
                    mode = "search"
                else:
                    # Only NOW is a full rotation the right answer. The bounded
                    # sweep has already had `search_s` to re-find something near
                    # the last bearing and failed, so the target is genuinely
                    # somewhere else -- on the circuit demos it is driving a lap
                    # and will come back round. A drone that gives up stays
                    # pointed at nothing for the rest of the flight, which is
                    # what it used to do before there was a scan at all.
                    side = 1.0 if last_bearing >= 0 else -1.0
                    yaw_rate = side * args.search_rate * 0.6
                    fwd, bearing = 0.0, 0.0
                    mode = "scan"

            # altitude hold, in the controller where it belongs — the Shield is a
            # constraint filter, not a regulator
            vz_up = float(np.clip((args.cruise_alt - state.up) * args.alt_gain,
                                  -args.climb_max, args.climb_max))
            # Radial term: forward speed along the nose, which the width servo
            # sizes to hold a stand-off. On its own this is STATION KEEPING —
            # point at the subject, hold distance, hover facing it. Measured on
            # the first orbit attempt: radius held at 37.7 +- 2.6 m (pass) while
            # angular coverage reached only -23.9 deg (fail). The radius was
            # right and the angle never advanced, because nothing in the
            # commanded velocity carried the aircraft AROUND anything.
            #
            # Tangential term: the yaw servo already keeps the nose on the
            # subject, so perpendicular to the nose IS the tangent of a circle
            # about it. One lateral component is the whole orbit.
            #
            # Only while the target is actually in view. Orbiting on a coasted or
            # searched heading would circle a place the subject is not, and the
            # aircraft would spiral away from a target it had already lost.
            orbit_fwd = fwd
            rng_m = range_from_depth(obs.get_depth(), det) if have_depth else None
            verdict, why = presence.update(det, rng_m, g.get("app"))
            # Carried to the next tick, where the search branch decides whether
            # creeping forward is justified. See --search-creep.
            last_verdict = verdict
            if verdict == "ABSENT":
                n_absent += 1
            if args.orbit_speed and mode == "track" and args.orbit_radius > 0:
                if rng_m is not None:
                    # Low-pass the range before it drives anything.
                    #
                    # The sim publishes depth as 16UC1 — uint16 METRES — so the
                    # signal is quantised to 1 m, and the median inside a
                    # shrunken box jumps whenever the box wobbles. A pure
                    # proportional radial term turns each of those jumps into a
                    # command: raising the gain from 0.15 to 0.45 took the flight
                    # from a 24.3 m mean radius to 120.8 m, the aircraft leaving
                    # entirely. The answer is a quieter signal, not a louder
                    # response.
                    rng_f = (rng_m if rng_f is None
                             else (1.0 - args.orbit_rng_lp) * rng_f
                                  + args.orbit_rng_lp * rng_m)
                    # A true range closes the loop the width servo could not.
                    # Positive error means too far, so close in. Same sign as the
                    # width servo, but the signal no longer depends on which face
                    # of the subject happens to be showing.
                    want = float(np.clip(
                        (rng_f - args.orbit_radius) * args.orbit_radial_gain,
                        -args.speed_max, args.speed_max))
                    # ...and rate-limit the command itself, so even a filtered
                    # step cannot become an instant full-speed dash.
                    step = args.orbit_radial_slew * TICK
                    orbit_fwd = float(np.clip(want, orbit_fwd_prev - step,
                                              orbit_fwd_prev + step))
                    orbit_fwd_prev = orbit_fwd
            if args.orbit_speed and mode == "track" and rng_m is None:
                # Clamp the radial term hard while orbiting.
                #
                # Apparent width is a usable range proxy for a car, which looks
                # about the same width from any angle at these distances. It is a
                # BAD one for a 50 x 50 m block: apparent width swings by root-2
                # between face-on and corner-on, so circling the subject makes the
                # width servo read "too close" and command reverse purely from the
                # changing aspect. Measured on the first orbit-mode flight, with
                # the tangential term added and the radial term unclamped: angular
                # coverage improved from -23.9 to +97.7 deg (the tangential term
                # works) while the radius blew out from 37.7 +- 2.6 m to
                # 94.5 +- 51.2 m, reaching 178 m. It orbited, and spiralled away
                # while doing it.
                #
                # There is no range sensor in this path, so width is the only
                # radial feedback there is. Limiting how fast it may act keeps the
                # aspect swing from becoming a spiral while still letting a real
                # range error be corrected, just slowly.
                orbit_fwd = float(np.clip(fwd, -args.orbit_radial_max,
                                          args.orbit_radial_max))
            cvx, cvy = orbit_fwd * math.cos(yaw), orbit_fwd * math.sin(yaw)
            if args.orbit_speed and mode == "track":
                cvx += -args.orbit_speed * math.sin(yaw)
                cvy += args.orbit_speed * math.cos(yaw)
            fscale, fdist, fblocked = fence.gate(state.x, state.y, cvx, cvy)
            gvx, gvy = cvx * fscale, cvy * fscale
            # Commit to a side at the BRAKE distance, not at the stand-off.
            #
            # gate() only raises `fblocked` inside 6.15 m (k < 0.35), by which
            # point the aircraft is already down to 35% speed, and 5 m of lateral
            # travel at that speed costs more ground than a 2 m/s target gives
            # away. Measured on follow_car_gap.yaml: 26.1% of the flight within
            # 30 m against 99.6% unfenced -- the rule cost the path AND the
            # target. Starting the detour at 12 m is what buys the aircraft
            # enough room to still be following something at the far end.
            approaching = (fdist is not None and fdist < fence.brake_m
                           and fscale < 1.0)
            if fblocked or approaching:
                if fblocked:
                    guard_hold_ticks += 1
                    # ...and NFZ holds only when there is actually an NFZ.
                    #
                    # `fblocked` used to mean "a fence blocked us" and now also
                    # means "a building did", because gate() was taught about
                    # obstacles. Counting both under `nfz_hold_ticks` reported
                    # 7 no-fly-zone holds on follow_car.yaml, a policy that
                    # declares no fence at all. The mission-outcome test reads
                    # `nfz_s`, which is measured separately and was never
                    # affected - this is a reported number being wrong, not a
                    # verdict being wrong.
                    if fence.polys:
                        nfz_hold_ticks += 1
                sx, sy, scost = fence.slide(state.x, state.y, cvx, cvy)
                if sx or sy:
                    # Lateral urgency rises as the fence closes, but never waits
                    # for the stand-off: at the brake distance it is already a
                    # third of slide_speed, which is what makes it anticipatory.
                    urgency = float(np.clip(1.0 - fscale, 0.33, 1.0))
                    lat = args.slide_speed * urgency
                    gvx += sx * lat
                    gvy += sy * lat
                    fmode = "skirt"
                else:
                    fmode = "hold" if fblocked else "near"
            else:
                fmode = "clear" if fdist is None or fdist > 20 else "near"
            raw = Action4D(vx=gvx, vy=gvy, vz_up=vz_up, yaw_rate=yaw_rate)
            smooth = limiter(raw)

            # Tell the Shield where the subject is, so SubjectStandoff rules can
            # bind. The position comes from the ESTIMATOR's state vector, which
            # is built from bearing, range and the aircraft's own pose - so this
            # carries no target ground truth and the no-leak property holds.
            #
            # Cleared to None the moment the estimator has nothing, and that is
            # required rather than tidy: a stale position would have the Shield
            # enforcing a stand-off from where the subject used to be, which is
            # both wrong and invisible in the logs.
            if estimator is not None and estimator.x is not None and est_obs is not None:
                shield.set_subject(float(estimator.x[0]), float(estimator.x[1]),
                                   subject_class)
            else:
                shield.set_subject(None)

            d = shield.filter(state, smooth)
            audit.log(tick, d)
            if d.touched:
                n_touched += 1
            e = d.emitted

            traj.append({"x": state.x, "y": state.y, "up": state.up,
                         "touched": d.touched})
            rows.append({
                "t": round(time.time() - t0, 3), "tick": tick,
                "x": state.x, "y": state.y, "up": state.up, "psi": yaw,
                "det_seq": g["seq"], "det_age_s": round(min(age, 99), 3),
                "seen": seen, "mode": mode,
                "fence_d": (round(fdist, 2) if fdist is not None else None),
                "fence_scale": round(fscale, 3), "fence_hold": fblocked,
                "fence_mode": fmode,
                "est": (None if estimator is None else
                        {"served": est_obs is not None,
                         "rng": None if est_obs is None else round(est_obs[1], 2),
                         "spd": round(estimator.speed(), 2),
                         "gated": estimator.n_rejected}),
                "presence": verdict, "presence_why": why,
                "app_sim": (round(presence.last_sim, 3)
                            if presence.last_sim is not None else None),
                "rng_m": (round(rng_m, 2) if rng_m is not None else None),
                "infer_ms": round(g["infer_ms"], 1),
                "pre_ms": round(g.get("pre_ms", 0.0), 1),
                "fwd_ms": round(g.get("fwd_ms", 0.0), 1),
                "det": (None if det is None else
                        {"cx": round(det[0], 1), "cy": round(det[1], 1),
                         "w": round(det[2], 1), "score": round(det[4], 4),
                         "colour": round(det[7], 3)}),
                "bearing_deg": round(math.degrees(bearing), 2),
                "raw": raw.model_dump(), "smooth": smooth.model_dump(),
                "emitted": e.model_dump(),
                "touched": d.touched, "braked": d.braked,
                "violations": [v.model_dump() for v in d.violations],
                "repairs": [r.model_dump() for r in d.repairs],
                # The check on what was FLOWN, not on what was asked for.
                # `violations` above is the check on `raw`, so without this the
                # grant's hard KPI - a P0 seen and then flown anyway - could
                # only be inferred, and guardrail/kpi.py inferred it wrongly.
                # Empty is the good case and the normal one.
                "emitted_violations": [v.model_dump() for v in d.emitted_violations],
                "tgt_x": (car.pos[0] if car else None),
                "tgt_y": (car.pos[1] if car else None),
            })

            await drone.move_by_velocity_async(
                e.vx, e.vy, -e.vz_up, duration=0.3,
                yaw_is_rate=True, yaw=e.yaw_rate)
            if recorder is not None:
                # Hand over state only. The decode, draw and encode happen on the
                # recorder thread: doing them here cost 32-40 ms a tick and drove
                # the control loop from 10 Hz down to 7.4 Hz, which is what the
                # video showed as lag.
                sep = (math.hypot(state.x - car.pos[0], state.y - car.pos[1])
                       if car else None)
                recorder.set_hud({
                    "t": time.time() - t0, "sep": sep,
                    "brg": math.degrees(bearing), "fwd": fwd,
                    "alt": state.up, "shield": d.touched,
                    "query": args.object, "mode": mode,
                    "fence_d": fdist, "fence_hold": fblocked,
                    "presence": verdict, "rng_m": rng_m,
                    "fence_mode": fmode,
                }, det if seen else None)
            if tick % 50 == 0:
                sep = (math.hypot(state.x - car.pos[0], state.y - car.pos[1])
                       if car else float("nan"))
                print(f"  tick {tick}: pos=({state.x:6.1f},{state.y:6.1f},"
                      f"{state.up:4.1f}) {mode.upper():6} "
                      f"brg={math.degrees(bearing):+5.1f} fwd={fwd:4.1f} "
                      f"sep={sep:5.1f}m shield={'HIT' if d.touched else '-'}")
            await asyncio.sleep(TICK)

        grounder.stop()
        for _ in range(600):
            kin = drone.get_ground_truth_kinematics()
            up = -kin["pose"]["position"]["z"]
            if up <= 1.2:
                break
            await drone.move_by_velocity_async(
                0.0, 0.0, max(0.6, min(2.0, up * 0.15)), duration=0.3)
            await asyncio.sleep(0.1)
        await (await drone.land_async())
        drone.disarm()
        drone.disable_api_control()
    except Exception as exc:
        print(f"[warn] flight aborted: {type(exc).__name__}: {exc}")
    finally:
        # WRITE THE FLIGHT LOG FIRST. It used to be written after teardown, and a
        # teardown that hung threw the whole flight away: 70 s flown, 1742 frames
        # recorded, and no flight_log.jsonl, so not one number was recoverable.
        # Nothing below this line may cost us the data again.
        try:
            with (out / "flight_log.jsonl").open("w", encoding="utf-8") as fh:
                for r in rows:
                    fh.write(json.dumps(r) + "\n")
        except Exception as exc:
            print(f"[warn] could not write the flight log: {type(exc).__name__}: {exc}")

        # BOUND THE WHOLE TEARDOWN, not just the disconnect.
        #
        # Every step below is an RPC to the simulator, and a wedged simulator
        # does not raise - it simply never returns, so `try/except` is no
        # protection. Measured twice: a completed flight sat for 48 minutes with
        # the frames written and nothing else, and had to be killed by hand.
        # The flight log is already on disk by this point, so the worst a
        # timeout costs is an undestroyed prop in a simulator we are leaving.
        def _teardown():
            if recorder is not None:
                recorder.stop()
                recorder.join(timeout=5.0)
                # Written to disk, not just printed: the video builder reads it
                # to get the real capture rate. Deriving fps from the flight log
                # is wrong because the recorder outlives the mission clock.
                s = recorder.write_sidecar(Path(view_dir) / "recorder.json")
                print(f"[view] recorder: {s}")
            if grounder is not None:
                grounder.stop()
            try:
                if people is not None:
                    people.destroy()
                if traffic is not None:
                    traffic.destroy()
                elif car is not None:
                    car.destroy()
            except Exception:
                pass
            try:
                client.disconnect()
            except Exception:
                pass

        t_td = threading.Thread(target=_teardown, daemon=True)
        t_td.start()
        t_td.join(timeout=30.0)
        if t_td.is_alive():
            print("[warn] the simulator did not finish the teardown in 30 s. "
                  "Carrying on - the flight data is already written, and the "
                  "simulator is about to be restarted anyway.")

    if not (out / "flight_log.jsonl").exists():   # normally written above
        with (out / "flight_log.jsonl").open("w", encoding="utf-8") as fh:
            for r in rows:
                fh.write(json.dumps(r) + "\n")

    fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
    inside = [pt for pt in traj for f, poly in fences
              if f.altitude_floor_m <= pt["up"] <= f.altitude_ceiling_m
              and poly.contains(Point(pt["x"], pt["y"]))]
    band = policy.by_type(AltitudeEnvelope)
    alt_bad = (sum(1 for pt in traj
                   if pt["up"] < band[0].alt_min_m or pt["up"] > band[0].alt_max_m)
               if band else 0)
    seps = [math.hypot(r["x"] - r["tgt_x"], r["y"] - r["tgt_y"])
            for r in rows if r["tgt_x"] is not None]
    g = grounder.latest() if grounder else {"n_seen": 0, "n_miss": 0}
    metrics = {
        "tag": args.tag, "ticks": len(traj), "object": args.object,
        "detector": DETECTOR_ID,
        "det_seen": g["n_seen"], "det_missed": g["n_miss"],
        "det_hit_rate": round(g["n_seen"] / max(1, g["n_seen"] + g["n_miss"]), 3),
        "det_hz": round((g["n_seen"] + g["n_miss"]) / max(1e-6, len(traj) * TICK), 2),
        "start_heading_err_deg": (None if start_heading_err_deg is None
                                  else round(start_heading_err_deg, 2)),
        "frac_ticks_seen": round(sum(1 for r in rows if r["seen"]) / max(1, len(rows)), 3),
        # Which colour measurement actually ran. A fix that silently never
        # engages looks identical to one that works, so it is counted.
        "target_estimator": ({"enabled": True, **estimator.summary()}
                             if estimator is not None else {"enabled": False}),
        "colour_mask": ({"enabled": bool(args.colour_mask),
                         **(grounder.mask_stats if grounder else {})}
                        if grounder else {"enabled": bool(args.colour_mask)}),
        "mode_frac": {m: round(sum(1 for r in rows if r.get("mode") == m) / max(1, len(rows)), 3)
                      for m in ("track", "coast", "search", "scan")},
        "sep_min_m": round(min(seps), 1) if seps else None,
        "sep_mean_m": round(float(np.mean(seps)), 1) if seps else None,
        "sep_end_m": round(seps[-1], 1) if seps else None,
        "frac_within_30m": round(float(np.mean([s <= 30 for s in seps])), 3) if seps else None,
        "nfz_s": round(len(inside) * TICK, 2), "nfz_entered": bool(inside),
        "alt_violation_s": round(alt_bad * TICK, 1),
        "interventions": n_touched,
        "frac_absent": round(n_absent / max(1, len(traj)), 3),
        "nfz_hold_ticks": nfz_hold_ticks,
        "guard_hold_ticks": guard_hold_ticks,

        "params": {"yaw_gain": args.yaw_gain, "want_width": args.want_width,
                   "speed_max": args.speed_max, "cruise_alt": args.cruise_alt,
                   "alt_gain": args.alt_gain, "det_thresh": args.det_thresh},
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=1), encoding="utf-8")

    # --- WP4 artefacts -----------------------------------------------------
    # The grant requires a six-field determinism manifest per episode, and states
    # that contractual KPI numbers may only come from a run that has one. Ours had
    # none, so nothing measured here was reportable on the grant's own terms.
    manifest = build_manifest(
        policy_hash=policy.policy_hash, model_id=DETECTOR_ID,
        seed=args.seed, scene_path=str(ROOT / "demo" / "pas_config" / SCENE))
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1), encoding="utf-8")

    kpis = kpi_mod.compute(rows, kpi_mod.rule_priorities(policy), metrics)
    graded, reasons = is_kpi_grade(manifest, metrics)
    kpis["kpi_grade"] = graded
    kpis["kpi_grade_reasons"] = reasons
    kpis["manifest"] = manifest
    (out / "kpi.json").write_text(json.dumps(kpis, indent=1), encoding="utf-8")
    metrics["kpi_grade"] = graded
    metrics["p0_violation_escape_rate"] = kpis["p0_violation_escape_rate"]
    (out / "metrics.json").write_text(json.dumps(metrics, indent=1), encoding="utf-8")

    print(f"[kpi] P0 violation escape rate {kpis['p0_violation_escape_rate']} "
          f"(hard limit 0) | repairs {kpis['repair_count']} | "
          f"outcome {kpis['outcome']}")
    print(f"[kpi] manifest: {manifest['code_revision']} / "
          f"{manifest['vla_model_hash']} / seed {manifest['random_seed']} / "
          f"speedup {manifest['sim_speedup']} / {manifest['topology']}")
    if graded:
        print("[kpi] KPI-GRADE: this run's numbers are contractually reportable")
    else:
        print("[kpi] NOT KPI-grade - these numbers are evidence, not contractual "
              "KPI figures:")
        for r in reasons:
            print(f"[kpi]   - {r}")
    (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")

    print(f"\n[report] ticks {len(traj)} | detector {metrics['det_hz']} Hz, "
          f"hit rate {metrics['det_hit_rate']} | target visible on "
          f"{metrics['frac_ticks_seen']*100:.0f}% of ticks")
    print(f"[report] separation: min {metrics['sep_min_m']} m, "
          f"mean {metrics['sep_mean_m']} m, end {metrics['sep_end_m']} m, "
          f"within 30 m {metrics['frac_within_30m']}")
    print(f"[report] guardrail: NFZ {metrics['nfz_s']}s, altitude escape "
          f"{metrics['alt_violation_s']}s, interventions {n_touched}")
    print(f"[report] presence: verdict ABSENT on {metrics['frac_absent']*100:.0f}% "
          f"of ticks (with no target in the scene this should be HIGH)")
    # det_hit_rate is seen/(seen+missed) over the inferences that RAN. If the
    # detector stalls it can read 1.000 on a flight that spent most of its time
    # scanning, which is exactly what one demo_traffic run did: 29 inferences at
    # 0.52 Hz, hit rate 1.000, target actually tracked on 13.6% of ticks. Say so
    # rather than let the headline number flatter a stalled run.
    if metrics["det_hz"] < 2.0:
        print(f"[report] *** the detector only managed {metrics['det_hz']:.2f} Hz "
              f"({metrics['det_seen'] + metrics['det_missed']} inferences). "
              f"hit rate {metrics['det_hit_rate']} is measured over those alone "
              f"and is NOT a tracking result - the target was held on "
              f"{metrics['frac_ticks_seen'] * 100:.0f}% of ticks. Treat this "
              f"flight as unrepresentative ***")

    cmk = metrics.get("colour_mask") or {}
    n_m, n_w = cmk.get("masked", 0), cmk.get("whole_box", 0)
    if cmk.get("enabled") and (n_m + n_w):
        print(f"[report] colour measured on the OBJECT for {n_m}/{n_m + n_w} "
              f"boxes ({n_m / (n_m + n_w) * 100:.0f}%); the rest fell back to "
              f"the whole box")
        if n_m == 0:
            print("[report] *** the depth mask never engaged - the colour gate "
                  "is still measuring the background ***")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", default="a white car",
                    help="what to follow, in words. This is the only thing that "
                         "tells the drone what its target is.")
    ap.add_argument("--policy", default=str(ROOT / "policies" / "follow_car.yaml"))
    ap.add_argument("--citymap", default=str(ROOT / "demo" / "out" / "citymap" / "occ_day.npz"))
    ap.add_argument("--tag", default="vlmfollow")
    ap.add_argument("--max-s", type=float, default=120.0)
    ap.add_argument("--cruise-alt", type=float, default=9.0)
    ap.add_argument("--car-speed", type=float, default=3.0)
    ap.add_argument("--orbit-speed", type=float, default=0.0,
                    help="lateral m/s perpendicular to the nose, which turns "
                         "station-keeping into an orbit. 0 disables it and the "
                         "follow behaviour is byte-identical. Positive circles "
                         "one way, negative the other. Only applied while the "
                         "target is in view: orbiting a coasted heading would "
                         "circle a place the subject is not.")
    ap.add_argument("--orbit-radial-max", type=float, default=0.6,
                    help="cap on the width-servo forward speed while orbiting. "
                         "Apparent width swings by root-2 on a square block "
                         "between face-on and corner-on, and unclamped that "
                         "aspect change alone spiralled the radius from 37.7 m "
                         "to 178 m. Only applies when --orbit-speed is set.")
    ap.add_argument("--orbit-rng-lp", type=float, default=1.0,
                    help="low-pass weight on the depth range for the orbit "
                         "radial term; 1.0 is pass-through. DEFAULT OFF because "
                         "damping was measured and made things WORSE: 0.15 took "
                         "within-30 m from 0.821 to 0.714 at the same gain. Depth "
                         "is quantised to 1 m (uint16 metres) and does jump, but "
                         "filtering it costs more phase than it buys in noise.")
    ap.add_argument("--orbit-radial-slew", type=float, default=99.0,
                    help="max change in the radial command, m/s per second; the "
                         "default is high enough to be inert. See --orbit-rng-lp.")
    ap.add_argument("--park-at", default=None, metavar="X,Y",
                    help="park the car at this world point instead of driving "
                         "a route, for the orbit task. The subject must be "
                         "SMALL in frame: the orbit failed on a 50x50 m city "
                         "block because its box filled 99 percent of the image, "
                         "leaving no geometry to servo, lock or range. A 4.3 m "
                         "car at a 16 m radius is a 68 px box, which is the "
                         "regime the whole pipeline was built for.")
    ap.add_argument("--orbit-radius", type=float, default=0.0,
                    help="target orbit radius in metres, held from the DEPTH "
                         "camera rather than apparent box width. 0 keeps the "
                         "width servo.")
    ap.add_argument("--orbit-radial-gain", type=float, default=0.15,
                    help="m/s of radial correction per metre of range error")
    ap.add_argument("--lock-target", action="store_true",
                    help="bind to ONE instance of the named class instead of "
                         "whichever the detector prefers this tick. Needed when "
                         "the scene holds several of the same kind - four cars, "
                         "or nine city blocks.")
    ap.add_argument("--lock-gate", type=float, default=0.28,
                    help="max accepted jump from the predicted position, as a "
                         "fraction of image width")
    ap.add_argument("--pedestrians", type=int, default=0,
                    help="how many decorative people to place on pavements near "
                         "the route. They are scenery: nothing detects them and "
                         "no rule refers to them yet. Standing figures cost no "
                         "per-tick RPC at all. Default 0 so recorded flights are "
                         "unaffected.")
    ap.add_argument("--pedestrians-walking", type=int, default=2,
                    help="how many of them walk rather than stand. Each walker "
                         "is one teleport per tick, on the same budget the "
                         "detector uses.")
    ap.add_argument("--people-dir", default=None,
                    help="baked figures from tools/bake_glb_poses.py; defaults "
                         "to VLA_PEOPLE_DIR or D:/models/quaternius_people/posed")
    ap.add_argument("--car-mode", choices=["spawn", "envactor"], default="spawn",
                    help="how the subject vehicle moves. 'spawn' (default) is "
                         "the client-side teleport, one per control tick; it can "
                         "carry the yellow taxi glTF, which every measured "
                         "flight uses. 'envactor' uploads the whole route once "
                         "and the SIMULATOR interpolates it at render rate - "
                         "much smoother and no per-frame RPC - but env-actor "
                         "links take a packaged unreal_mesh, not a glTF, and the "
                         "only packaged car renders WHITE. Use it with "
                         "--object 'a white car': with a yellow query the colour "
                         "gate rejects it and the hit rate falls to 0.23.")
    ap.add_argument("--traffic", type=int, default=0,
                    help="number of BACKGROUND vehicles besides the target. "
                         "They are the same mesh and unpainted, so the noun "
                         "cannot separate them and only the colour test can - "
                         "which turns 'the noun does most of the work' from a "
                         "stated limitation into a measurement. Capped at the "
                         "number of lanes (4).")
    ap.add_argument("--traffic-mode", choices=("demo", "experiment"),
                    default="demo",
                    help="demo: distractors differ in MESH as well as paint, "
                         "which is what makes the scene readable and the track "
                         "stable. experiment: same mesh throughout, so colour is "
                         "the only free variable - weaker to watch, stronger as "
                         "evidence.")
    ap.add_argument("--glb-dir", default=None,
                    help="directory of glTF vehicle models. When present, demo "
                         "traffic uses four differently-COLOURED vehicles - the "
                         "packaged path can only produce one colour, because "
                         "this build binds exactly one material. Defaults to "
                         "$VLA_GLB_DIR then D:/models/kenney_car-kit/glb; if "
                         "neither exists the fleet falls back to meshes and says "
                         "so. The models are third-party and NOT in this repo: "
                         "see docs/FINDING-glb-vehicles-aug15.md to install.")
    ap.add_argument("--traffic-every", type=int, default=1,
                    help="ticks between teleports for BACKGROUND vehicles. The "
                         "target always updates every tick. At 2 m/s and every "
                         "2nd tick a vehicle moves 0.4 m between updates.")
    ap.add_argument("--car-stop-s", type=float, default=6.0,
                    help="how long the car pauses at each of two points along "
                         "the straight route. A follower has to stop too, so "
                         "this is the clearest evidence the drone is tracking "
                         "the car and not just flying down the same street. "
                         "0 disables the stops.")
    ap.add_argument("--route", choices=("straight", "turn"), default=None,
                    help="fixed route for the subject. 'straight' is the "
                         "baseline every measured flight used, so it stays the "
                         "one to quote. 'turn' drives up the street and turns "
                         "left at the intersection onto the cross street - "
                         "better to watch, but its separation figures are NOT "
                         "comparable with the straight-route results.")
    ap.add_argument("--straight", action="store_true",
                    help="drive the car in a straight line and park it at the "
                         "end, instead of looping. A circuit looks natural but "
                         "its turns swing the target through the aircraft's "
                         "blind spot — the front camera cannot see closer than "
                         "0.86 x altitude — and every lost lock costs tracking.")
    ap.add_argument("--no-car", action="store_true",
                    help="control condition: fly the same mission with no car "
                         "in the scene")
    ap.add_argument("--yaw-gain", type=float, default=1.2)
    # COUPLED TO CRUISE ALTITUDE, and nobody had written that down until a
    # flight was misdiagnosed twice over it. This is an ANGULAR stand-off: the
    # servo holds the target at this fraction of frame width. At the 9 m cruise
    # the follow policies use, 0.10 is right and both tracking flights score
    # 1.000 within 30 m. follow_car_gap.yaml forces 13 m (its band is 10-17, to
    # clear street furniture the obstacle map cannot see), and at 13 m the same
    # angular stand-off is about 30 m of GROUND distance - which is exactly the
    # metric threshold, so the aircraft sat on it and the score read 0.26.
    # Raising it to 0.20 took that flight to 0.759 with nothing else changed.
    # Any policy that moves the cruise altitude must revisit this.
    ap.add_argument("--want-width", type=float, default=0.10,
                    help="target apparent width as a fraction of the image; "
                         "sets the standoff distance")
    ap.add_argument("--speed-max", type=float, default=4.0)
    ap.add_argument("--alt-gain", type=float, default=0.6)
    ap.add_argument("--climb-max", type=float, default=1.8)
    ap.add_argument("--det-thresh", type=float, default=0.02)
    ap.add_argument("--colour-min", type=float, default=0.10,
                    help="minimum fraction of the OBJECT that must actually be "
                         "the named colour. 0 disables the colour check.")
    ap.add_argument("--target-estimator", dest="target_estimator",
                    action="store_true", default=True,
                    help="servo on an ESTIMATE of the target's position rather "
                         "than on the latest detection box. On by default. The "
                         "box width the forward channel used jitters p95 37.7% "
                         "between detections in traffic, which pushed the raw "
                         "command 1.897 m/s in one 0.1 s tick; replayed on the "
                         "recorded flights the estimator drops that p95 6.4x "
                         "and supplies a signal on 100% of ticks instead of "
                         "38-55%.")
    ap.add_argument("--no-target-estimator", dest="target_estimator",
                    action="store_false",
                    help="restore the box-width servo, as the A/B control arm.")
    ap.add_argument("--no-range-feedforward", dest="range_feedforward",
                    action="store_false", default=True,
                    help="drop the target-velocity feedforward from the "
                         "stand-off loop, leaving pure P. Kept so the "
                         "steady-state lag it removes can be re-measured.")
    ap.add_argument("--range-gain", type=float, default=0.25,
                    help="forward speed per metre of stand-off error, m/s/m")
    ap.add_argument("--want-range", type=float, default=0.0,
                    help="stand-off in metres. 0 derives it from --want-width, "
                         "so old command lines keep their behaviour.")
    ap.add_argument("--object-width-m", type=float, default=None,
                    help="real width of the subject in metres, used to turn an "
                         "apparent box width into a range when depth is "
                         "unusable. Default: derived from --object via "
                         "SUBJECT_WIDTH_M, so 'a pedestrian' gets 0.5 m and "
                         "'a yellow car' gets 4.0 m. Pass a value to override; "
                         "the flight prints which width it used and why.")
    ap.add_argument("--seed", type=int, default=20260817,
                    help="random seed, recorded in the WP4 determinism "
                         "manifest and applied to every RNG the flight "
                         "reaches. Change it to get a genuinely "
                         "different sample, not a different-looking one.")
    ap.add_argument("--colour-legacy-sat", action="store_true",
                    help="restore the old saturation floor (s>90) in place of the "
                         "illumination-tolerant chroma floor, as the control arm "
                         "for an A/B. The chroma change has to earn its place "
                         "against what it replaced.")
    ap.add_argument("--colour-mask", dest="colour_mask", action="store_true",
                    default=False,
                    help="measure the colour on the object's pixels, using depth "
                         "to separate it from the ground it stands on. OFF by "
                         "default, deliberately. It was built for the first "
                         "diagnosis of the intersection dropout - that the zebra "
                         "crossing diluted the yellow fraction - and that "
                         "diagnosis was WRONG: replaying the real detector boxes "
                         "showed the car's own pixels failing the saturation "
                         "floor, which is fixed in colour_match instead. The "
                         "mask is a strictly more accurate measurement and is "
                         "kept, but it has never been shown to help in flight, "
                         "so it does not ship on.")
    ap.add_argument("--no-colour-mask", dest="colour_mask", action="store_false",
                    help="explicit off (already the default).")
    ap.add_argument("--det-max-age", type=float, default=1.0)
    ap.add_argument("--coast-s", type=float, default=2.0,
                    help="after losing the target, keep following its last "
                         "known motion for this long before searching")
    ap.add_argument("--search-s", type=float, default=5.0,
                    help="how long to sweep looking for it before holding still")
    ap.add_argument("--search-rate", type=float, default=0.35,
                    help="yaw rate of the give-up SCAN rotation, rad/s. The "
                         "earlier search sweep is set by --search-sweep-deg "
                         "and --search-period-s instead.")
    ap.add_argument("--view-switch", type=int, default=1,
                    help="how many times to advance the simulator's main view "
                         "after the scene loads. The view binds to the first "
                         "camera with streaming-enabled=true, which here is the "
                         "400x225 DEPTH stream - so with 0 the window shrinks to "
                         "400x225 and shows white, which is the 'small blank "
                         "window' fault. 1 advances to the Chase camera. The "
                         "depth stream cannot just be disabled: its streaming "
                         "flag is load-bearing for the range signal.")
    ap.add_argument("--start-heading-deg", type=float, default=90.0,
                    help="heading the aircraft takes up before the run, degrees "
                         "from North. 90 is +y, along the street the subject "
                         "drives. This is a SCENE CONSTANT, deliberately not "
                         "derived from where the subject is: the demo's claim is "
                         "that the only steering input is the detector's box. "
                         "Fixing it also makes flights comparable - an "
                         "uncontrolled takeoff heading decided whether the "
                         "subject was in frame at all, which swamped every other "
                         "effect.")
    ap.add_argument("--search-legacy-spin", action="store_true",
                    help="restore the old constant-sign search rotation, for an "
                         "A/B against the bounded sweep. Kept because the sweep "
                         "should have to prove itself against what it replaced.")
    ap.add_argument("--search-sweep-deg", type=float, default=25.0,
                    help="half-amplitude of the search sweep about the last "
                         "known bearing. The sweep oscillates rather than "
                         "rotating, so the nose keeps re-crossing where the "
                         "target actually was instead of walking away from it. "
                         "25 deg sits well inside the 45 deg horizontal "
                         "half-FOV, so the last bearing never leaves frame.")
    ap.add_argument("--search-period-s", type=float, default=4.0,
                    help="period of that sweep, seconds")
    ap.add_argument("--search-creep", type=float, default=0.5,
                    help="fraction of the last commanded speed to keep flying "
                         "while searching. Standing still is what let the car "
                         "drive away during a lost lock (26 -> 35 m in one "
                         "episode); creeping forward holds the range AND "
                         "changes the parallax, which is what actually clears "
                         "a street tree from the line of sight. 0 restores the "
                         "old stop-and-spin behaviour.")
    ap.add_argument("--save-view", action="store_true",
                    help="record the third-person (Chase) camera to "
                         "demo/out/<tag>/tps/ for the demo video")
    ap.add_argument("--record-hz", type=float, default=20.0,
                    help="frames per second the recorder writes, on its OWN "
                         "thread. Replaces --view-every, which tied recording "
                         "to the control tick: asking for every tick put two "
                         "JPEG encodes and a draw on the flight path and took "
                         "the loop from 10 Hz to 7.4 Hz.")
    ap.add_argument("--record-height", type=int, default=720,
                    help="height the recorded frames are written at. The "
                         "first-person frame is enlarged to this BEFORE the "
                         "overlay is drawn, so the box and text stay sharp "
                         "instead of being magnified 2.1x afterwards.")
    ap.add_argument("--view-every", type=int, default=0,
                    help="deprecated and ignored; see --record-hz.")
    ap.add_argument("--slide-speed", type=float, default=2.5,
                    help="lateral speed used to skirt round a no-fly zone when "
                         "the direct line is blocked but a gap exists")
    ap.add_argument("--fence-brake", type=float, default=12.0,
                    help="start slowing this far from a no-fly zone")
    ap.add_argument("--fence-standoff", type=float, default=3.0,
                    help="hold this far outside a no-fly zone")
    ap.add_argument("--dv-h", type=float, default=0.4)
    ap.add_argument("--dv-z", type=float, default=0.2)
    return asyncio.run(fly(ap.parse_args()))


if __name__ == "__main__":
    code = main()
    # FORCE THE EXIT.
    #
    # Everything this script produces - flight log, metrics, kpi, manifest,
    # frames - is on disk by the time main() returns. What is not guaranteed is
    # that the simulator client's own receive threads have shut down: they are
    # not daemons, so a disconnect that never completes keeps the interpreter
    # alive indefinitely. Measured: a flight that had written every one of its
    # outputs at 17:37 was still running 88 minutes later, holding up the two
    # demos queued behind it.
    #
    # The teardown above is already bounded, so reaching here means the work is
    # done and only bookkeeping can still be pending. Leaving is correct.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code or 0)
