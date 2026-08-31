"""
OWL-ViT box -> AerialVLA compass phrase. A learned action stage, no coordinate leak.

What this restores and what it costs
------------------------------------
AerialVLA has exactly one prompt slot that demonstrably drives its output: the
`{direction}` compass phrase. Over 108 controlled forward passes
(docs/FINDING-what-drives-aerialvla.md) the commanded yaw is monotonic in that
phrase — "to your left" -0.466 rad/s, "forward-left" -0.167, "straight ahead"
-0.035, "forward-right" +0.121, "to your right" +0.161 — while the `{object}`
slot is inert: correct and wrong colour words give indistinguishable actions.

In every earlier experiment that phrase was computed by
`aerialvla_demo.semantic_direction(pos, yaw, target)` from GROUND-TRUTH TARGET
COORDINATES. That is why those runs were called a coordinate leak: the only
input that steered was one the perception stack had not earned.

This module computes the identical phrase from the OWL-ViT BOX CENTRE:

    frame + "an orange car" -> OWL-ViT -> box centre cx
      -> bearing = 45deg * (cx - W/2)/(W/2)        (servo()'s formula, hfov/2)
      -> compass phrase from the training vocabulary
      -> AerialVLA {direction} -> action tokens -> Action4D -> Shield

Same phrase, legitimately derived. Nothing downstream of the detector knows
where the target is in world coordinates.

The exact vocabulary (this is load-bearing)
-------------------------------------------
Copied verbatim from `semantic_direction()` in demo/aerialvla_demo.py, trailing
space and all. A phrase the LoRA never saw in training is worth nothing, so the
strings are not paraphrased and not reformatted:

    "straight ahead "      "forward-right "      "to your right "
    "to your right rear "  "forward-left "       "to your left "
    "to your left rear "

plus the empty string "" for "no hint available", which this module never emits
(see "No detection" below). `tests/test_vla_bridge.py` sweeps
`semantic_direction()` over the full circle at 0.25 deg and asserts the set it
produces is exactly `DIRECTION_VOCABULARY`, so the two cannot drift apart
silently. `bearing_to_phrase()` is asserted to be its faithful inverse over the
same sweep.

Inverse mapping: which phrase covers which angular band
-------------------------------------------------------
Bearing in degrees, body frame, positive = right of the nose, wrapped to
[-180, 180) exactly as `semantic_direction()` wraps it. A boundary angle always
belongs to the MORE CENTRAL band:

    [-180, -120)  "to your left rear "
    [-120,  -60)  "to your left "
    [ -60,  -15)  "forward-left "
    [ -15,  +15]  "straight ahead "
    ( +15,  +60]  "forward-right "
    ( +60, +120]  "to your right "
    (+120, +180]  "to your right rear "        (+180 wraps to -180)

Only three of the seven are reachable from a detection
------------------------------------------------------
A box only exists inside the camera's field of view. FrontCamera is 90 deg
horizontal, so `bearing_from_box()` is bounded by +-45 deg by construction and
only "forward-left ", "straight ahead " and "forward-right " can ever be
produced at unit gain. The two phrases with the strongest measured response —
"to your left" (-0.466 rad/s) and "to your right" (+0.161) — are structurally
unreachable, and the rear phrases doubly so.

That leaves the bridge operating on the flat middle of the response curve:
-0.167, -0.035, +0.121 rad/s. Our own fine-tune (run2/epoch1) does not help
here; it sharpened exactly the unreachable extremes (-0.540 / +0.508) and left
the reachable band at -0.149 / +0.016 / +0.090.

`bearing_gain` scales the bearing before the band lookup, so gain 2.0 makes a
box at the frame edge read as "to your right " and buys real yaw authority. It
is a GAIN, not geometry: the phrase then overstates where the target is. Default
is 1.0 — the honest mapping — and any run using a gain must say so.

Latency budget (measured, not estimated)
----------------------------------------
    OWL-ViT + servo()   3.24 Hz and 4.31 Hz in flight     -> 0.5 m of target
                        (34-56 ms/frame on an idle GPU)      travel at 2 m/s
    AerialVLA           225 ms/token x 12 tokens = 2.7 s   -> 5.4 m at 2 m/s
                        out-of-process (demo/vla_server.py)
    AerialVLA in-loop   1330 ms/token = 0.11 Hz = ~9 s     -> 18 m at 2 m/s

End to end the bridge is detector age + inference = ~0.3 + 2.7 = ~3.0 s, so the
box that built the prompt is 3 s old when the action lands. The detector runs ~9
times per inference; most boxes are discarded and only the freshest one at
prompt-build time matters. Do not average them — averaging a 3 Hz signal into a
0.37 Hz consumer only adds lag.

State it plainly: THIS BRIDGE WILL TRACK A MOVING CAR WORSE THAN
`follow_vlm.py` DOES. servo() decides 9-12 times more often and its output is
continuous. The intended target of this bridge is a STATIC or slow objective —
the building-orbit task — where a decision every 2.7 s and a three-level command
are adequate because the scene is not running away. Running it against the
moving car is a control experiment, not a demo.

The quantisation cost
---------------------
servo() is continuous proportional control: yaw_rate = yaw_gain * bearing, so at
yaw_gain 1.2 a 15 deg error commands 0.314 rad/s and a 1 deg error commands
0.021 rad/s. The phrase path has three levels inside the FOV and no gradient at
all within a level. The resulting deadband, at hfov 90 and W = 400:

    |bearing| <= 15 deg  ->  |cx - W/2| <= W/6 = 66.7 px

The target can traverse 133 px — one third of the frame width — with the
commanded action completely unchanged. At 20 m range that is 20*tan(15deg) =
5.4 m of lateral movement before the command moves at all, which is also about
one inference period of travel for a 2 m/s car. Static targets never leave the
band; that is the regime this is for.

The deadband is also biased, not neutral: "straight ahead " commands -0.035
rad/s, i.e. -2.0 deg/s, so a dead-centred target still yaws the aircraft left by
~5.4 deg over the 2.7 s the action is held. And the reachable band is asymmetric
(-0.167 left vs +0.121 right, 38% more authority to the left than the right), so
equal and opposite bearing errors do not produce equal and opposite corrections.

No detection: no prompt, deliberately
-------------------------------------
When the detector has nothing, `phrase_for()` and `prompt_for()` return None and
the caller MUST NOT run an inference. The alternatives were both worse:

  * Emit "" (the sentinel `semantic_direction()` returns for no hint). Measured:
    mean forward speed collapses from ~4 m/s to 0.56 m/s and the model emits
    LAND on 11 of 18 real frames. Handing an empty phrase to the model in flight
    is a 61%-per-inference chance of commanding a landing, wherever the aircraft
    happens to be. That is not a fallback, it is a hazard.
  * Emit "straight ahead " and fly on. This asserts the target is in front when
    the system does not know that, and the model answers it with ~3.8 m/s of
    forward speed. The Shield covers fences and BUILDINGS ONLY — the obstacle map
    has no trees or street furniture, and a 9 m flight already hit street
    furniture at (48.3, -0.9) — so 3.8 m/s of confident blind forward flight is
    the least safe option available.

So: no perception, no learned action. The caller falls back to the hand-written
search/scan sweep that `follow_vlm.py` already implements (yaw only, zero
forward speed) and must ZERO the VLA action rather than latch it — at 0.37 Hz a
latched action keeps 4 m/s of blind forward flight alive for seconds after the
target is lost.

Grep-checkable claim
--------------------
There is no target coordinate anywhere in this module: not a parameter, not a
local, not an import. `semantic_direction` is deliberately NOT imported — the
only thing pulled from aerialvla_demo is `build_prompt`, which takes two
strings. `tests/test_vla_bridge.py::test_module_never_references_a_target_
coordinate` tokenizes this file, discards comments and docstrings, and fails on
any of `target`, `target_xy`, `tgt`, `goal`, `truth`, `gt_`, `car_pos`,
`semantic_direction` surviving in the code, then walks the AST and fails if any
function takes an argument with such a name. The leak is structurally
impossible, and that is checked rather than asserted in prose.

Run it to see what it can emit:
    python demo/vla_bridge.py
"""
from __future__ import annotations

import hashlib
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

# build_prompt only — two strings in, one string out. Importing it rather than
# re-implementing it guarantees the prompt is byte-identical to the shape the
# LoRA was trained on, including the "Fly ... and find the target." frame and the
# "<image>\n" prefix. semantic_direction() lives in the same module and is
# deliberately left there: it is the coordinate path.
from aerialvla_demo import build_prompt                              # noqa: E402

# Verbatim from semantic_direction(). The trailing space is part of the string as
# the model saw it; build_prompt() strips it when it joins the sentence, but the
# vocabulary conformance test compares against semantic_direction()'s raw output,
# so the constants keep it.
STRAIGHT_AHEAD = "straight ahead "
FORWARD_RIGHT = "forward-right "
RIGHT = "to your right "
RIGHT_REAR = "to your right rear "
FORWARD_LEFT = "forward-left "
LEFT = "to your left "
LEFT_REAR = "to your left rear "

DIRECTION_VOCABULARY = frozenset({
    STRAIGHT_AHEAD, FORWARD_RIGHT, RIGHT, RIGHT_REAR,
    FORWARD_LEFT, LEFT, LEFT_REAR,
})

# semantic_direction()'s eighth return value: "no hint available". Never emitted
# here — it costs LAND on 11 of 18 real frames. See the module docstring.
EMPTY_PHRASE = ""

# What the caller gets when the detector has nothing. Explicit constant so the
# no-detection path reads the same at every call site.
NO_DETECTION_PROMPT = None

# Left to right. Used for the monotonicity test and for ordering log output; the
# index is an ordinal, not an angle.
PHRASE_ORDER = (LEFT_REAR, LEFT, FORWARD_LEFT, STRAIGHT_AHEAD,
                FORWARD_RIGHT, RIGHT, RIGHT_REAR)

# The inverse mapping, as data. Boundaries belong to the more central band, which
# is why the edges are stated as a half-open pair plus a note rather than a
# simple range: see bearing_to_phrase() for the authoritative chain.
PHRASE_BANDS_DEG = {
    LEFT_REAR: (-180.0, -120.0),
    LEFT: (-120.0, -60.0),
    FORWARD_LEFT: (-60.0, -15.0),
    STRAIGHT_AHEAD: (-15.0, 15.0),
    FORWARD_RIGHT: (15.0, 60.0),
    RIGHT: (60.0, 120.0),
    RIGHT_REAR: (120.0, 180.0),
}

# Mean commanded yaw per phrase, original AerialVLA LoRA, 108 controlled forward
# passes (docs/FINDING-what-drives-aerialvla.md). The two rear phrases were NOT
# in that ablation — it varied five phrases plus the empty one — so their
# response is unmeasured and recorded as None rather than guessed.
PHRASE_YAW_RAD_S = {
    LEFT_REAR: None,
    LEFT: -0.466,
    FORWARD_LEFT: -0.167,
    STRAIGHT_AHEAD: -0.035,
    FORWARD_RIGHT: +0.121,
    RIGHT: +0.161,
    RIGHT_REAR: None,
}

# FrontCamera: 400x225, 90 deg horizontal FOV. servo() takes the same default.
DEFAULT_HFOV_DEG = 90.0

# Hard ceiling on the phrase bearing, for two independent reasons.
#
# Safety: a gain large enough to push the scaled bearing past 180 deg WRAPS, and
# `bearing_to_phrase` faithfully reproduces semantic_direction's wrap. At gain 5
# and hfov 90 a box on the LEFT edge of the frame scaled to -225 deg, wrapped to
# +135, and came out as "to your right rear " — the steering inverted. The
# monotonicity test caught it; the clamp is what makes monotonicity hold for
# every gain instead of only small ones.
#
# Meaning: a box exists because the forward camera saw it, so the target is in
# front. Telling the model the target is behind the aircraft is not exaggeration
# like the rest of the gain, it is a different claim entirely — and the two rear
# phrases are the two the ablation never measured, so their effect on the action
# is unknown. 120 deg is the outer edge of the last measured band.
MAX_PHRASE_BEARING_DEG = 120.0

assert set(PHRASE_BANDS_DEG) == DIRECTION_VOCABULARY, "band table drifted"
assert set(PHRASE_ORDER) == DIRECTION_VOCABULARY, "phrase order drifted"
assert set(PHRASE_YAW_RAD_S) == DIRECTION_VOCABULARY, "response table drifted"
assert EMPTY_PHRASE not in DIRECTION_VOCABULARY, "the empty phrase is not a hint"


def _wrap_deg(deg: float) -> float:
    """Wrap to [-180, 180), byte-for-byte the arithmetic semantic_direction uses.

    It matters at the seam: 180.0 wraps to -180.0, so a target directly behind
    is reported as left rear rather than right rear. Reproducing the wrap rather
    than clamping keeps bearing_to_phrase an exact inverse there too.
    """
    return (deg + 180.0) % 360.0 - 180.0


def bearing_to_phrase(bearing_rad: float) -> str:
    """Bearing (rad, body frame, + = right) -> a phrase the LoRA was trained on.

    Pure function, and the exact inverse of semantic_direction()'s band chain —
    the if-order is replicated rather than rewritten as a range search because
    the order is what decides the boundary angles: the first arm claims
    [-15, +15] inclusive on both ends, so a boundary always resolves to the more
    central phrase. Rewriting it as `lo < a <= hi` lookups silently moved -15
    from "straight ahead " to "forward-left " the first time it was tried.

    Returns a member of DIRECTION_VOCABULARY for every finite input, including
    bearings outside the camera FOV — callers get those only by applying a
    bearing_gain, never from a raw box.
    """
    ang = _wrap_deg(math.degrees(bearing_rad))
    if -15 <= ang <= 15:
        return STRAIGHT_AHEAD
    if 15 < ang <= 60:
        return FORWARD_RIGHT
    if 60 < ang <= 120:
        return RIGHT
    if 120 < ang <= 180:
        return RIGHT_REAR
    if -60 <= ang < -15:
        return FORWARD_LEFT
    if -120 <= ang < -60:
        return LEFT
    return LEFT_REAR


def bearing_from_box(det, hfov_deg: float = DEFAULT_HFOV_DEG) -> float:
    """Box centre -> bearing in rad. servo()'s formula, on servo()'s tuple.

    `det` is the Grounder tuple (cx, cy, w, h, score, W, H, colour_fraction);
    only the first seven are unpacked, exactly as follow_vlm.servo() does. The
    eighth element is the HSV colour agreement, which has already gated this box
    upstream — the adjective is enforced before the phrase is built, which is
    also why the {object} slot of the prompt being inert costs nothing.

    The two lines are duplicated from servo() rather than imported, to keep this
    module out of follow_vlm's import graph (follow_vlm will import this one, and
    a cycle at import time is not worth the two lines).
    tests/test_vla_bridge.py::test_bearing_matches_servo_exactly pins them
    together over a sweep of cx and hfov, so a change to one fails on the other.
    """
    cx, _cy, _bw, _bh, _s, W, _H = det[:7]
    off = (cx - W / 2) / (W / 2)                 # -1 left .. +1 right
    return math.radians(hfov_deg / 2.0) * off


class VLABridge:
    """OWL-ViT detection -> AerialVLA prompt. No coordinates in any signature.

    The methods take a detection and nothing else. There is no pose argument, no
    goal argument and no target argument anywhere in this class — the box is the
    only input, so the compass phrase cannot be derived from ground truth even by
    accident. That is checked by
    tests/test_vla_bridge.py::test_module_never_references_a_target_coordinate,
    which fails on the name as well as the value.

    `obj_desc` fills the prompt's {object} slot. It is measured inert (correct vs
    wrong colour word give indistinguishable actions) and is carried only so the
    prompt keeps the shape the adapter was trained on. It stays a public
    attribute because demo/vla_server.py rewrites it live from its command file.
    """

    def __init__(self, obj_desc: str, hfov_deg: float = DEFAULT_HFOV_DEG,
                 bearing_gain: float = 1.0):
        """
        hfov_deg: the camera's real horizontal FOV, 90 for FrontCamera. Fixes the
        bearing scale and therefore which phrases are reachable at all.

        bearing_gain: multiplies the measured bearing before the band lookup.
        1.0 is the honest mapping and restricts the bridge to three phrases; 2.0
        lets a frame-edge box reach "to your right " and roughly triples the
        model's yaw response, at the cost of a prompt that overstates the
        bearing. A run that uses anything but 1.0 must report it.
        """
        if hfov_deg <= 0:
            raise ValueError("hfov_deg must be positive")
        if bearing_gain <= 0:
            raise ValueError("bearing_gain must be positive")
        self.obj_desc = obj_desc
        self.hfov_deg = float(hfov_deg)
        self.bearing_gain = float(bearing_gain)
        self.n_prompts = 0
        self.n_no_detection = 0
        self.phrase_counts = {p: 0 for p in PHRASE_ORDER}

    def _scaled_deg(self, bearing_deg: float) -> float:
        """Apply the gain, then the MAX_PHRASE_BEARING_DEG clamp.

        The clamp is the whole reason this is a method rather than a bare
        multiply at three call sites. Without it, gain 5 at hfov 90 sends a box
        on the LEFT edge to -225 deg, which wraps to +135 and comes out as
        "to your right rear " -- the steering inverts, silently, and the model is
        told the target is behind the aircraft when the forward camera can see
        it. Measured before the fix: cx=0 -> 'to your right rear ',
        cx=40 -> 'to your left rear ', cx=399 -> 'to your left rear '.

        Every producer of a phrase bearing must go through here, so the
        monotonicity guarantee holds for every gain rather than only small ones.
        """
        scaled = bearing_deg * self.bearing_gain
        return max(-MAX_PHRASE_BEARING_DEG, min(MAX_PHRASE_BEARING_DEG, scaled))

    @property
    def max_bearing_deg(self) -> float:
        """Largest bearing a real box can produce, after gain and clamp.

        A box centre cannot leave the image, so |off| <= 1 and the bearing is
        bounded by hfov/2 times the gain -- and then by the clamp, which is what
        keeps the two rear phrases (the two the ablation never measured)
        unreachable from any real box at any gain.
        """
        return min(MAX_PHRASE_BEARING_DEG,
                   self.hfov_deg / 2.0 * self.bearing_gain)

    def reachable_phrases(self) -> tuple[str, ...]:
        """The phrases this configuration can actually emit, left to right.

        Computed from the band table rather than hardcoded, so it stays correct
        for a different FOV or gain. At hfov 90 and gain 1 the answer is three
        phrases: forward-left, straight ahead, forward-right.
        """
        lim = self.max_bearing_deg
        out = [p for p in PHRASE_ORDER
               if any(bearing_to_phrase(math.radians(a)) == p
                      for a in _band_probe_angles(lim))]
        return tuple(out)

    def deadband_px(self, img_w: int) -> float:
        """Pixel half-width of the straight-ahead band: no command change inside.

        At hfov 90 and W = 400 this is 66.7 px, so the target sweeps 133 px —
        a third of the frame — with the commanded action bit-identical. servo()
        would have moved the yaw command continuously across the same span.

        Clamped at W/2: a narrow FOV or a gain below 1 can put the whole frame
        inside the band, and the honest report of that is "the entire image",
        not a pixel count wider than the image.
        """
        half = self.max_bearing_deg
        return min(img_w / 2.0, img_w / 2.0 * (15.0 / half))

    def phrase_for(self, det) -> str | None:
        """Detection -> compass phrase, or None when there is no detection.

        Returning None rather than EMPTY_PHRASE is the safety decision argued in
        the module docstring: an empty {direction} slot costs LAND on 11 of 18
        real frames, so the model must not be asked at all. The assertion is not
        decoration — it is the guarantee that nothing outside the training
        vocabulary can ever reach the prompt.
        """
        if det is None:
            self.n_no_detection += 1
            return None
        raw_deg = math.degrees(bearing_from_box(det, self.hfov_deg))
        phrase = bearing_to_phrase(math.radians(self._scaled_deg(raw_deg)))
        assert phrase in DIRECTION_VOCABULARY, f"off-vocabulary phrase {phrase!r}"
        self.phrase_counts[phrase] += 1
        return phrase

    def prompt_for(self, det) -> str | None:
        """Detection -> the full AerialVLA prompt string, or NO_DETECTION_PROMPT.

        The returned string is whatever build_prompt() makes of it, so it is
        identical to what the coordinate path produced for the same phrase — the
        only difference between the two experiments is where the phrase came
        from, which is the whole point of the comparison.
        """
        phrase = self.phrase_for(det)
        if phrase is None:
            return NO_DETECTION_PROMPT
        self.n_prompts += 1
        return build_prompt(phrase, self.obj_desc)

    def record_for(self, det) -> dict:
        """One JSONL row: what was seen, what phrase it became, what was asked.

        Logged per inference, not per tick — at 0.37 Hz a per-tick copy would be
        27 identical rows. `prompt_sha8` matches AerialVLABackend's own hash so a
        flight log and an inference log can be joined on it, and `bearing_deg` is
        kept alongside the phrase because the quantisation error (bearing minus
        band centre) is the thing this design has to be judged on.
        """
        if det is None:
            self.n_no_detection += 1
            return {"seen": False, "phrase": None, "prompt": NO_DETECTION_PROMPT,
                    "prompt_sha8": None, "bearing_deg": None, "gain": self.bearing_gain,
                    "reason": "no detection: inference skipped, VLA action zeroed"}
        raw_deg = math.degrees(bearing_from_box(det, self.hfov_deg))
        phrase = bearing_to_phrase(math.radians(self._scaled_deg(raw_deg)))
        assert phrase in DIRECTION_VOCABULARY, f"off-vocabulary phrase {phrase!r}"
        self.phrase_counts[phrase] += 1
        self.n_prompts += 1
        prompt = build_prompt(phrase, self.obj_desc)
        lo, hi = PHRASE_BANDS_DEG[phrase]
        return {"seen": True, "phrase": phrase, "prompt": prompt,
                "prompt_sha8": hashlib.sha256(prompt.encode("utf-8")).hexdigest()[:8],
                "bearing_deg": round(raw_deg, 2),
                "phrase_bearing_deg": round(self._scaled_deg(raw_deg), 2),
                "band_deg": [lo, hi], "gain": self.bearing_gain,
                "expected_yaw_rad_s": PHRASE_YAW_RAD_S[phrase],
                "cx": round(float(det[0]), 1), "img_w": int(det[5])}

    def stats(self) -> dict:
        """Post-flight accounting, in the shape Grounder.latest() uses.

        `phrase_counts` is the honest measure of how much the phrase channel
        actually carried: a flight that spent every inference in
        "straight ahead " gave the model one constant, and its trajectory says
        nothing about perception driving anything.
        """
        total = self.n_prompts + self.n_no_detection
        return {"n_prompts": self.n_prompts, "n_no_detection": self.n_no_detection,
                "no_detection_frac": round(self.n_no_detection / max(1, total), 3),
                "phrase_counts": dict(self.phrase_counts),
                "distinct_phrases": sum(1 for v in self.phrase_counts.values() if v),
                "hfov_deg": self.hfov_deg, "bearing_gain": self.bearing_gain}


def _band_probe_angles(limit_deg: float):
    """Angles to test for reachability: every band edge inside +-limit, plus it.

    Sampling on a fixed grid missed narrow bands when the gain was large; probing
    the edges themselves cannot.
    """
    edges = sorted({0.0, limit_deg, -limit_deg}
                   | {e for lo_hi in PHRASE_BANDS_DEG.values() for e in lo_hi
                      if abs(e) < limit_deg})
    out = list(edges)
    for a, b in zip(edges[:-1], edges[1:]):
        out.append((a + b) / 2.0)
    return out


def main() -> int:
    """Print what the module can emit. No sim, no GPU, no model — pure tables."""
    br = VLABridge("an orange car")
    print(f"vocabulary ({len(DIRECTION_VOCABULARY)} phrases, from "
          f"semantic_direction()):")
    for p in PHRASE_ORDER:
        lo, hi = PHRASE_BANDS_DEG[p]
        y = PHRASE_YAW_RAD_S[p]
        print(f"  [{lo:+7.1f},{hi:+7.1f}] deg  {p!r:22} "
              f"measured yaw {'unmeasured' if y is None else f'{y:+.3f} rad/s'}")
    print(f"\nFrontCamera hfov {br.hfov_deg:.0f} deg, gain {br.bearing_gain:.1f} "
          f"-> bearing bounded by +-{br.max_bearing_deg:.1f} deg")
    print(f"reachable: {[p.strip() for p in br.reachable_phrases()]}")
    print(f"deadband at W=400: +-{br.deadband_px(400):.1f} px "
          f"({2 * br.deadband_px(400) / 400:.0%} of the frame, no command change)")
    for cx in (0, 100, 166, 200, 234, 300, 399):
        det = (float(cx), 112.0, 40.0, 20.0, 0.05, 400, 225, 0.4)
        rec = br.record_for(det)
        print(f"  cx={cx:3d} -> {rec['bearing_deg']:+6.1f} deg -> "
              f"{rec['phrase']!r}")
    print(f"no detection -> prompt {br.prompt_for(None)!r} "
          f"(inference skipped by design)")
    print(f"\nprompt example:\n  {br.record_for(det)['prompt']!r}")
    print(f"stats: {br.stats()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
