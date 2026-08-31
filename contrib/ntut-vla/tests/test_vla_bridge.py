"""
demo/vla_bridge.py — vocabulary conformance, monotonicity, and the absent leak.

Run either way:
    pytest tests/test_vla_bridge.py -v
    python tests/test_vla_bridge.py

Three things are actually at stake here and each has a test that would fail if it
broke silently:

  1. The phrase strings must be the ones the LoRA was trained on. A paraphrase
     costs everything — the adapter's response to an unseen phrase is undefined,
     and the whole reason for using the {direction} slot is its measured
     monotonicity. So the vocabulary is not compared against a hardcoded list in
     this file; it is compared against what `aerialvla_demo.semantic_direction()`
     actually emits, swept over the full circle.

  2. The bearing must be servo()'s bearing. If the two drift, the bridge and the
     hand-written follower stop being comparable and the experiment loses its
     control condition.

  3. No target coordinate may reach the module — not as a value, not as an
     argument name, not as an import. That is what separates this from every
     earlier "semantic" run, all of which were confounded by exactly that leak
     (docs/FINDING-what-drives-aerialvla.md, Problem 1).
"""
import ast
import io
import math
import sys
import tokenize
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import vla_bridge                                                    # noqa: E402
from aerialvla_demo import build_prompt, semantic_direction          # noqa: E402
from follow_vlm import servo                                         # noqa: E402
from vla_bridge import (DIRECTION_VOCABULARY, EMPTY_PHRASE,          # noqa: E402
                        NO_DETECTION_PROMPT, PHRASE_BANDS_DEG,
                        PHRASE_ORDER, PHRASE_YAW_RAD_S, VLABridge,
                        bearing_from_box, bearing_to_phrase)

MODULE_PATH = ROOT / "demo" / "vla_bridge.py"
W, H = 400, 225                        # FrontCamera, measured


def _det(cx: float, img_w: int = W, w: float = 40.0):
    """A Grounder detection tuple: (cx, cy, w, h, score, W, H, colour)."""
    return (float(cx), H / 2.0, w, 20.0, 0.05, img_w, H, 0.4)


def _sweep_deg(step: float = 0.25):
    a = -180.0
    while a < 180.0:
        yield a
        a += step


def _ground_phrase(a_deg: float, yaw: float = 0.0, r: float = 10.0):
    """What semantic_direction() answers for a target `a_deg` off the nose.

    Both sides are handed the SAME floats — semantic_direction computes
    `degrees(atan2(dy, dx) - yaw)` and the bearing passed to bearing_to_phrase is
    `atan2(dy, dx) - yaw` — so an exact string comparison is meaningful even at
    the band edges, where a re-derived angle would land on the other side of a
    boundary by one ulp and prove nothing.
    """
    dx = r * math.cos(math.radians(a_deg + math.degrees(yaw)))
    dy = r * math.sin(math.radians(a_deg + math.degrees(yaw)))
    return (semantic_direction((0.0, 0.0), yaw, (dx, dy)),
            math.atan2(dy, dx) - yaw)


# ------------------------------------------------------- vocabulary conformance

def test_vocabulary_is_exactly_what_semantic_direction_emits():
    """The set the adapter was trained on, discovered rather than declared.

    If someone edits semantic_direction() — adds a band, drops a trailing space,
    rewords a phrase — this fails, which is the point. A phrase the LoRA never
    saw is worth nothing, and a bridge that emits one looks like it is working.
    """
    seen = set()
    for a in _sweep_deg(0.25):
        gt, _ = _ground_phrase(a)
        assert gt != EMPTY_PHRASE, f"unexpected empty hint at {a} deg"
        seen.add(gt)
    assert seen == set(DIRECTION_VOCABULARY), (
        f"vocabulary drift: semantic_direction emits {sorted(seen)}, "
        f"the bridge declares {sorted(DIRECTION_VOCABULARY)}")
    assert len(DIRECTION_VOCABULARY) == 7
    # the trailing space is part of the trained string, not incidental
    assert all(p.endswith(" ") for p in DIRECTION_VOCABULARY)


def test_bearing_to_phrase_is_the_faithful_inverse_of_semantic_direction():
    """Every angle on the circle: the pure function and the coordinate function
    must agree on the phrase, including at all six band boundaries and the
    +-180 seam."""
    for a in _sweep_deg(0.25):
        gt, bearing = _ground_phrase(a)
        got = bearing_to_phrase(bearing)
        assert got == gt, f"{a:+.2f} deg: bridge {got!r} != trained {gt!r}"


def test_the_inverse_holds_under_a_nonzero_heading():
    """semantic_direction subtracts the aircraft yaw before bucketing. The bridge
    has no yaw input at all — the bearing is already body-frame, because the
    camera is bolted to the body — so this checks the two agree for the same
    body-frame angle regardless of where the aircraft is pointing."""
    for yaw in (-2.5, -0.7, 0.0, 0.9, 3.0):
        for a in _sweep_deg(2.0):
            gt, bearing = _ground_phrase(a, yaw=yaw)
            assert bearing_to_phrase(bearing) == gt, f"yaw {yaw} at {a} deg"


def test_every_phrase_the_bridge_can_emit_is_in_the_vocabulary():
    """The requirement stated as a sweep: every pixel column, several FOVs and
    several gains, including the gains that reach outside the FOV bands."""
    for hfov in (60.0, 90.0, 120.0):
        for gain in (0.5, 1.0, 2.0, 4.0, 9.0):
            br = VLABridge("an orange car", hfov_deg=hfov, bearing_gain=gain)
            for cx in range(0, W + 1):
                p = br.phrase_for(_det(cx))
                assert p in DIRECTION_VOCABULARY, (
                    f"hfov {hfov} gain {gain} cx {cx}: {p!r} not trained")
                assert p != EMPTY_PHRASE


def test_the_bands_and_tables_cover_the_vocabulary_exactly():
    """PHRASE_BANDS_DEG / PHRASE_ORDER / PHRASE_YAW_RAD_S are three parallel
    tables; a phrase missing from one of them is a silent hole. The module
    asserts this at import, so this test mostly documents that it does."""
    assert set(PHRASE_BANDS_DEG) == DIRECTION_VOCABULARY
    assert set(PHRASE_ORDER) == DIRECTION_VOCABULARY
    assert set(PHRASE_YAW_RAD_S) == DIRECTION_VOCABULARY
    assert len(PHRASE_ORDER) == len(set(PHRASE_ORDER))
    # every band's interior maps back to its own phrase
    for phrase, (lo, hi) in PHRASE_BANDS_DEG.items():
        mid = (lo + hi) / 2.0
        assert bearing_to_phrase(math.radians(mid)) == phrase, f"{phrase!r} band"


# ------------------------------------------------------------------ boundaries

def test_boundary_angles_belong_to_the_more_central_band():
    """semantic_direction's if-chain claims [-15, +15] first, then widens, so
    every edge resolves inward. Worth pinning: rewriting the chain as ordered
    `lo < a <= hi` lookups moves -15 from straight-ahead to forward-left.

    Fed as radians, math.degrees(math.radians(15.0)) is 14.999999999999998 —
    the roundtrip error happens to land inside the central band at every edge,
    so the float path agrees with the intent instead of fighting it.
    """
    for edge, inner in ((15.0, vla_bridge.STRAIGHT_AHEAD),
                        (-15.0, vla_bridge.STRAIGHT_AHEAD),
                        (60.0, vla_bridge.FORWARD_RIGHT),
                        (-60.0, vla_bridge.FORWARD_LEFT),
                        (120.0, vla_bridge.RIGHT),
                        (-120.0, vla_bridge.LEFT)):
        assert bearing_to_phrase(math.radians(edge)) == inner, f"{edge} deg"
        gt, bearing = _ground_phrase(edge)
        assert bearing_to_phrase(bearing) == gt, f"{edge} deg vs trained"


def test_just_outside_a_boundary_is_the_outer_band():
    """The other half of the edge contract — the bands really do change, and at
    the stated angle rather than somewhere near it."""
    eps = 0.05
    cases = [(15 + eps, vla_bridge.FORWARD_RIGHT), (-15 - eps, vla_bridge.FORWARD_LEFT),
             (60 + eps, vla_bridge.RIGHT), (-60 - eps, vla_bridge.LEFT),
             (120 + eps, vla_bridge.RIGHT_REAR), (-120 - eps, vla_bridge.LEFT_REAR)]
    for a, expect in cases:
        assert bearing_to_phrase(math.radians(a)) == expect, f"{a} deg"


def test_the_seam_at_180_wraps_left_like_semantic_direction_does():
    """`(deg + 180) % 360 - 180` sends +180 to -180, so a target dead astern is
    "to your left rear ", not right rear. Inherited quirk, not a choice — but it
    has to be inherited exactly, and 3.5 rad (200 deg) must not fall off the
    table."""
    assert bearing_to_phrase(math.radians(180.0)) == vla_bridge.LEFT_REAR
    assert bearing_to_phrase(math.radians(-180.0)) == vla_bridge.LEFT_REAR
    assert bearing_to_phrase(math.radians(179.9)) == vla_bridge.RIGHT_REAR
    for a in (200.0, 360.0, 540.0, -200.0, -359.0, 1e4):
        assert bearing_to_phrase(math.radians(a)) in DIRECTION_VOCABULARY


# ------------------------------------------------------------- the servo pin

def test_bearing_matches_servo_exactly():
    """The bridge's two-line formula is a deliberate copy of servo()'s. Bitwise
    equality, not approximate: if follow_vlm's bearing changes, the bridge's must
    change with it or the two controllers are no longer the same experiment."""
    for hfov in (45.0, 60.0, 90.0, 120.0):
        for img_w in (400, 224, 640):
            for cx in range(0, img_w + 1, 7):
                det = _det(cx, img_w=img_w)
                _yaw, _fwd, bearing = servo(det, img_w, 1.2, 0.10, 4.0,
                                            hfov_deg=hfov)
                assert bearing_from_box(det, hfov) == bearing, (
                    f"hfov {hfov} W {img_w} cx {cx}: "
                    f"{bearing_from_box(det, hfov)} != {bearing}")


def test_bearing_geometry_is_sane():
    """Centre is zero, edges are +-hfov/2, sign is right-positive. This is the
    assumption the whole phrase mapping rests on."""
    assert bearing_from_box(_det(W / 2)) == 0.0
    assert bearing_from_box(_det(0)) == -math.radians(45.0)
    assert bearing_from_box(_det(W)) == math.radians(45.0)
    assert bearing_from_box(_det(W * 0.75)) > 0.0
    assert bearing_from_box(_det(W * 0.25)) < 0.0


# ------------------------------------------------------------- monotonicity

def test_more_left_box_is_never_a_more_right_phrase():
    """The property the {direction} slot is being used for. Walk the box centre
    across the frame; the phrase ordinal must never decrease."""
    idx = {p: i for i, p in enumerate(PHRASE_ORDER)}
    for gain in (0.5, 1.0, 2.0, 5.0):
        br = VLABridge("an orange car", bearing_gain=gain)
        prev = -1
        for cx in range(0, W + 1):
            i = idx[br.phrase_for(_det(cx))]
            assert i >= prev, f"gain {gain}: phrase went left at cx={cx}"
            prev = i


def test_the_extremes_of_the_frame_are_the_extremes_of_the_reachable_set():
    """Not just monotonic but actually using the range: leftmost column gives
    the leftmost reachable phrase and rightmost the rightmost."""
    for gain in (1.0, 2.0, 4.0):
        br = VLABridge("an orange car", bearing_gain=gain)
        reach = br.reachable_phrases()
        assert br.phrase_for(_det(0)) == reach[0]
        assert br.phrase_for(_det(W)) == reach[-1]
        assert br.phrase_for(_det(W / 2)) == vla_bridge.STRAIGHT_AHEAD


def test_measured_response_is_monotonic_across_the_measured_phrases():
    """The published ablation, restated as a test so the table cannot be edited
    into something that no longer supports the design. Only five of the seven
    phrases were measured; the two rear ones are None and are skipped rather
    than interpolated."""
    measured = [(p, PHRASE_YAW_RAD_S[p]) for p in PHRASE_ORDER
                if PHRASE_YAW_RAD_S[p] is not None]
    assert len(measured) == 5
    ys = [y for _p, y in measured]
    assert ys == sorted(ys), f"published yaw response is not monotonic: {measured}"
    assert ys[0] < 0 < ys[-1]


# --------------------------------------------------- FOV limits and quantisation

def test_only_three_phrases_are_reachable_from_a_real_box():
    """The honest limit of the whole idea: a box lives inside the 90 deg FOV, so
    the bearing cannot exceed +-45 deg and the four outer phrases — including
    both of the strongest-responding ones — can never be produced at unit gain."""
    br = VLABridge("an orange car")
    assert br.max_bearing_deg == 45.0
    assert br.reachable_phrases() == (vla_bridge.FORWARD_LEFT,
                                      vla_bridge.STRAIGHT_AHEAD,
                                      vla_bridge.FORWARD_RIGHT)
    emitted = {br.phrase_for(_det(cx)) for cx in range(0, W + 1)}
    assert emitted == set(br.reachable_phrases())
    assert vla_bridge.LEFT not in emitted and vla_bridge.RIGHT not in emitted
    # and the reachable band is the weak, asymmetric middle of the response curve
    ys = [PHRASE_YAW_RAD_S[p] for p in br.reachable_phrases()]
    assert max(abs(y) for y in ys) < abs(PHRASE_YAW_RAD_S[vla_bridge.LEFT])


def test_gain_is_the_only_way_to_reach_the_outer_phrases():
    """`bearing_gain` is a documented dishonesty knob: at 2.0 a frame-edge box
    reads as "to your right ", which buys yaw authority by overstating the
    bearing. Verified so that a run reporting gain 1.0 can be trusted not to
    have reached those phrases."""
    strong = VLABridge("an orange car", bearing_gain=2.0)
    emitted = {strong.phrase_for(_det(cx)) for cx in range(0, W + 1)}
    assert vla_bridge.LEFT in emitted and vla_bridge.RIGHT in emitted
    assert vla_bridge.LEFT_REAR not in emitted        # 2 x 45 = 90 deg, no rears
    assert strong.max_bearing_deg == 90.0


def test_deadband_is_a_third_of_the_frame():
    """The quantisation cost, in pixels. At hfov 90 the straight-ahead band is
    |bearing| <= 15 deg = |cx - W/2| <= W/6, so the target crosses 133 of 400
    columns with the commanded action bit-identical — where servo() would have
    moved the yaw command continuously across the same span."""
    br = VLABridge("an orange car")
    assert abs(br.deadband_px(W) - W / 6.0) < 1e-9
    same = [cx for cx in range(0, W + 1)
            if br.phrase_for(_det(cx)) == vla_bridge.STRAIGHT_AHEAD]
    assert len(same) >= W // 3
    # servo() at the edge of that band already commands real yaw, the phrase
    # path commands a constant: 1.2 * radians(15) = 0.314 rad/s vs -0.035
    _yaw_edge, _f, brg = servo(_det(W / 2 + W / 6), W, 1.2, 0.10, 4.0)
    assert abs(_yaw_edge) > 0.3
    assert abs(PHRASE_YAW_RAD_S[vla_bridge.STRAIGHT_AHEAD]) < 0.05
    # a gain narrows the deadband, and the clamp holds when it would exceed W/2
    assert VLABridge("c", bearing_gain=3.0).deadband_px(W) < br.deadband_px(W)
    assert VLABridge("c", bearing_gain=0.1).deadband_px(W) == W / 2.0


def test_straight_ahead_is_biased_not_neutral():
    """A dead-centred target still commands -0.035 rad/s, so the aircraft yaws
    left ~5.4 deg over the 2.7 s an action is held. Recorded as a test because it
    is the reason a static objective drifts under this bridge at all."""
    y = PHRASE_YAW_RAD_S[vla_bridge.STRAIGHT_AHEAD]
    assert y != 0.0
    assert abs(math.degrees(y) * 2.7) > 5.0


# ---------------------------------------------------------- the no-detection path

def test_no_detection_yields_no_prompt_and_never_the_empty_phrase():
    """The safety decision: no perception, no learned action. Emitting
    semantic_direction's "" instead would command LAND on 11 of 18 real frames,
    and emitting "straight ahead " would fly blind at ~3.8 m/s over an obstacle
    map that contains buildings only."""
    br = VLABridge("an orange car")
    assert br.phrase_for(None) is None
    assert br.prompt_for(None) is NO_DETECTION_PROMPT
    assert NO_DETECTION_PROMPT is None
    assert EMPTY_PHRASE not in DIRECTION_VOCABULARY
    rec = br.record_for(None)
    assert rec["seen"] is False
    assert rec["phrase"] is None and rec["prompt"] is None
    assert "zero" in rec["reason"]          # the caller must not latch the action
    assert br.n_prompts == 0 and br.n_no_detection == 3


def test_a_missing_detection_never_produces_a_prompt_string():
    """Belt and braces: nothing falsy-but-stringy sneaks through, because a
    prompt string is the one thing that would cause an inference to run."""
    br = VLABridge("an orange car")
    for _ in range(5):
        p = br.prompt_for(None)
        assert p is None and not isinstance(p, str)


# ---------------------------------------------------------------- the prompt

def test_prompt_is_the_shape_the_adapter_was_trained_on():
    """Byte-identical to what the coordinate path produced for the same phrase.
    That equality is what makes the two experiments comparable: the only
    difference is where the phrase came from."""
    br = VLABridge("an orange car")
    det = _det(W * 0.8)                                  # +27 deg -> forward-right
    prompt = br.prompt_for(det)
    assert prompt == ("<image>\nFly forward-right and find the target. "
                      "an orange car\nAction: ")
    assert prompt == build_prompt(vla_bridge.FORWARD_RIGHT, "an orange car")
    # and identical to the leaky path's output when that path picked the same band
    gt, _bearing = _ground_phrase(27.0)
    assert prompt == build_prompt(gt, "an orange car")
    assert prompt.startswith("<image>\n") and prompt.endswith("\nAction: ")


def test_object_slot_is_carried_through_and_stays_mutable():
    """The {object} slot is measured inert, but vla_server.py rewrites it live
    from its command file, so it has to be a plain attribute and it has to reach
    the prompt."""
    br = VLABridge("an orange car")
    assert "an orange car" in br.prompt_for(_det(W / 2))
    br.obj_desc = "a white building"
    assert "a white building" in br.prompt_for(_det(W / 2))
    empty = VLABridge("")
    assert empty.prompt_for(_det(W / 2)) == (
        "<image>\nFly straight ahead and find the target.\nAction: ")


def test_record_carries_the_quantisation_error():
    """The thing this design must be judged on is bearing minus band centre, so
    both have to be in the log, along with the hash that joins a flight log to an
    inference log."""
    br = VLABridge("an orange car")
    rec = br.record_for(_det(0))
    assert rec["seen"] is True
    assert rec["phrase"] == vla_bridge.FORWARD_LEFT
    assert rec["bearing_deg"] == -45.0
    assert rec["band_deg"] == [-60.0, -15.0]
    assert rec["expected_yaw_rad_s"] == PHRASE_YAW_RAD_S[vla_bridge.FORWARD_LEFT]
    assert len(rec["prompt_sha8"]) == 8
    assert rec["cx"] == 0.0 and rec["img_w"] == W
    import hashlib
    assert rec["prompt_sha8"] == hashlib.sha256(
        rec["prompt"].encode("utf-8")).hexdigest()[:8]


def test_stats_account_for_every_call():
    br = VLABridge("an orange car")
    for cx in (0, 200, 399):
        br.prompt_for(_det(cx))
    br.prompt_for(None)
    st = br.stats()
    assert st["n_prompts"] == 3 and st["n_no_detection"] == 1
    assert st["no_detection_frac"] == 0.25
    assert st["distinct_phrases"] == 3
    assert sum(st["phrase_counts"].values()) == 3
    assert st["bearing_gain"] == 1.0 and st["hfov_deg"] == 90.0


def test_bad_configuration_is_rejected_at_construction():
    for kwargs in ({"hfov_deg": 0.0}, {"hfov_deg": -90.0},
                   {"bearing_gain": 0.0}, {"bearing_gain": -1.0}):
        try:
            VLABridge("an orange car", **kwargs)
        except ValueError:
            continue
        raise AssertionError(f"accepted {kwargs}")


# ------------------------------------------------- the leak, checked structurally

FORBIDDEN_NAMES = {
    "target", "target_xy", "target_x", "target_y", "targetxy",
    "tgt", "tgt_x", "tgt_y", "goal", "goal_x", "goal_y", "goal_xy",
    "truth", "ground_truth", "gt", "gt_x", "gt_y",
    "car", "car_pos", "pos", "world_pos",
    "semantic_direction", "get_object_pose", "get_ground_truth_kinematics",
}
FORBIDDEN_FRAGMENTS = ("target", "tgt_", "ground_truth", "semantic_direction")


def _code_names(path: Path):
    """Every NAME token in the file, comments and string literals discarded.

    Discarding strings is required, not a loophole: the module docstring argues
    at length about target coordinates, and build_prompt's trained sentence is
    literally "and find the target." — neither is a coordinate. What matters is
    whether an identifier exists that could carry one.
    """
    src = path.read_text(encoding="utf-8")
    names = []
    for tok in tokenize.generate_tokens(io.StringIO(src).readline):
        if tok.type == tokenize.NAME:
            names.append(tok.string)
    return names


def test_module_never_references_a_target_coordinate():
    """The grep-style check that makes the leak structurally impossible.

    Every earlier "semantic" flight was confounded by a coordinate-derived
    bearing reaching the prompt. This module's claim is that it cannot happen
    here, so the claim is checked against the source rather than trusted.
    """
    names = _code_names(MODULE_PATH)
    lowered = [n.lower() for n in names]
    hits = sorted({n for n in lowered if n in FORBIDDEN_NAMES})
    assert not hits, f"coordinate-flavoured identifiers in vla_bridge.py: {hits}"
    frag = sorted({n for n in lowered
                   for f in FORBIDDEN_FRAGMENTS if f in n})
    assert not frag, f"coordinate-flavoured identifiers in vla_bridge.py: {frag}"


def test_no_function_in_the_module_takes_a_coordinate_argument():
    """The signature half of requirement 2: not one callable in the module — and
    that includes every method of the bridge class — accepts a target position
    under any name."""
    tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"))
    offenders = []
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        a = node.args
        for arg in (list(a.posonlyargs) + list(a.args) + list(a.kwonlyargs)
                    + [x for x in (a.vararg, a.kwarg) if x]):
            low = arg.arg.lower()
            if low in FORBIDDEN_NAMES or any(f in low for f in FORBIDDEN_FRAGMENTS):
                offenders.append(f"{node.name}({arg.arg})")
    assert not offenders, f"coordinate arguments: {offenders}"


def test_module_imports_nothing_that_could_supply_coordinates():
    """The strongest form of the claim: the module has no access to a coordinate
    source at all. It imports one name from aerialvla_demo — build_prompt, two
    strings in and one string out — and explicitly not semantic_direction, which
    is the coordinate path living in the same file."""
    tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"))
    allowed_modules = {"__future__", "hashlib", "math", "sys", "pathlib",
                       "aerialvla_demo"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for al in node.names:
                assert al.name.split(".")[0] in allowed_modules, f"import {al.name}"
        elif isinstance(node, ast.ImportFrom):
            root = (node.module or "").split(".")[0]
            assert root in allowed_modules, f"from {node.module} import ..."
            if root == "aerialvla_demo":
                got = {al.name for al in node.names}
                assert got == {"build_prompt"}, f"imported {sorted(got)}"


def test_the_public_methods_take_a_detection_and_nothing_else():
    """Requirement 2 as the caller sees it: the detection is the only input, so
    the compass phrase cannot be derived from anything else even by accident."""
    import inspect
    for name in ("phrase_for", "prompt_for", "record_for"):
        params = list(inspect.signature(getattr(VLABridge, name)).parameters)
        assert params == ["self", "det"], f"{name}{params}"
    assert list(inspect.signature(bearing_from_box).parameters) == ["det", "hfov_deg"]
    assert list(inspect.signature(bearing_to_phrase).parameters) == ["bearing_rad"]
    assert list(inspect.signature(VLABridge.__init__).parameters) == [
        "self", "obj_desc", "hfov_deg", "bearing_gain"]


# --------------------------------------------------------------------- runner

if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}  {e}")
        except Exception as e:                       # noqa: BLE001
            # Any crash is a fail, not the end of the run. Without this arm
            # an ImportError or a numpy error in one test propagates out of
            # the loop, every remaining test is silently skipped, and the
            # summary line never prints - so the file looks like it ran
            # clean when most of it never executed. pytest runs them all, so
            # the two invocation modes disagreed about coverage.
            failed += 1
            print(f"ERROR {fn.__name__}  {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
