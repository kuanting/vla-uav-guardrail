"""Depth-based range and instance persistence — the two gaps the orbit exposed.

Run either way:
    pytest tests/test_range_and_lock.py -v
    python tests/test_range_and_lock.py

RANGE. The radial servo used apparent box width, a fine proxy for a car (same
width from any angle) and a bad one for a 50 x 50 m block, where it swings by
root-2 between face-on and corner-on. Circling made the servo command reverse
from the aspect change alone and the orbit radius spiralled 37.7 m -> 178 m.

PERSISTENCE. "a building" names a KIND and the map has nine city blocks; "a car"
names a kind and the traffic scene has four. Box-centre discontinuities over
80 px occurred 24, 5 and 8 times across the three orbit flights, so the aircraft
was chasing whichever instance was most salient rather than the one it started
on. For the car, colour supplied instance persistence by accident. Nothing does
for a building.
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from follow_vlm import (PresenceMonitor, TargetLock,          # noqa: E402
                        appearance, appearance_similarity,
                        implied_width_m, presence_verdict, range_from_depth,
                        search_sweep_rate, colour_match,
                        object_mask_from_depth)

W, H = 400, 225


def _det(cx, cy=112.0, bw=40.0, bh=30.0, score=0.05, colour=0.5):
    return (cx, cy, bw, bh, score, W, H, colour)


def _depth(bg=200.0, box=None, val=25.0):
    d = np.full((H, W), bg, dtype=np.float32)
    if box:
        x0, y0, x1, y1 = box
        d[y0:y1, x0:x1] = val
    return d


# ----------------------------------------------------------------- range

def test_range_reads_the_object_not_the_background():
    """The whole point of shrinking the sampling window."""
    d = _depth(bg=300.0, box=(180, 97, 220, 127), val=25.0)
    r = range_from_depth(d, _det(200.0, 112.0, 40.0, 30.0))
    assert r is not None and abs(r - 25.0) < 0.5, r


def test_a_few_background_pixels_do_not_move_the_answer():
    """Median, not mean: one sky pixel at 5 km would wreck a mean."""
    d = _depth(bg=300.0, box=(180, 97, 220, 127), val=25.0)
    d[112, 195] = 5000.0
    d[110, 202] = 4000.0
    r = range_from_depth(d, _det(200.0))
    assert r is not None and abs(r - 25.0) < 0.5, r


def test_non_finite_and_zero_depths_are_rejected():
    """Invalid samples must be dropped, and the OBJECT still read.

    `r is None or math.isfinite(r)` only rejected NaN and infinity, so it also
    accepted the background. Regress the sampling window back to the whole box
    and this fixture returns ~300 - the sky - which is finite, and the test
    passed while the servo flew on it. The value is what matters, so assert the
    value; the sibling tests above already do.

    Both invalid kinds are covered: NaN over part of the object, and 0.0, which
    the name always promised and the fixture never actually contained.
    """
    d = _depth(bg=300.0, box=(180, 97, 220, 127), val=25.0)
    d[105:115, 190:210] = np.nan
    r = range_from_depth(d, _det(200.0))
    assert r is not None and abs(r - 25.0) < 0.5, r

    z = _depth(bg=300.0, box=(180, 97, 220, 127), val=25.0)
    z[105:115, 190:210] = 0.0
    rz = range_from_depth(z, _det(200.0))
    assert rz is not None and abs(rz - 25.0) < 0.5, rz


def test_an_all_invalid_window_returns_none_rather_than_a_number():
    """Returning a plausible-looking number from garbage is worse than nothing:
    the caller would servo on it."""
    d = np.full((H, W), np.nan, dtype=np.float32)
    assert range_from_depth(d, _det(200.0)) is None


def test_no_depth_or_no_detection_is_not_an_error():
    """A missing stream must degrade to the width servo, not crash the flight."""
    assert range_from_depth(None, _det(200.0)) is None
    assert range_from_depth(_depth(), None) is None


def test_a_box_off_the_edge_of_the_frame_is_handled():
    """Clipped at the frame edge, it must still read the OBJECT.

    `r is None or r > 0` accepted the 300 m background as readily as the 18 m
    object, so this proved only that the call did not crash.
    """
    d = _depth(bg=300.0, box=(0, 97, 20, 127), val=18.0)
    r = range_from_depth(d, _det(2.0, 112.0, 40.0, 30.0))
    assert r is not None and abs(r - 18.0) < 0.5, r


def test_range_is_independent_of_apparent_width():
    """The property the width servo did not have. Same object, same distance,
    two very different box widths - the range must not move."""
    d = _depth(bg=300.0, box=(150, 92, 250, 132), val=40.0)
    narrow = range_from_depth(d, _det(200.0, 112.0, 30.0, 30.0))
    wide = range_from_depth(d, _det(200.0, 112.0, 90.0, 36.0))
    assert narrow is not None and wide is not None
    assert abs(narrow - wide) < 0.5, (narrow, wide)


# ------------------------------------------------------------------ lock

def test_the_first_detection_establishes_the_lock():
    lk = TargetLock()
    chosen, switched = lk.select([_det(100.0), _det(300.0)], W, 0.0, 1.0)
    assert chosen[0] == 100.0        # best-ranked wins when nothing is held
    assert not switched


def test_it_keeps_the_held_instance_over_a_better_scoring_rival():
    """The core behaviour. A distractor that outscores the target must not steal
    the lock just by scoring higher."""
    lk = TargetLock()
    lk.select([_det(100.0, score=0.05)], W, 0.0, 1.0)
    rival = _det(300.0, score=0.50)          # ten times the score, far away
    held = _det(104.0, score=0.03)
    chosen, switched = lk.select([rival, held], W, 0.0, 1.1)
    assert chosen[0] == 104.0, "the lock followed the score instead of the object"
    assert not switched


def test_a_yaw_turn_does_not_break_the_lock():
    """Objects slide across the frame when the nose turns. Without predicting
    that, the gate would reject the true target exactly when the aircraft is
    tracking hardest."""
    lk = TargetLock(hfov_deg=90.0)
    lk.select([_det(200.0)], W, 0.0, 1.0)
    yaw = math.radians(20.0)                 # turn right 20 deg
    px = 200.0 - yaw * (W / math.radians(90.0))
    chosen, switched = lk.select([_det(px), _det(390.0)], W, yaw, 1.1)
    assert abs(chosen[0] - px) < 1.0, chosen[0]
    assert not switched, "a normal turn was reported as a target switch"


def test_a_real_jump_is_reported_as_a_switch_not_hidden():
    """When the held instance genuinely disappears, re-acquiring is correct — but
    it must be visible, because that is the moment the mission changes target."""
    lk = TargetLock()
    lk.select([_det(80.0)], W, 0.0, 1.0)
    chosen, switched = lk.select([_det(350.0)], W, 0.0, 1.1)
    assert chosen[0] == 350.0
    assert switched, "target changed silently"
    assert lk.stats()["switched"] >= 1


def test_a_stale_lock_is_dropped_rather_than_trusted_forever():
    """After hold_s with no match the lock must let go, or one long occlusion
    would pin the aircraft to an instance it can no longer see."""
    lk = TargetLock(hold_s=2.0)
    lk.select([_det(80.0)], W, 0.0, 1.0)
    chosen, switched = lk.select([_det(350.0)], W, 0.0, 1.0 + 5.0)
    assert chosen[0] == 350.0, "the stale lock still constrained the choice"
    # And it is still reported. Re-acquiring on a different object after a long
    # loss IS a target change - the mission is now following something else, and
    # a reader of the log needs to know that even though the re-acquire itself
    # was the correct behaviour.
    assert switched, "target changed after a stale lock without being reported"


def test_no_candidates_leaves_the_lock_untouched():
    """A miss must not clear the lock, or one dropped frame would re-acquire on
    whatever appears next."""
    lk = TargetLock()
    lk.select([_det(120.0)], W, 0.0, 1.0)
    chosen, switched = lk.select([], W, 0.0, 1.1)
    assert chosen is None and not switched
    assert lk.cx == 120.0


def test_the_gate_scales_with_image_width():
    """0.28 of 400 px is 112 px; a 150 px jump must fail and a 90 px one pass."""
    for jump, want_switch in ((150.0, True), (90.0, False)):
        lk = TargetLock(gate_frac=0.28)
        lk.select([_det(200.0)], W, 0.0, 1.0)
        _, switched = lk.select([_det(200.0 + jump)], W, 0.0, 1.1)
        assert switched is want_switch, (jump, switched)


def test_lock_is_opt_in_and_absent_by_default():
    """Grounder(lock=None) must behave exactly as before this feature existed."""
    import follow_vlm
    import inspect
    sig = inspect.signature(follow_vlm.Grounder.__init__)
    assert sig.parameters["lock"].default is None


# -------------------------------------------------------------- presence

def test_a_car_sized_box_at_a_plausible_range_reads_present():
    """40 px at 30 m in a 400 px / 90 deg frame implies about 6 m — a car."""
    v, why = presence_verdict(_det(200.0, bw=40.0, colour=0.5), 30.0,
                              "a white car", 0.10)
    assert v == "PRESENT", (v, why)


def test_no_detection_reads_absent():
    v, why = presence_verdict(None, 25.0, "a white car", 0.10)
    assert v == "ABSENT" and "no detection" in why


def test_a_box_filling_the_frame_is_a_wall_not_an_object():
    """The orbit flights returned a MEDIAN box of 395 px in a 400 px frame for
    "a building", and every downstream stage treated it as a target."""
    v, why = presence_verdict(_det(200.0, bw=395.0, colour=0.9), 40.0,
                              "a building", 0.10)
    assert v == "ABSENT" and "frame" in why, (v, why)


def test_the_wrong_colour_reads_absent():
    v, why = presence_verdict(_det(200.0, bw=40.0, colour=0.02), 30.0,
                              "a white car", 0.10)
    assert v == "ABSENT" and "colour" in why, (v, why)


def test_a_car_that_would_have_to_be_forty_metres_wide_reads_absent():
    """The check that only depth makes possible. Same box, same colour, same
    score — only the range says this cannot be a car."""
    near = presence_verdict(_det(200.0, bw=84.0, colour=0.5), 12.0, "a car", 0.10)
    far = presence_verdict(_det(200.0, bw=84.0, colour=0.5), 60.0, "a car", 0.10)
    assert near[0] == "PRESENT", near        # 84 px at 12 m is 4.0 m: a car
    assert far[0] == "ABSENT" and "wide" in far[1], far   # at 60 m it is 19.8 m


def test_without_range_it_says_unsure_rather_than_present():
    """Honest about not knowing. Defaulting to PRESENT is how the old system
    scored 1.000 subject retention on a flight 200 m from any traffic light."""
    v, why = presence_verdict(_det(200.0, bw=40.0, colour=0.5), None,
                              "a white car", 0.10)
    assert v == "UNSURE", (v, why)


def test_implied_width_scales_with_range_and_not_with_score():
    a = implied_width_m(_det(200.0, bw=40.0, score=0.9), 30.0)
    b = implied_width_m(_det(200.0, bw=40.0, score=0.01), 30.0)
    c = implied_width_m(_det(200.0, bw=40.0), 60.0)
    assert abs(a - b) < 1e-9, "score changed a geometric quantity"
    assert abs(c - 2 * a) < 0.2, (a, c)


def test_an_unknown_noun_skips_the_size_check_rather_than_guessing():
    v, _ = presence_verdict(_det(200.0, bw=40.0, colour=0.5), 30.0,
                            "a white widget", 0.10)
    assert v == "PRESENT"


# ----------------------------------------------------------- appearance

def _img(rgb, w=200, h=120):
    return np.tile(np.array(rgb, dtype=np.uint8), (h, w, 1))


BOX = (40.0, 20.0, 160.0, 100.0)


def test_the_same_object_looks_the_same():
    a = appearance(_img((200, 30, 30)), BOX)
    b = appearance(_img((200, 30, 30)), BOX)
    assert appearance_similarity(a, b) > 0.99


def test_a_different_colour_looks_different():
    red = appearance(_img((200, 30, 30)), BOX)
    blue = appearance(_img((30, 30, 200)), BOX)
    assert appearance_similarity(red, blue) < 0.2, appearance_similarity(red, blue)


def test_a_shade_change_is_tolerated_more_than_a_hue_change():
    """Lighting must not read as a different object; a different object must."""
    base = appearance(_img((200, 40, 40)), BOX)
    dim = appearance(_img((150, 30, 30)), BOX)      # same hue, darker
    other = appearance(_img((40, 200, 40)), BOX)    # different hue
    assert appearance_similarity(base, dim) > appearance_similarity(base, other)


def test_the_descriptor_is_normalised_and_small():
    a = appearance(_img((200, 30, 30)), BOX)
    assert a is not None and a.size == 32
    assert abs(float(a.sum()) - 1.0) < 1e-6


def test_a_degenerate_box_yields_nothing_rather_than_noise():
    assert appearance(_img((200, 30, 30)), (10.0, 10.0, 12.0, 12.0)) is None
    assert appearance(None, BOX) is None


def test_missing_descriptors_are_no_opinion_not_a_mismatch():
    """A frame without an image must not be read as 'the target changed'."""
    a = appearance(_img((200, 30, 30)), BOX)
    assert appearance_similarity(a, None) == 1.0
    assert appearance_similarity(None, None) == 1.0


def test_the_monitor_rejects_a_look_alike_of_the_wrong_colour():
    """The ceiling this was built to break: geometry says 'consistent with a
    car', appearance says 'a DIFFERENT car-like thing'."""
    # appear_min defaults to 0 (disabled — it does not help at flight
    # resolution, see the docstring), so this test opts in explicitly.
    m = PresenceMonitor("a white car", colour_min=0.0, window=999, appear_min=0.55)
    red = appearance(_img((200, 30, 30)), BOX)
    blue = appearance(_img((30, 30, 200)), BOX)
    v1, _ = m.update(_det(200.0, bw=84.0, colour=0.5), 12.0, red)
    assert v1 == "PRESENT", v1
    v2, why = m.update(_det(200.0, bw=84.0, colour=0.5), 12.0, blue)
    assert v2 == "ABSENT" and "different" in why, (v2, why)


def test_the_monitor_tolerates_gradual_drift():
    """Slow lighting change must not trip it, or every flight would end ABSENT."""
    m = PresenceMonitor("a white car", colour_min=0.0, window=999, appear_min=0.55)
    for k in range(30):
        shade = 200 - k          # gently darkening, same hue
        app = appearance(_img((shade, 30, 30)), BOX)
        v, why = m.update(_det(200.0, bw=84.0, colour=0.5), 12.0, app)
        assert v == "PRESENT", f"tripped at step {k}: {why}"


def test_appearance_is_opt_out():
    m = PresenceMonitor("a white car", colour_min=0.0, window=999, appear_min=0.0)
    red = appearance(_img((200, 30, 30)), BOX)
    blue = appearance(_img((30, 30, 200)), BOX)
    m.update(_det(200.0, bw=84.0, colour=0.5), 12.0, red)
    v, _ = m.update(_det(200.0, bw=84.0, colour=0.5), 12.0, blue)
    assert v == "PRESENT"


# --------------------------------------------------------------------- runner


# ------------------------------------------------- search sweep (2026-08-15)

def test_the_search_sweep_returns_to_where_the_target_was():
    """The defect this replaced: `yaw_rate = side * search_rate` had a constant
    sign, so despite being called a sweep the nose rotated and never came back.
    Measured on demo_traffic, one lost-lock episode swept 292 deg -- the "360
    manoeuvre" reported from the video.

    It was the worst possible response to why the lock is lost there. All three
    episodes across two flights happened at the same place with the target dead
    ahead (aspect 0.4-2.9 deg) and presence still PRESENT: the car sits behind a
    street tree. Turning away from a bearing that is still correct cannot help.
    """
    dt, period = 0.1, 4.0
    ang = 0.0
    for i in range(int(period * 4 / dt)):
        ang += search_sweep_rate(i * dt, 25.0, period) * dt
    assert abs(math.degrees(ang)) < 1.0, (
        f"sweep drifted {math.degrees(ang):.1f} deg over four periods; it must "
        f"oscillate about the last bearing, not rotate away from it")


def test_the_search_sweep_keeps_the_last_bearing_in_frame():
    """25 deg of half-amplitude against a 45 deg horizontal half-FOV. If the
    excursion exceeded the half-FOV the sweep would carry the last known bearing
    out of the picture, which is the thing it exists to avoid."""
    dt, period = 0.05, 4.0
    ang = worst = 0.0
    for i in range(int(period * 2 / dt)):
        ang += search_sweep_rate(i * dt, 25.0, period) * dt
        worst = max(worst, abs(ang))
    assert math.degrees(worst) < 45.0, f"swept {math.degrees(worst):.0f} deg off axis"
    assert math.degrees(worst) > 15.0, "sweep too small to re-find anything"


def test_the_search_sweep_respects_the_yaw_cap():
    """A short period with a wide sweep would otherwise command a yaw rate the
    airframe cannot hold, and the resulting clipping would break the
    zero-net-rotation property silently."""
    for r in (search_sweep_rate(t * 0.05, 25.0, 4.0) for t in range(200)):
        assert abs(r) <= 1.1 + 1e-9
    assert abs(search_sweep_rate(0.0, 90.0, 0.5)) == 1.1, "cap not applied"


# --------------------------- the colour gate and sunlight (aug 2026)

def _car(bright: bool, rgb_shade=(196, 168, 52)):
    """The same yellow car, lit two ways, on asphalt.

    `bright` lifts every channel toward white the way sunlight does. That leaves
    the HUE alone and leaves the ABSOLUTE chroma nearly alone, but it inflates
    the denominator of HSV saturation - which is what the old gate keyed on.
    """
    H, W = 60, 80
    img = np.full((H, W, 3), 70, np.uint8)
    rgb = rgb_shade
    if bright:
        # 0.60 reproduces the saturation actually measured in flight:
        # the car's yellow pixels fell to a median S of 69 in sunlight,
        # against 99 in shade. A gentler wash leaves S above 90 and the
        # fixture would not reproduce the fault at all.
        rgb = tuple(int(c + (255 - c) * 0.60) for c in rgb_shade)
    img[20:46, 22:60] = rgb
    return img, (14, 14, 68, 52)


def test_the_same_car_passes_in_shade_and_in_sun():
    """THE REGRESSION. Measured in flight: the car's yellow pixels dropped from
    median saturation 99 to 69 when it drove into sunlight at the intersection,
    8.1% of them cleared the old s>90 floor instead of 56.7%, and 25 consecutive
    boxes were rejected while the taxi filled the frame."""
    shade_img, box = _car(bright=False)
    sun_img, _ = _car(bright=True)
    shade = colour_match(shade_img, box, "yellow")
    sun = colour_match(sun_img, box, "yellow")
    assert shade >= 0.10, f"shaded car fails the gate: {shade:.3f}"
    assert sun >= 0.10, (
        f"the SAME car fails once the sun is on it: shade {shade:.3f}, "
        f"sun {sun:.3f}, gate 0.10")


def test_sunlight_would_have_broken_a_saturation_floor():
    """Pins the mechanism, so a future change back to a saturation ratio fails
    here rather than in flight."""
    import cv2
    sun_img, box = _car(bright=True)
    x0, y0, x1, y1 = box
    hsv = cv2.cvtColor(sun_img[y0:y1, x0:x1], cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    hue = (h >= 22) & (h <= 35)
    assert hue.any()
    assert (s[hue] > 90).mean() < 0.5, "fixture does not reproduce the washout"
    chroma = s[hue].astype(float) * v[hue] / 255.0
    assert (chroma > 40).mean() > 0.9, "chroma should survive the sunlight"


def test_road_and_zebra_crossing_still_score_nothing():
    """The gate must keep rejecting the background completely. Measured on the
    real frames: road+zebra 0.0004, plain road 0.0000."""
    H, W = 60, 80
    road = np.full((H, W, 3), 70, np.uint8)
    assert colour_match(road, (0, 0, W, H), "yellow") < 0.01
    for x in range(0, W, 8):
        road[:, x:x + 4] = 245                       # zebra stripes
    assert colour_match(road, (0, 0, W, H), "yellow") < 0.01


def test_the_wrong_colour_still_fails():
    """The discrimination claim must not be weakened by the looser floor."""
    for bright in (False, True):
        img, box = _car(bright=bright, rgb_shade=(190, 40, 40))   # red car
        assert colour_match(img, box, "yellow") < 0.10


# ------------------- depth mask: built for a hypothesis that was wrong ------
# The first diagnosis was that the zebra crossing DILUTED the yellow fraction by
# filling the box. That was an artefact of a hand-projected box; replaying the
# real detector boxes showed the car's own pixels failing the saturation floor.
# The mask is kept because it is a strictly more accurate measurement, but it is
# OFF by default and unproven in flight.

def _scene_with_depth(car_rgb=(196, 168, 52)):
    H, W = 60, 80
    img = np.full((H, W, 3), 70, np.uint8)
    depth = np.full((H, W), 20.0, np.float32)
    img[22:44, 26:56] = car_rgb
    # The depth stream is quantised to whole metres, so the mask margin is 1.0 m
    # and the fixture has to present a step the stream could actually carry. A
    # real car's roof reads about 1 m nearer than the road at the depression
    # angles flown here, which is exactly the marginal case.
    depth[22:44, 26:56] = 18.0
    return img, depth, (10, 14, 72, 52)


def test_the_depth_mask_keeps_the_object_and_drops_the_ground():
    img, depth, box = _scene_with_depth()
    m = object_mask_from_depth(depth, box)
    assert m is not None and 0.05 < m.mean() < 0.90
    assert colour_match(img, box, "yellow", depth=depth) >         colour_match(img, box, "yellow")


def test_the_depth_mask_falls_back_rather_than_guessing():
    img, depth, box = _scene_with_depth()
    base = colour_match(img, box, "yellow")
    assert colour_match(img, box, "yellow", depth=None) == base
    assert colour_match(img, box, "yellow", depth=np.zeros_like(depth)) == base
    assert colour_match(img, box, "yellow",
                        depth=np.full_like(depth, 65504.0)) == base
    assert object_mask_from_depth(np.full_like(depth, 20.0), box) is None
    assert object_mask_from_depth(np.full_like(depth, 5.0), box) is None


def test_the_stats_counter_reports_which_path_ran():
    img, depth, box = _scene_with_depth()
    st = {}
    colour_match(img, box, "yellow", depth=depth, stats=st)
    colour_match(img, box, "yellow", depth=None, stats=st)
    assert st.get("masked") == 1 and st.get("whole_box") == 1, st


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
