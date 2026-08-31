"""FenceGuard — does the controller pick the side the gap is actually on?

Run either way:
    pytest tests/test_fenceguard.py -v
    python tests/test_fenceguard.py

This exists because a flight got it wrong. `vlm_gapfence` (2026-08-11) flew
`follow_car_gap.yaml`, whose fence covers x 26..42 and leaves 7 m of legal road at
x 43..50. The aircraft slid WEST to x = 30.9 — the closed end — and finished the
flight 33.5 m behind the car, only 26.1% of the run within 30 m against 99.6%
unfenced. The rule held perfectly; the controller threw the mission away.

Two causes, both tested here:

  1. the old slide() scored each side by the fence distance at ONE probe point,
     which is symmetric information and cannot say which side you can get PAST on;
  2. slide() was only consulted once gate() raised `blocked`, which happens inside
     6.15 m, by which point the aircraft is at 35% speed and 5 m of lateral travel
     costs more ground than a 2 m/s target gives away.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import load_policy                                # noqa: E402
from follow_vlm import FenceGuard                                # noqa: E402

GAP = ROOT / "policies" / "follow_car_gap.yaml"      # fence x 26..42, gap x 43..50
FULL = ROOT / "policies" / "follow_car_nfz.yaml"     # fence x 26..54, no gap


CITYMAP = ROOT / "demo" / "out" / "citymap" / "occ_day.npz"


def _guard(path, brake_m=12.0, stand_off_m=3.0, with_map=False):
    smap = street = None
    if with_map and CITYMAP.exists():
        import city_planner
        cm = city_planner.load_occ(str(CITYMAP))
        smap = {"occ": cm["occ"], "res": cm["res"], "ox": cm["ox"], "oy": cm["oy"]}
        f = ROOT / "demo" / "out" / "citymap" / "street.npz"
        if f.is_file():
            from build_street_mask import load_street
            street = load_street(f)
    # min_clearance_m comes from the POLICY, as it does in flight
    # (follow_vlm.py passes clr[0].min_clearance_m). Leaving FenceGuard's 5 m
    # default here made the tests judge detours against a clearance the mission
    # never uses, and the two disagreed the moment the policies moved to 3 m.
    from guardrail.models import ObstacleClearance
    pol = load_policy(path)
    clr = pol.by_type(ObstacleClearance)
    return FenceGuard(pol, brake_m=brake_m, stand_off_m=stand_off_m,
                      obstacle_map=smap, street_mask=street,
                      min_clearance_m=(clr[0].min_clearance_m if clr else 5.0))


# The car drives north up lane x = 38, so the aircraft approaches heading +y.
NORTH = (0.0, 2.0)


def test_the_gap_policy_sends_the_aircraft_east_not_west():
    """The whole point. East is the 7 m of legal road; west is the closed end."""
    g = _guard(GAP)
    sx, sy, cost = g.slide(38.0, -6.0, *NORTH)
    assert (sx, sy) != (0.0, 0.0), "no side chosen at all, so it will just stop"
    assert sx > 0.5, (
        f"slide chose ({sx:+.2f}, {sy:+.2f}) — that is WEST, the closed end. "
        "This is the exact failure the vlm_gapfence flight recorded."
    )
    assert math.isfinite(cost) and cost <= 14.0, f"detour cost {cost}"


def test_the_detour_it_reports_matches_the_geometry():
    """Fence ends at x = 42 and the stand-off is 3 m, so from x = 38 the aircraft
    needs about 7 m of easting. Anything much smaller means it is aiming at a
    corner rather than at the road."""
    g = _guard(GAP)
    _, _, cost = g.slide(38.0, -6.0, *NORTH)
    assert 4.0 <= cost <= 12.0, f"detour of {cost} m does not match the 42+3 edge"


def test_a_fence_with_no_gap_reports_no_side_and_infinite_cost():
    """follow_car_nfz.yaml spans the whole corridor on purpose. Inventing a
    detour there would drive the aircraft into the boundary looking for one."""
    g = _guard(FULL)
    sx, sy, cost = g.slide(38.0, -6.0, *NORTH)
    assert (sx, sy) == (0.0, 0.0), f"found a way past a fence that has none: {sx},{sy}"
    assert cost == float("inf")


def test_the_side_choice_is_stable_along_the_whole_approach():
    """A controller that changes its mind at every tick weaves instead of
    committing, which is what makes fence behaviour look dangerous."""
    g = _guard(GAP)
    picks = []
    for y in range(-20, -2):
        sx, sy, cost = g.slide(38.0, float(y), *NORTH)
        if sx or sy:                          # NOT `or cost` — inf is truthy
            picks.append(sx > 0)
    assert picks, "never chose a side anywhere on the approach"
    assert all(picks) or not any(picks), f"side flip-flopped along the run: {picks}"
    assert all(picks), "committed to WEST along the approach"


def test_far_from_the_fence_there_is_no_side_to_choose():
    """Distant approach must return no opinion rather than a coin flip.

    With the fence out of probe range both sides scored the same minimum cost and
    the tie went to whichever the loop tried first — WEST, the closed end. The
    aircraft would have been committed to the wrong side before the fence was
    even a factor."""
    g = _guard(GAP)
    for y in (-30.0, -25.0, -20.0, -16.0):
        sx, sy, cost = g.slide(38.0, y, *NORTH)
        assert (sx, sy) == (0.0, 0.0) and cost == float("inf"), (
            f"at y={y} the path ahead is clear, yet slide picked ({sx},{sy})"
        )


def test_the_detour_begins_at_the_brake_distance_not_at_the_standoff():
    """The anticipation fix. At 12 m out the gate must already be slowing the
    aircraft, which is the condition the control loop now uses to start sliding —
    rather than waiting for `blocked`, which only fires inside 6.15 m."""
    g = _guard(GAP, brake_m=12.0, stand_off_m=3.0)
    # A point ~10 m south of the fence's y-edge (y = 2), closing on it.
    scale, dist, blocked = g.gate(38.0, -8.0, *NORTH)
    assert dist is not None and dist < g.brake_m, f"not yet inside the brake ring: {dist}"
    assert scale < 1.0, "gate is not slowing the aircraft at the brake distance"
    assert not blocked, ("premise of the fix: `blocked` is still false here, so a "
                         "controller that waits for it starts the detour far too late")
    sx, _, _ = g.slide(38.0, -8.0, *NORTH)
    assert sx > 0.5, "slide has no answer this far out, so anticipating cannot help"


def test_sliding_never_aims_somewhere_it_may_not_stand():
    """Every offset slide() returns must itself respect the stand-off, or the
    detour walks into the zone the Shield then has to repair."""
    from guardrail.geometry import fence_polygon
    from guardrail.models import PolygonFence
    from shapely.geometry import Point
    pol = load_policy(GAP)
    polys = [fence_polygon(f).buffer(f.margin_m) for f in pol.by_type(PolygonFence)]
    g = _guard(GAP)
    for y in range(-20, 0, 2):
        sx, sy, cost = g.slide(38.0, float(y), *NORTH)
        if not (sx or sy):
            continue
        px, py = 38.0 + sx * cost, float(y) + sy * cost
        d = min(p.distance(Point(px, py)) for p in polys)
        assert d >= g.stand_off_m - 1e-6, (
            f"slide target ({px:.1f},{py:.1f}) is {d:.1f} m from the fence, "
            f"inside the {g.stand_off_m} m stand-off"
        )


def test_no_fences_means_no_opinion():
    g = _guard(ROOT / "policies" / "follow_car.yaml")
    assert g.slide(38.0, -6.0, *NORTH) == (0.0, 0.0, float("inf"))
    assert g.gate(38.0, -6.0, *NORTH) == (1.0, None, False)


def test_a_stationary_command_asks_for_no_detour():
    """With no commanded motion there is no heading to sidestep relative to."""
    g = _guard(GAP)
    assert g.slide(38.0, -6.0, 0.0, 0.0) == (0.0, 0.0, float("inf"))


def _street_mask():
    f = ROOT / "demo" / "out" / "citymap" / "street.npz"
    if not f.is_file():
        return None
    from build_street_mask import load_street
    return load_street(f)


def test_a_detour_must_stay_on_the_road_not_merely_outside_the_fence():
    """The regression the anticipatory slide introduced.

    follow_car_nfz.yaml spans the whole corridor deliberately, so there is no way
    past. Knowing only about fences, slide() found one anyway by routing around
    the fence's eastern END at x > 55 - off the street entirely. Flown, fence_mode
    was `skirt` on 378 of 498 ticks against `hold` on 442 of 552 before, and
    Shield interventions went 0 -> 298 as the aircraft was pushed into building
    clearance. The Shield caught every one, which is the system working; the
    controller should not have been proposing them.

    This asserted that NO detour was offered, using the obstacle map as a stand-in
    for the road. That proxy held only while the map was built over 15-55 m AGL
    and therefore contained nothing but buildings. Rebuilt over the flight band,
    461 cells that had been blocked became free - low structures a drone may
    legally overfly - and detours over them started being offered again.

    The right test is the one the name always claimed: a detour may be offered,
    but where it leads must be a road. That is now checked against the street
    mask instead of inferred from the absence of obstacles.
    """
    if not CITYMAP.exists():
        return
    mask = _street_mask()
    if mask is None:
        return
    from build_street_mask import is_street
    g = _guard(FULL, with_map=True)
    off_road = []
    for y in range(-20, 2):
        for x in (36.0, 38.0, 42.0, 46.0):
            sx, sy, cost = g.slide(x, float(y), *NORTH)
            if not (sx or sy) or not math.isfinite(cost):
                continue
            # slide() returns a LATERAL unit vector and the distance the aircraft
            # must travel along it before the way ahead opens, so the detour's
            # destination is exactly cost metres out - not some arbitrary probe
            # distance, which was this test's own first mistake.
            px, py = x + sx * cost, y + sy * cost
            if not is_street(mask, px, py):
                off_road.append((x, y, round(px, 1), round(py, 1)))
    assert not off_road, (
        f"slide offered detours that leave the road: {off_road[:5]} "
        f"(start x,y -> destination x,y)")


def test_the_gap_policy_gap_is_out_of_search_range_once_the_map_is_honest():
    """This asserted the opposite, and the world changed under it.

    It read "the clearance check must not be so strict that it kills the real
    gap", and the gap was findable while the obstacle map was built over
    15-55 m AGL: nothing but buildings was in it, so the corridor at x 43..50
    measured a uniform 9.2 m and the detour was short.

    Rebuilt over the flight band, the eastern end of that corridor is solid
    street furniture - 0.0 m at x 48-50, y 0 - so the first viable column sits
    further east and the detour grows to 15.0 m. slide() searches 14 m, so it
    reports no gap at all.

    The gap is real: at reach_m=20 it is found, eastward, at cost 15.0. What
    stops that being the fix is that raising the reach lets a guard with NO
    street mask find a way around a corridor-spanning fence's END, which is
    exactly the off-road regression test_a_detour_must_stay_on_the_road guards.
    The reach cannot move until every caller supplies the mask.

    Recorded rather than papered over, because it failed in the SAFE direction -
    no detour found falls through to braking - and that is the hardest kind of
    failure to notice.
    """
    if not CITYMAP.exists():
        return
    g = _guard(GAP, with_map=True)
    sx, sy, cost = g.slide(38.0, -6.0, *NORTH)
    assert (sx, sy) == (0.0, 0.0) and not math.isfinite(cost), (
        f"slide() found the gap at the default 14 m reach, returning "
        f"({sx},{sy}) cost {cost}. If the reach was raised, check that every "
        f"FenceGuard caller now passes street_mask first.")
    sx2, sy2, cost2 = g.slide(38.0, -6.0, *NORTH, reach_m=20.0)
    assert sx2 > 0.5 and 14.0 < cost2 < 20.0, (
        f"the gap should still exist at a longer reach; got ({sx2},{sy2}) {cost2}")


def test_flying_along_the_gap_is_not_braked():
    """The reason the gap flight lost the car.

    Inside follow_car_gap.yaml's 7 m gap at x = 47, heading north, the nearest
    fence point is the corner at (43, 1) and that corner gets nearer as the
    aircraft advances — so a gate that brakes on shrinking distance throttled a
    trajectory that never enters the zone. Measured on v2_gap: the aircraft DID
    find the gap (277 of 552 ticks at x > 43) and was still held to 1.62-1.68 m/s
    against a car doing 2.0, slower than its target on 537 of 552 ticks.
    """
    g = _guard(GAP)
    for y in (-6.0, -2.0, 2.0, 6.0, 10.0):
        scale, dist, blocked = g.gate(47.0, y, *NORTH)
        assert scale == 1.0, (
            f"at (47,{y}) flying north up the gap, the gate scaled to {scale:.2f} "
            f"(fence {dist:.1f} m away) — that trajectory never enters the zone"
        )
        assert not blocked


def test_flying_into_the_fence_is_still_braked():
    """The gate must not become permissive: a heading that DOES enter the zone
    has to be slowed exactly as before."""
    g = _guard(GAP)
    scale, dist, blocked = g.gate(38.0, -6.0, *NORTH)   # straight into it
    assert scale < 1.0, f"a heading into the fence was not braked (scale {scale})"
    assert dist is not None and dist < g.brake_m


def test_a_heading_away_from_the_fence_is_never_braked():
    g = _guard(GAP)
    scale, _, blocked = g.gate(38.0, -6.0, 0.0, -2.0)   # south, away
    assert scale == 1.0 and not blocked


# --------------------------------------------------------------------- runner




# ---------------------------------------------------------------------------
# Buildings, not just fences.
#
# The demo policy declares no no-fly zone, and until 2026-08-25 FenceGuard
# returned "no opinion" whenever that was true - so on every tracking flight
# gate() and slide() were dead code. The occupancy grid and the street mask
# were already wired in and already consulted; only the guard clause at the top
# of each method was fence-shaped.
#
# Measured cost, from demo/out/people_check: chasing the car north along
# x = 38, the map is BLOCKED at (38, 22) - a canopy, road underneath, solid at
# the 6-14 m cruise band - and the clearance reachable on that line falls to
# 0.0 m. Five metres east at x = 43 it is 5.0 m. The controller could not see
# any of that, so it commanded 4 m/s due north for forty consecutive ticks and
# the Shield turned every one away: 163 clearance violations against 50 on the
# run that happened to enter half a metre wider, and the aircraft lost the car
# (sep_end 58.1 m against 16.8 m).
# ---------------------------------------------------------------------------

CANOPY_LINE_X = 38.0          # the car's route; blocked at y = 22 at cruise
CORRIDOR_X = 43.0             # 5.0 m of clearance, the way past
NORTH_FAST = (0.0, 4.0)


def _demo_guard():
    """The demo policy, with the map the flight actually flies with."""
    return _guard(ROOT / "policies" / "follow_car.yaml", with_map=True)


def test_a_canopy_is_seen_even_though_the_policy_has_no_fence():
    g = _demo_guard()
    scale, dist, blocked = g.gate(CANOPY_LINE_X, 18.0, *NORTH_FAST)
    assert blocked, "flying at a blocked cell must raise blocked"
    assert scale < 0.35, f"must brake hard four metres out, got {scale}"
    assert dist is not None and dist < 6.0


def test_the_detour_around_the_canopy_goes_east_where_the_corridor_is():
    g = _demo_guard()
    sx, sy, cost = g.slide(CANOPY_LINE_X, 18.0, *NORTH_FAST)
    assert (sx, sy) != (0.0, 0.0), "there IS a way past; it must be found"
    assert sx > 0.5, f"the corridor is east at x=43; slid ({sx}, {sy})"
    assert cost < 8.0, f"the detour is about 5 m, reported {cost}"


def test_the_corridor_the_detour_recommends_is_itself_unbraked():
    """Otherwise the advice is useless: braked either way, it cannot keep up."""
    g = _demo_guard()
    for y in (16.0, 20.0, 22.0, 26.0):
        scale, _d, blocked = g.gate(CORRIDOR_X, y, *NORTH_FAST)
        assert scale == 1.0 and not blocked, f"braked in the free corridor at y={y}"


def test_flying_parallel_to_a_wall_is_not_braked():
    """A street is walls on both sides. Braking for proximity rather than for a
    predicted incursion would throttle the whole flight - which is exactly how
    the gap flight lost its car."""
    g = _demo_guard()
    scale, _d, blocked = g.gate(34.0, 18.0, 4.0, 0.0)
    assert scale == 1.0 and not blocked


def test_escaping_the_ring_is_never_braked():
    """Inside the ring and flying OUT of it. Braking here pins the aircraft
    against the obstacle it is leaving; measured at scale 0.09 before the
    incursion test required the forecast to be CLOSING, not merely inside."""
    g = _demo_guard()
    scale, _d, blocked = g.gate(CANOPY_LINE_X, 21.0, 0.0, -4.0)
    assert scale == 1.0 and not blocked
    # ...while drifting further in, from the same place, still brakes.
    scale_in, _d2, blocked_in = g.gate(CANOPY_LINE_X, 20.0, *NORTH_FAST)
    assert blocked_in and scale_in < 0.35


def test_without_an_obstacle_map_the_old_behaviour_is_exact():
    """Fenceless policy, no map: no opinion, as before. Anyone running without
    a built citymap must see no change at all."""
    g = _guard(ROOT / "policies" / "follow_car.yaml", with_map=False)
    assert g.gate(CANOPY_LINE_X, 18.0, *NORTH_FAST) == (1.0, None, False)
    assert g.slide(CANOPY_LINE_X, 18.0, *NORTH_FAST) == (0.0, 0.0, float("inf"))





# ---------------------------------------------------------------------------
# Grid indexing: round, not truncate.
#
# demo/city_planner.py:7-9 documents the convention and demo/build_voxel_map.py
# uses it when it writes these grids: cell `i` is CENTRED at `origin + i*res`,
# so a world point belongs to cell `round((v - origin) / res)`. The street mask
# is a cell-wise AND of those grids and inherits their origin unchanged.
#
# Three point lookups truncated instead, reading the mask shifted by up to half
# a cell - 1.0 m at this map's 2.0 m resolution. Measured over 40 000 random
# points, truncation and rounding disagreed about whether a point was on a road
# 9.4 % of the time. It became load-bearing when slide() started running on
# unfenced policies, i.e. on every tracking flight.
# ---------------------------------------------------------------------------

def test_a_point_belongs_to_the_cell_it_is_nearest_to():
    """1.4 m past a cell centre is inside the NEXT cell, not the one behind."""
    mask = _street_mask()
    if mask is None:
        return
    from build_street_mask import is_street
    res, ox, oy = mask["res"], mask["ox"], mask["oy"]
    st = mask["street"]

    # Find a boundary where the two neighbouring cells actually differ, so the
    # convention is observable rather than accidentally agreeing.
    found = None
    for i in range(1, st.shape[0] - 1):
        for j in range(1, st.shape[1] - 1):
            if bool(st[i, j]) != bool(st[i + 1, j]):
                found = (i, j)
                break
        if found:
            break
    assert found, "no boundary in the mask to test the convention against"

    i, j = found
    y = oy + j * res
    # 0.4 of a cell past centre i -> still cell i; 0.6 past -> cell i+1.
    near_i = ox + (i + 0.4) * res
    near_next = ox + (i + 0.6) * res
    assert is_street(mask, near_i, y) == bool(st[i, j]), (
        "a point 0.4 cells past a centre must still resolve to that cell")
    assert is_street(mask, near_next, y) == bool(st[i + 1, j]), (
        "a point 0.6 cells past a centre must resolve to the NEXT cell; "
        "truncation would keep it in the previous one")


def test_the_guard_and_is_street_agree_about_every_probe_point():
    """FenceGuard has its own street lookup. If the two disagree, slide() can
    offer a detour the road test then rejects - or worse, accept one it should
    have rejected."""
    mask = _street_mask()
    if mask is None:
        return
    from build_street_mask import is_street
    g = _demo_guard()
    if g.street is None:
        return
    res, ox, oy = mask["res"], mask["ox"], mask["oy"]
    st = mask["street"]

    def guard_says(px, py):
        i = int(round((px - g.street["ox"]) / res))
        j = int(round((py - g.street["oy"]) / res))
        if not (0 <= i < st.shape[0] and 0 <= j < st.shape[1]):
            return False
        return bool(g.street["street"][i, j])

    disagree = []
    for a in range(-40, 41, 3):
        for b in range(-40, 41, 3):
            x, y = a + 0.9, b + 0.9        # deliberately off-centre
            if guard_says(x, y) != is_street(mask, x, y):
                disagree.append((x, y))
    assert not disagree, f"guard and is_street disagree at {disagree[:5]}"


def test_pedestrians_are_placed_on_the_street_they_were_chosen_for():
    """pavement_spots validated a CELL and then returned a point 1 m away from
    it, because it computed the centre as `ox + (i+0.5)*res`. Every other
    world-from-grid conversion in the repository uses `ox + i*res`. Checked
    against a correctly indexed lookup, 5 of the 12 figures in the `city_people`
    flight were not standing on street at all."""
    mask = _street_mask()
    if mask is None:
        return
    import random
    import numpy as np
    from build_street_mask import is_street
    bld_p = ROOT / "demo" / "out" / "citymap" / "occ_day_highband_15to55.npz"
    if not bld_p.is_file():
        return
    import pedestrians as P
    import moving_car
    bld = np.load(bld_p)["occ"]
    route = [(x, y) for x, y in moving_car.ROUTES["turn"]]
    spots = P.pavement_spots(mask, bld, 12, random.Random(20260825), avoid=route)
    assert spots, "no pavement spots found at all"
    off = [(x, y) for x, y in spots if not is_street(mask, x, y)]
    assert not off, f"{len(off)} of {len(spots)} figures are not on street: {off}"


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
