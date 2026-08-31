"""
obstacle_clearance golden cases — one test per failure mode the review found.

Run either way:
    pytest tests/test_clearance.py -v
    python tests/test_clearance.py

The invariant behind every case: the Shield must never answer a clearance
problem with a FREEZE. A stationary vehicle inside the clearance ring never
recedes, so a brake there re-raises the identical violation on the next tick and
every tick after it — the fail-safe becomes the deadlock.
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import Action4D, Shield, State, load_policy       # noqa: E402
from guardrail.models import (KinematicEnvelope, ObstacleClearance,  # noqa: E402
                              Policy)
from guardrail.shield import BRAKE                               # noqa: E402

CITYMAP = ROOT / "demo" / "out" / "citymap" / "occ_day.npz"
URBAN = ROOT / "policies" / "urban_clearance.yaml"

# Synthetic maps mirror the shipped ones: 80 x 80 cells at 2 m, origin (-80,-80).
RES, OX, OY, N = 2.0, -80.0, -80.0, 80


def _ci(x):                      # world x -> row index (city_planner's mapping)
    return int(round((x - OX) / RES))


def _cj(y):                      # world y -> column index
    return int(round((y - OY) / RES))


def _shield(occ, min_clearance=6.0, soft=2.0, cap=4.0):
    pol = Policy(policy_id="clr-test", constraints=[
        ObstacleClearance(id="clr", type="obstacle_clearance",
                          min_clearance_m=min_clearance, soft_margin_m=soft),
        KinematicEnvelope(id="kin", type="kinematic_envelope", speed_max_mps=cap,
                          climb_rate_max_mps=2.0, yaw_rate_max_dps=45.0)])
    return Shield(pol, lookahead_s=3.0, dt=0.5,
                  obstacle_map={"occ": occ, "res": RES, "ox": OX, "oy": OY})


def _city_shield():
    import city_planner
    cm = city_planner.load_occ(str(CITYMAP))
    sh = Shield(load_policy(URBAN), lookahead_s=3.0, dt=0.5,
                obstacle_map={"occ": cm["occ"], "res": cm["res"],
                              "ox": cm["ox"], "oy": cm["oy"]})
    return sh, cm


def _step_dist(sh, st, a, t=0.5):
    """Distance one control step along `a` — "is it actually getting away?"."""
    return sh._distance_at(st.x + a.vx * t, st.y + a.vy * t)


def _moving(a, floor=0.5):
    return math.hypot(a.vx, a.vy) > floor


# ------------------------------------------------------- distance field basics

def test_distance_field_is_signed_inside_buildings():
    """The unsigned field read 0 across a whole footprint, so its gradient there
    was flat and "the way out" was undefined."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(-10):_ci(10) + 1, _cj(-10):_cj(10) + 1] = 1     # 20 m square block
    sh = _shield(occ)
    assert sh._distance_at(0.0, 0.0) < 0.0                  # dead centre: inside
    assert sh._distance_at(0.0, 30.0) > 0.0                 # out in the open
    assert sh._distance_at(0.0, 0.0) < sh._distance_at(0.0, -8.0)  # deeper = lower
    assert sh._away_dir(0.0, -8.0, 6.0) != (0.0, 0.0)       # always a way out


def test_away_dir_never_returns_no_direction_on_the_city_map():
    """Finding: 39% of occupied cells on the shipped map answered "no direction",
    which fell through to BRAKE and froze the drone in mid-air over a building
    (the demo's ESCAPE mode climbs onto exactly those cells on purpose)."""
    sh, cm = _city_shield()
    occ = np.asarray(cm["occ"])
    dead = 0
    for i, j in zip(*np.nonzero(occ)):
        x = cm["ox"] + int(i) * cm["res"]
        y = cm["oy"] + int(j) * cm["res"]
        if sh._away_dir(x, y, 5.0) == (0.0, 0.0):
            dead += 1
    assert dead == 0, f"{dead} occupied cells still have no escape bearing"


def test_no_freeze_on_a_building_footprint():
    """The reported dead cell: the old Shield emitted (0,0,0) braked=True here,
    forever, at 45 m altitude."""
    sh, _ = _city_shield()
    st = State(x=-74.0, y=-18.0, up=45.0)
    assert sh._distance_at(st.x, st.y) < 0.0                # over a building
    d = sh.filter(st, Action4D(vx=3.0, vy=0.0, vz_up=1.0))
    assert not d.braked
    assert _moving(d.emitted)
    assert _step_dist(sh, st, d.emitted) > sh._distance_at(st.x, st.y)


# ------------------------------------------------------------ the repair chain

def test_geofence_tangent_is_re_repaired_for_clearance():
    """Repair ordering: _repair_geofence ran AFTER _repair_clearance and its
    output was never re-repaired, so the anti-stall tangent tripped the clearance
    rule and the P0 guard braked — permanently, at the demo's own spawn."""
    sh, _ = _city_shield()
    st = State(x=36.4, y=-18.8, up=45.0)
    d = sh.filter(st, Action4D(vx=-0.05, vy=4.0))
    assert not d.braked
    assert _moving(d.emitted)
    assert not sh._check(st, d.emitted)


def test_escape_direction_comes_from_the_forecast_not_the_present():
    """The push used the gradient at the CURRENT pose while the depth came from
    the forecast. With a different building nearest right now, "outward" pointed
    straight AT the future obstacle and the repair accelerated into it."""
    occ = np.zeros((N, N), np.uint8)
    occ[:, :_cj(-12) + 1] = 1                 # building A, face at y = -12
    occ[:, _cj(6):] = 1                       # building B, face at y = +6
    sh = _shield(occ, min_clearance=6.0, soft=2.0)
    st = State(x=0.0, y=-4.0, up=40.0)
    raw = Action4D(vx=0.0, vy=4.0)            # heading at B, but A is nearer NOW
    assert sh._distance_at(st.x, st.y) >= 6.0             # legal right now...
    assert sh._check(st, raw)                             # ...forecast is not
    before = sh._clear_min_dist(st, raw)
    d = sh.filter(st, raw)
    assert d.emitted.vy <= 0.0                            # never accelerate at B
    assert sh._clear_min_dist(st, d.emitted) > before      # repairs must improve
    assert not d.braked


def test_repair_never_lowers_the_forecast_clearance():
    """The invariant the old code had no way to state, swept over headings."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(8):_ci(20) + 1, _cj(-30):_cj(-6) + 1] = 1
    occ[_ci(-20):_ci(-8) + 1, _cj(6):_cj(30) + 1] = 1
    sh = _shield(occ, min_clearance=6.0, soft=2.0)
    checked = 0
    for x in range(-24, 25, 4):
        for y in range(-24, 25, 4):
            st = State(x=float(x), y=float(y), up=40.0)
            for k in range(8):
                ang = k * math.pi / 4.0
                raw = Action4D(vx=4.0 * math.cos(ang), vy=4.0 * math.sin(ang))
                if not sh._check(st, raw):
                    continue
                d = sh.filter(st, raw)
                if d.braked:
                    continue
                checked += 1
                assert (sh._clear_min_dist(st, d.emitted)
                        >= sh._clear_min_dist(st, raw) - 1e-9), (
                    f"repair made it worse at ({x},{y}) heading {k}")
    assert checked > 50, f"only {checked} repaired samples exercised"


# ------------------------------------------------------------ the trend-aware check

def test_receding_does_not_green_light_the_whole_horizon():
    """`if d_now < min and receding: continue` dropped the ENTIRE lookahead for
    that rule, so a drone inside the ring got a free pass on every obstacle on
    the map — including a forecast that ended inside a wall."""
    occ = np.zeros((N, N), np.uint8)
    occ[:, :_cj(-2) + 1] = 1                  # west wall, face at y = -2
    occ[:, _cj(6):] = 1                       # east wall, face at y = +6
    sh = _shield(occ, min_clearance=5.0, soft=2.0)
    st = State(x=0.0, y=0.0, up=40.0)
    raw = Action4D(vx=0.0, vy=4.0)
    d_now = sh._distance_at(st.x, st.y)
    assert d_now < 5.0                                     # inside the ring...
    assert _step_dist(sh, st, raw) > d_now                 # ...and receding
    viol = sh._check(st, raw)
    assert viol, "forecast crosses the far wall but was not flagged"
    assert viol[0].predicted_at_s > 0.0                    # caught in the FUTURE
    d = sh.filter(st, raw)
    assert d.touched and not d.braked
    assert not sh._check(st, d.emitted)


def test_escaping_the_ring_is_still_not_flagged():
    """The other half of trend-awareness: retreating from a wall must stay a
    clean passthrough, or the Shield fights its own recovery."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(10):, :] = 1                                   # wall, face at x = 10
    sh = _shield(occ, min_clearance=6.0, soft=2.0)
    st = State(x=6.0, y=0.0, up=40.0)
    assert sh._distance_at(st.x, st.y) < 6.0               # inside the ring
    out = sh.filter(st, Action4D(vx=-3.0, vy=0.0))
    assert not out.violations and out.emitted == out.raw and not out.braked
    deeper = sh.filter(st, Action4D(vx=3.0, vy=0.0))
    assert any(v.category == "clearance" for v in deeper.violations)
    hover = sh.filter(st, Action4D())
    assert any(v.category == "clearance" for v in hover.violations)


# ------------------------------------------------------------------- the brake

def test_brake_is_never_emitted_when_stopping_is_itself_illegal():
    """filter() returned BRAKE without ever re-checking it. Inside the ring a
    zero action leaves d_next == d_now, so `receding` is false and the identical
    violation is raised forever."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(0):_ci(6) + 1, _cj(0):_cj(6) + 1] = 1
    sh = _shield(occ, min_clearance=6.0, soft=2.0)
    st = State(x=2.0, y=-3.0, up=40.0)
    assert sh._check(st, BRAKE), "test premise: stopping here IS a violation"
    for raw in (Action4D(), Action4D(vx=0.0, vy=4.0), Action4D(vx=4.0, vy=0.0),
                Action4D(vx=-2.8, vy=2.8)):
        d = sh.filter(st, raw)
        assert not d.braked, f"froze on raw {raw}"
        assert _moving(d.emitted), f"emitted a standstill on raw {raw}"
        assert _step_dist(sh, st, d.emitted) > sh._distance_at(st.x, st.y)


def test_wedged_between_a_wall_and_a_zone_still_recovers():
    """The reported deadlock: ClearanceFix pushes into the NFZ, GeofenceSlide
    cancels it, the P0 guard brakes, and at rest the clearance violation
    persists. Four consecutive ticks of (0,0) that never terminate."""
    from guardrail.models import PolygonFence, XY
    occ = np.zeros((N, N), np.uint8)
    occ[:, _cj(-16):_cj(-8) + 1] = 1                       # wall face at y = -8
    pol = Policy(policy_id="wedge", constraints=[
        ObstacleClearance(id="clr", type="obstacle_clearance",
                          min_clearance_m=6.0, soft_margin_m=2.0),
        PolygonFence(id="nfz", type="polygon_fence",
                     vertices=[XY(x=-40, y=1), XY(x=40, y=1),
                               XY(x=40, y=40), XY(x=-40, y=40)],
                     altitude_floor_m=0, altitude_ceiling_m=60),
        KinematicEnvelope(id="kin", type="kinematic_envelope", speed_max_mps=4.0,
                          climb_rate_max_mps=2.0, yaw_rate_max_dps=45.0)])
    sh = Shield(pol, lookahead_s=3.0, dt=0.5,
                obstacle_map={"occ": occ, "res": RES, "ox": OX, "oy": OY})
    x, y = 0.0, -4.0
    frozen = 0
    for _ in range(40):                                    # 4 s at 10 Hz
        st = State(x=x, y=y, up=40.0)
        d = sh.filter(st, Action4D(vx=0.0, vy=2.0))        # commanded at the NFZ
        if not _moving(d.emitted, 0.2):
            frozen += 1
        x += d.emitted.vx * 0.1
        y += d.emitted.vy * 0.1
    assert frozen == 0, f"stood still on {frozen}/40 ticks in the sliver"


# --------------------------------------------------------------- the standoff

def test_head_on_standoff_is_braking_sized_not_lookahead_sized():
    """A constant-velocity scan over the full 3 s geofence horizon turns a 5 m
    rule into a 17 m standoff at 4 m/s: the drone stopped dead 15.75 m from the
    wall and never got closer."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(20):, :] = 1                                   # wall, face at x = 20
    sh = _shield(occ, min_clearance=5.0, soft=2.0)
    x, y, brakes = -20.0, 0.0, 0
    closest = 1e9
    for _ in range(200):                                   # 20 s at 10 Hz
        st = State(x=x, y=y, up=40.0)
        d = sh.filter(st, Action4D(vx=4.0, vy=0.0))        # straight at the wall
        brakes += d.braked
        x += d.emitted.vx * 0.1
        y += d.emitted.vy * 0.1
        closest = min(closest, sh._distance_at(x, y))
    gap = sh._distance_at(x, y)
    assert brakes == 0, f"braked {brakes}/200 ticks on a head-on approach"
    assert closest >= 5.0 - 1.2, f"breached the ring: closest {closest:.2f} m"
    assert gap < 12.0, f"still parked {gap:.2f} m out — standoff not fixed"


def test_planned_routes_are_flyable_with_the_shipped_policy():
    """End to end on the shipped artefacts: urban_clearance.yaml over
    occ_day.npz, from the demo's own spawn, following the demo's own planner.
    The measured brake rate before the fix was 95%, and 11/11 routes stalled."""
    import city_planner
    from shapely.geometry import Point

    from guardrail.geometry import fence_polygon
    from guardrail.models import PolygonFence

    sh, cm = _city_shield()
    # Same planning grid the demo builds: surveyed buildings + every policy NFZ
    # stamped in, then inflated once. Planning against buildings ALONE routes
    # straight through the no-fly zones, and the resulting stalls are the
    # geofence slide's, not the clearance rule's.
    occ, res = np.asarray(cm["occ"]).copy(), cm["res"]
    ox, oy = cm["ox"], cm["oy"]
    for f in sh.policy.by_type(PolygonFence):
        buf = fence_polygon(f).buffer(f.margin_m)
        minx, miny, maxx, maxy = buf.bounds
        for i in range(max(0, int((minx - ox) / res)),
                       min(occ.shape[0], int((maxx - ox) / res) + 1)):
            for j in range(max(0, int((miny - oy) / res)),
                           min(occ.shape[1], int((maxy - oy) / res) + 1)):
                if occ[i, j] == 0 and buf.contains(Point(ox + i * res, oy + j * res)):
                    occ[i, j] = 1
    grid = city_planner.inflate(occ, res, 6.0)
    rng = np.random.default_rng(11)
    start = (35.0, -20.0)

    # Sample goals, keep what the planner can actually reach, and fly the TEN
    # LONGEST routes — short hops would not exercise the rule enough to mean
    # anything. (Most of the map is unreachable from the spawn once the 6 m
    # inflation and the NFZs are in, so goals cannot simply be drawn far away.)
    def _plen(p):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p[:-1], p[1:]))

    found = []
    for _ in range(150):
        goal = (float(rng.integers(-70, 70)), float(rng.integers(-70, 70)))
        path = city_planner.plan(grid, res, ox, oy, start, goal)
        if path is None or len(path) < 2:
            continue
        pts = [(float(a), float(b)) for a, b in path]
        found.append((_plen(pts), pts))
    found.sort(key=lambda t: -t[0])
    assert len(found) >= 10, f"only {len(found)} routes planned — map/planner problem"

    def fly(drift):
        """Carrot follower at 4 m/s, 10 Hz, Shield in the loop. `drift` bends the
        commanded heading away from the planned corridor — the VLA wander the
        rule actually exists to catch (at drift = 0 the planner's own 6 m
        inflation should keep the Shield silent)."""
        brake = ticks = stalled = touched = frozen = 0
        closest = 1e9
        for _, pts in found[:10]:
            x, y, up, seg = pts[0][0], pts[0][1], 45.0, 1
            for k in range(900):                           # 90 s at 10 Hz
                while seg < len(pts) and math.hypot(pts[seg][0] - x, pts[seg][1] - y) < 3.0:
                    seg += 1
                if seg >= len(pts):
                    break
                ticks += 1
                gx, gy = pts[seg]
                ang = math.atan2(gy - y, gx - x) + drift * math.sin(k / 60.0)
                d = sh.filter(State(x=x, y=y, up=up),
                              Action4D(vx=4.0 * math.cos(ang), vy=4.0 * math.sin(ang)))
                brake += d.braked
                touched += d.touched
                frozen += not _moving(d.emitted, 0.2)
                x += d.emitted.vx * 0.1
                y += d.emitted.vy * 0.1
                up += d.emitted.vz_up * 0.1
                closest = min(closest, sh._distance_at(x, y))
            if seg < len(pts):
                stalled += 1
        return brake, ticks, stalled, touched, frozen, closest

    # 1) nominal: on its own planned corridor the Shield must stay out of the way
    brake, ticks, stalled, touched, frozen, closest = fly(0.0)
    print(f"    nominal: {ticks} ticks, brake {brake}, touched {touched}, "
          f"stalled {stalled}, closest {closest:.2f} m")
    assert ticks > 800, f"only {ticks} ticks flown — the sweep is too thin"
    assert brake == 0, f"braked {brake}/{ticks} ticks on planner-clean routes"
    assert stalled == 0, f"{stalled}/10 planned routes stalled"
    assert closest > 0.0, "flew through a building"

    # 2) drifting off the corridor: now the rule DOES fire, and it must steer
    #    rather than stop — this is where the old code braked 95% of ticks.
    brake, ticks, stalled, touched, frozen, closest = fly(1.5)
    rate = 100.0 * brake / max(1, ticks)
    print(f"    drifting: {ticks} ticks, brake {brake} ({rate:.1f}%), "
          f"touched {touched}, frozen {frozen}, closest {closest:.2f} m")
    assert touched > 500, f"only {touched} interventions — the drift did not bite"
    assert rate < 5.0, f"brake rate {rate:.1f}% >= 5%"
    assert frozen < 0.05 * ticks, f"stood still on {frozen}/{ticks} ticks"
    assert closest > 3.0, f"steered too close to a building ({closest:.2f} m)"


# ------------------------------------------------------------------ inertness

def test_no_map_leaves_clearance_rules_inert():
    """A policy that names obstacle_clearance but a Shield built without a map:
    the rule must never fire and never raise."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(10):, :] = 1
    armed = _shield(occ, min_clearance=6.0)
    inert = Shield(armed.policy, lookahead_s=3.0, dt=0.5, obstacle_map=None)
    st = State(x=8.0, y=0.0, up=40.0)
    for a in (Action4D(vx=2.0), Action4D(vx=-2.0), Action4D(vy=3.0), Action4D()):
        d = inert.filter(st, a)
        assert not d.violations and not d.braked and d.emitted == a
    assert armed.filter(st, Action4D(vx=2.0)).touched     # ...but armed it fires


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
