"""
Guardrail coverage suite — the safety cases NO flight has ever exercised.

Run either way:
    pytest tests/test_guardrail_coverage.py -v
    python tests/test_guardrail_coverage.py

Why this file exists. The 2026-08-05 review flagged that safety evidence stopped
at the no-fly zone: minimum stand-off, altitude limits and lost-target behaviour
had no test cases. Flights cannot close that gap, because a flight SAMPLES the
state space -- ten flights visit maybe a few thousand states along ten
trajectories, all of them ones a working controller chose to visit. The states
that break a safety layer are the ones a working controller never picks.

So this suite attacks the Shield directly, offline and deterministically:

  1. UNITS      -- is each cap reachable by the values production actually emits?
  2. FUZZ       -- P0 escape = 0 as a property over 20k random state/action pairs
  3. HOSTILE    -- NaN and infinity from a failing detector must not become flight
  4. COVERAGE   -- every constraint TYPE provably fires at least once
  5. CONFLICT   -- several rules violated at the same instant
  6. POLICIES   -- the three follow_car policies, including the never-flown one

Group 1 already found a live defect and Group 6 answers a question that would
otherwise cost a 12-minute flight to ask.
"""
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import Action4D, Shield, State, load_policy        # noqa: E402
from guardrail.models import (AltitudeEnvelope, KinematicEnvelope,  # noqa: E402
                              ObstacleClearance, PolygonFence)

DEMO = ROOT / "policies" / "demo_policy.yaml"
CITYMAP = ROOT / "demo" / "out" / "citymap" / "occ_day.npz"
FOLLOW = {
    "plain": ROOT / "policies" / "follow_car.yaml",
    "full-fence": ROOT / "policies" / "follow_car_nfz.yaml",
    "gap-fence": ROOT / "policies" / "follow_car_gap.yaml",
}


def _shield(path=DEMO, with_map=False):
    kw = {}
    if with_map:
        import city_planner
        cm = city_planner.load_occ(str(CITYMAP))
        kw["obstacle_map"] = {"occ": cm["occ"], "res": cm["res"],
                              "ox": cm["ox"], "oy": cm["oy"]}
    return Shield(load_policy(path), lookahead_s=3.0, dt=0.5, **kw)


def _prio(shield, rule_id: str) -> str:
    """Priority lives on the CONSTRAINT, not on the Violation it raises."""
    for c in shield.policy.constraints:
        if c.id == rule_id:
            return c.priority
    return "P0"          # unknown rule (e.g. the synthetic contract check)


def _finite(a: Action4D) -> bool:
    return all(math.isfinite(v) for v in
               (a.vx, a.vy, a.vz_up, a.yaw_rate))


# ------------------------------------------------------------------ 1. UNITS
#
# Action4D.yaw_rate is documented "deg/s" at guardrail/models.py:36, but every
# producer feeds rad/s: move_by_velocity_async(yaw_is_rate=True) takes rad/s, and
# the official upstream contract (vlaguard_common.frames) says rad/s too. The cap
# is named yaw_rate_max_dps and compared against the raw number, so the units
# disagree by a factor of 57.3 -- in the direction that makes the cap unreachable.

def test_yaw_cap_compares_degrees_with_degrees():
    """Fixed 2026-08-17. The policy states the cap in DEGREES per second and the
    Action4D contract carries yaw_rate in RADIANS per second; the two were
    compared raw, so a 45 dps cap effectively sat at 45 rad/s = 2578 dps and
    this P1 rule could never fire on any real action."""
    s = _shield()
    over = math.radians(60.0)                      # 60 dps, over the 45 dps cap
    d = s.filter(State(x=-20, y=-20, up=4), Action4D(yaw_rate=over))
    assert d.touched, "a 60 dps command must trip a 45 dps cap"
    assert math.degrees(abs(d.emitted.yaw_rate)) <= 45.0 + 1e-6


def test_the_yaw_cap_now_fires_at_flight_scale():
    """AerialVLA's widest output is +-1.1 rad/s = 63 dps, which SHOULD be
    clamped by a 45 dps cap and previously was not."""
    s = _shield()
    widest = 1.1
    assert math.degrees(widest) > 45.0, "premise: this should violate"
    d = s.filter(State(x=-20, y=-20, up=4), Action4D(yaw_rate=widest))
    assert abs(d.emitted.yaw_rate) < widest, "the cap is inert again"
    assert math.degrees(abs(d.emitted.yaw_rate)) <= 45.0 + 1e-6


def test_F1_the_shield_still_does_not_edit_yaw_in_the_follow_demo():
    """The property the follow demo's headline rests on: heading is the pilot's,
    the track is the guardrail's.

    Fixing the unit above made a P1 rule live for the first time, which is
    exactly the change that could have broken F1. Measured over all three demo
    flights, the WORST yaw command anywhere is 11.8 dps - four times under the
    45 dps cap - so the live rule is a no-op there. This asserts that margin
    rather than trusting it, and will fail if either the cap or the controller's
    yaw authority moves toward the other.
    """
    s = _shield()
    worst_seen_dps = 11.8
    for dps in (0.0, 5.0, worst_seen_dps, 2 * worst_seen_dps):
        yr = math.radians(dps)
        d = s.filter(State(x=-20, y=-20, up=4), Action4D(vx=1.0, yaw_rate=yr))
        assert d.emitted.yaw_rate == yr, (
            f"the Shield edited yaw at {dps} dps; F1 is broken and the follow "
            f"demo's claim needs restating")


# ------------------------------------------------------------------- 2. FUZZ
#
# The grant's hard KPI is "P0 violation escape rate = 0". A flight demonstrates
# that on one trajectory. This demonstrates it as a property.

def _fuzz(shield, n=20000, seed=20260810, span=60.0):
    """Random states x random actions -> how often does the OUTPUT still violate?"""
    rng = random.Random(seed)
    escapes, braked, touched = [], 0, 0
    for _ in range(n):
        st = State(x=rng.uniform(-span, span), y=rng.uniform(-span, span),
                   up=rng.uniform(0.0, 40.0),
                   yaw_deg=rng.uniform(-180.0, 180.0))
        # Deliberately unreasonable: 3x the speed cap, both signs, sometimes zero.
        a = Action4D(vx=rng.uniform(-12, 12), vy=rng.uniform(-12, 12),
                     vz_up=rng.uniform(-6, 6), yaw_rate=rng.uniform(-2, 2))
        d = shield.filter(st, a)
        touched += bool(d.touched)
        braked += bool(d.braked)
        assert _finite(d.emitted), f"non-finite emitted from {st} {a}"
        left = [v for v in shield._check(st, d.emitted)
                if _prio(shield, v.rule_id) == "P0"]
        if left:
            escapes.append((st, a, d.emitted, [v.rule_id for v in left]))
    return escapes, touched, braked


def test_p0_escape_is_zero_over_random_states_and_actions():
    """20k adversarial pairs, no fence map: nothing the Shield emits may violate P0."""
    s = _shield()
    escapes, touched, braked = _fuzz(s)
    assert touched > 0, "fuzz never triggered the Shield -- the test proves nothing"
    assert not escapes, (
        f"{len(escapes)} P0 escapes, first: state={escapes[0][0]} "
        f"raw={escapes[0][1]} emitted={escapes[0][2]} rules={escapes[0][3]}"
    )


def _fuzz_legal_starts(shield, n=3000, seed=20260811, span=60.0):
    """As _fuzz, but only from states the aircraft is ALLOWED to be in.

    Sampling uniformly over a dense city puts ~93% of draws inside a building or
    a zone, which are states a flight can only reach by being placed there. The
    KPI claim is about the operating envelope, so the envelope is what gets
    sampled; the excluded states get their own test below rather than being
    quietly dropped.
    """
    rng = random.Random(seed)
    escapes, drawn, kept = [], 0, 0
    while kept < n and drawn < n * 60:
        drawn += 1
        st = State(x=rng.uniform(-span, span), y=rng.uniform(-span, span),
                   up=rng.uniform(0.0, 40.0), yaw_deg=rng.uniform(-180.0, 180.0))
        if shield._check(st, Action4D()):
            continue                                    # not a legal place to be
        kept += 1
        a = Action4D(vx=rng.uniform(-12, 12), vy=rng.uniform(-12, 12),
                     vz_up=rng.uniform(-6, 6), yaw_rate=rng.uniform(-2, 2))
        d = shield.filter(st, a)
        assert _finite(d.emitted)
        left = [v for v in shield._check(st, d.emitted)
                if _prio(shield, v.rule_id) == "P0"]
        if left:
            escapes.append((st, a, d.emitted, [v.rule_id for v in left]))
    return escapes, kept


def test_p0_escape_is_zero_from_legal_states_with_clearance_live():
    """The KPI, with obstacle_clearance active: start legal, never escape."""
    if not CITYMAP.exists():
        return                      # map is a build artefact, not committed
    s = _shield(ROOT / "policies" / "urban_clearance.yaml", with_map=True)
    escapes, kept = _fuzz_legal_starts(s)
    assert kept >= 1000, f"only {kept} legal states sampled -- too few to claim anything"
    assert not escapes, (
        f"{len(escapes)} P0 escapes from LEGAL states, first: {escapes[0][0]} "
        f"-> {escapes[0][2]} rules={escapes[0][3]}"
    )


def test_wedged_in_overlapping_zones_moves_instead_of_freezing_KNOWN_LIMIT():
    """Placed inside TWO overlapping no-fly zones, the output still violates one.

    Found by the fuzzer: 13 of 6000 uniform draws land in the strip where
    nfz-edge-1 (x 32..44, y 5..17) and nfz-edge-2 (x 36..48, y -6..6) overlap.
    A clean escape exists in principle -- far east or far west leaves both -- but
    every intermediate sample of the 3 s forecast is still inside one zone, so no
    heading can be CERTIFIED clean and the Shield falls back to its documented
    best-effort branch ("stopping is itself illegal here").

    Not reachable by flying: the Shield stops the aircraft entering in the first
    place, which the legal-start fuzz above confirms over 3000 states. It IS
    reachable by hot_apply() dropping a new zone on top of the aircraft, so it is
    a real case and not a fuzzer artefact.

    What is asserted here is the property that actually matters in that corner:
    the aircraft KEEPS MOVING. Freezing inside a zone re-raises the same
    violation every tick forever, which is the deadlock this whole design avoids.
    All 13 cases moved.
    """
    s = _shield(ROOT / "policies" / "urban_clearance.yaml")
    st = State(x=39.0, y=5.5, up=20.0)                 # inside both zones
    assert s._check(st, Action4D()), "premise: hovering here must be illegal"

    d = s.filter(st, Action4D(vx=-3.0, vy=-1.0))       # asking to leave, westward
    assert _finite(d.emitted)
    assert math.hypot(d.emitted.vx, d.emitted.vy) > 0.1, (
        "frozen inside overlapping zones -- the violation can now never clear"
    )


# ---------------------------------------------------------------- 3. HOSTILE
#
# The detector CAN fail. servo() divides by a box width, and a zero-width or
# absent box is one arithmetic slip away from NaN. If NaN reaches the autopilot
# the aircraft does something undefined, so the Shield is the last place to stop
# it -- and nothing tested that it does.

def _hostile_actions():
    nan, inf = float("nan"), float("inf")
    return [
        ("nan vx", Action4D(vx=nan)),
        ("nan all", Action4D(vx=nan, vy=nan, vz_up=nan, yaw_rate=nan)),
        ("+inf vy", Action4D(vy=inf)),
        ("-inf climb", Action4D(vz_up=-inf)),
        ("nan yaw only", Action4D(vx=1.0, yaw_rate=nan)),
        ("absurd", Action4D(vx=1e12, vy=-1e12, vz_up=1e9, yaw_rate=1e6)),
    ]


def test_non_finite_actions_never_reach_the_output():
    """NaN or infinity in must not be NaN or infinity out, and must not crash."""
    s = _shield()
    st = State(x=-20, y=-20, up=4)
    bad = []
    for name, a in _hostile_actions():
        try:
            d = s.filter(st, a)
        except Exception as e:                      # noqa: BLE001 - any crash is a fail
            bad.append(f"{name}: raised {type(e).__name__}: {e}")
            continue
        if not _finite(d.emitted):
            bad.append(f"{name}: emitted {d.emitted}")
    assert not bad, "non-finite or crashing inputs survived:\n  " + "\n  ".join(bad)


def test_absurd_but_finite_action_is_clamped_to_the_envelope():
    """1e12 m/s is finite, so it must be clamped rather than merely tolerated."""
    s = _shield()
    d = s.filter(State(x=-20, y=-20, up=4), Action4D(vx=1e12, vy=-1e12))
    assert math.hypot(d.emitted.vx, d.emitted.vy) <= 4.0 + 1e-6


# --------------------------------------------------------------- 4. COVERAGE
#
# A rule that never fires is indistinguishable from a rule that is absent. Group
# 1 found one of those by hand; this finds them generically.

def test_every_constraint_type_in_the_demo_policy_can_actually_fire():
    """One triggering input per constraint TYPE present in the policy."""
    s = _shield()
    probes = {
        PolygonFence: (State(x=5, y=15, up=4), Action4D(vx=3.0)),
        AltitudeEnvelope: (State(x=-20, y=-20, up=5.8), Action4D(vz_up=2.0)),
        KinematicEnvelope: (State(x=-20, y=-20, up=4), Action4D(vx=9.0)),
    }
    present = {type(c) for c in s.policy.constraints}
    unfired = []
    for cls in present:
        if cls not in probes:
            continue                                # covered by test_clearance.py
        st, a = probes[cls]
        fired = {type(c) for c in s.policy.constraints
                 for v in s._check(st, a) if v.rule_id == c.id}
        if cls not in fired:
            unfired.append(cls.__name__)
    assert not unfired, f"constraint types that never fire: {unfired}"


def test_obstacle_clearance_fires_once_it_has_a_map():
    """The one rule inert by design without a map — prove it wakes up with one."""
    if not CITYMAP.exists():
        return
    import city_planner
    cm = city_planner.load_occ(str(CITYMAP))
    s = _shield(ROOT / "policies" / "urban_clearance.yaml", with_map=True)
    clr = [c for c in s.policy.constraints if isinstance(c, ObstacleClearance)]
    assert clr, "urban_clearance.yaml no longer carries an obstacle_clearance rule"

    # Walk the map for a cell that is genuinely blocked, then stand next to it.
    occ, res, ox, oy = cm["occ"], cm["res"], cm["ox"], cm["oy"]
    hit = None
    for i in range(occ.shape[0]):
        for j in range(occ.shape[1]):
            if occ[i, j]:
                hit = (ox + i * res, oy + j * res)
                break
        if hit:
            break
    assert hit, "occupancy map has no blocked cells at all"

    fired = False
    for d_off in (1.0, 2.0, 3.0):                   # inside any sane clearance ring
        st = State(x=hit[0] + d_off, y=hit[1], up=20.0)
        if any(v.rule_id == clr[0].id for v in s._check(st, Action4D(vx=-2.0))):
            fired = True
            break
    assert fired, "obstacle_clearance never fired beside a mapped building"


# --------------------------------------------------------------- 5. CONFLICT

def test_three_rules_violated_at_once_still_yields_a_legal_action():
    """Below the floor, overspeed, and driving into the fence, simultaneously."""
    s = _shield()
    st = State(x=5, y=15, up=1.0)                   # under the 2 m floor
    d = s.filter(st, Action4D(vx=11.0, vz_up=-3.0))  # fast, descending, into the NFZ
    assert d.touched
    assert _finite(d.emitted)
    assert not [v for v in s._check(st, d.emitted) if _prio(s, v.rule_id) == "P0"]


def test_repair_converges_rather_than_oscillating():
    """Feeding the output back in must reach a fixed point, not cycle forever."""
    s = _shield()
    st = State(x=5, y=15, up=1.0)
    a = Action4D(vx=11.0, vz_up=-3.0)
    seen = []
    for _ in range(12):
        a = s.filter(st, a).emitted
        key = (round(a.vx, 6), round(a.vy, 6), round(a.vz_up, 6))
        if key in seen:
            break
        seen.append(key)
    else:
        raise AssertionError(f"no fixed point after 12 passes: {seen}")
    assert not [v for v in s._check(st, a) if _prio(s, v.rule_id) == "P0"]


# --------------------------------------------------------------- 6. POLICIES

def test_all_follow_policies_load_and_hash_distinctly():
    """A copy-paste slip between the three would silently demo the wrong rule."""
    hashes = {}
    for name, path in FOLLOW.items():
        assert path.exists(), f"{name}: {path} missing"
        pol = load_policy(path)
        hashes[name] = pol.policy_hash
    assert len(set(hashes.values())) == len(hashes), f"duplicate policies: {hashes}"


def test_full_fence_policy_really_blocks_the_whole_corridor():
    """follow_car_nfz.yaml claims 'no way around it'. Verify, do not trust."""
    s = _shield(FOLLOW["full-fence"])
    fences = [c for c in s.policy.constraints if isinstance(c, PolygonFence)]
    assert len(fences) == 1
    ys = [v.y for v in fences[0].vertices]
    xs = [v.x for v in fences[0].vertices]
    # Corridor is x 30..50 with >=5 m building clearance; the fence must span it.
    assert min(xs) <= 30 and max(xs) >= 50, (
        f"fence x {min(xs)}..{max(xs)} does not span the 30..50 corridor -- the "
        "aircraft can track the car around it, which is the failure this policy "
        "was rewritten to avoid (an earlier x 30..40 fence gave 11 interventions)"
    )
    assert min(ys) < max(ys)


def test_gap_fence_leaves_a_flyable_gap_BEFORE_flying_it():
    """follow_car_gap.yaml has never been flown. Prove the gap exists first.

    The policy comment claims 7 m of legal road survives east of the fence. If
    that is wrong -- fence too wide, or the altitude band unreachable -- the
    flight is doomed before takeoff and would read as a guardrail failure rather
    than a policy bug. Cheaper to check here.
    """
    s = _shield(FOLLOW["gap-fence"])
    fence = [c for c in s.policy.constraints if isinstance(c, PolygonFence)][0]
    alt = [c for c in s.policy.constraints if isinstance(c, AltitudeEnvelope)][0]
    mid_alt = 0.5 * (alt.alt_min_m + alt.alt_max_m)
    y_mid = 0.5 * (min(v.y for v in fence.vertices)
                   + max(v.y for v in fence.vertices))

    # Hovering is legal only where every rule is satisfied, so a zero action is
    # the cleanest probe for "may the aircraft simply BE here".
    legal = [x for x in range(26, 51)
             if not s._check(State(x=float(x), y=y_mid, up=mid_alt), Action4D())]
    assert legal, (
        f"no legal x in 26..50 at y={y_mid}, alt={mid_alt} -- the gap does not "
        "exist and the flight would fail for policy reasons, not safety ones"
    )
    # The LONGEST CONTIGUOUS run, not the span.
    #
    # `max(legal) - min(legal)` measures the span of a set that need not be
    # contiguous. It is contiguous today (fence x 26..42, legal x 43..50), so
    # the old form happened to give the right answer. Move the fence into the
    # middle of the corridor - x 30..40, which this file's own
    # `test_full_fence_policy_really_blocks_the_whole_corridor` records as an
    # earlier configuration - and legal becomes [26..29] + [41..50]: a span of
    # 24 that passes a ">= 5 m" check while the western gap is 4 m wide and the
    # eastern one is on the far side of the zone. Two slivers on opposite sides
    # of a fence are not a gap an aircraft can fly through.
    runs, run = [], [legal[0]]
    for x in legal[1:]:
        if x == run[-1] + 1:
            run.append(x)
        else:
            runs.append(run)
            run = [x]
    runs.append(run)
    best = max(runs, key=len)
    width = best[-1] - best[0]
    assert width >= 5.0, (
        f"widest contiguous gap only {width} m (x {best[0]}..{best[-1]}); "
        f"all legal x: {legal}")
    # And the blocked side must still be blocked, or the fence does nothing.
    x_in = 0.5 * (min(v.x for v in fence.vertices) + max(v.x for v in fence.vertices))
    assert s._check(State(x=x_in, y=y_mid, up=mid_alt), Action4D()), (
        "the fence interior is legal -- the zone is not being enforced at all"
    )


def test_follow_policies_hold_the_altitude_band_they_advertise():
    """Each band must reject both a too-low and a too-high hover."""
    bad = []
    for name, path in FOLLOW.items():
        s = _shield(path)
        alts = [c for c in s.policy.constraints if isinstance(c, AltitudeEnvelope)]
        if not alts:
            bad.append(f"{name}: no altitude_envelope")
            continue
        a = alts[0]
        if not s._check(State(x=0, y=-40, up=a.alt_min_m - 2.0), Action4D()):
            bad.append(f"{name}: {a.alt_min_m - 2.0} m accepted, floor is {a.alt_min_m}")
        if not s._check(State(x=0, y=-40, up=a.alt_max_m + 2.0), Action4D()):
            bad.append(f"{name}: {a.alt_max_m + 2.0} m accepted, ceiling is {a.alt_max_m}")
    assert not bad, "altitude bands not enforced:\n  " + "\n  ".join(bad)


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
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                      # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
