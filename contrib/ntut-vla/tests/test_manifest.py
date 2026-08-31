"""The WP4 determinism manifest and the four locked acceptance KPIs.

Run either way:
    pytest tests/test_manifest.py -v
    python tests/test_manifest.py

The grant allows contractual KPI numbers only from a run that carries a six-field
determinism manifest at `sim_speedup=1.0`. Our flights carried none of it, so on
the grant's own terms nothing measured here was reportable.

The most important test in this file is the one that makes the hard KPI FAIL. A
P0-escape counter that only ever sees compliant logs proves nothing: it would
report a perfect score for a broken Shield. So a log where a P0 violation is
detected and then flown anyway must come out non-zero.
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import kpi as K                                        # noqa: E402
from guardrail.manifest import (MAX_HEADING_ERR_DEG, MIN_DET_HZ,      # noqa: E402
                                TOPOLOGY_CANONICAL_HIL,
                                TOPOLOGY_PROJECTAIRSIM, build_manifest,
                                is_kpi_grade, sim_speedup_from_scene)

SCENE = ROOT / "demo" / "pas_config" / "scene_semantic.jsonc"
GOOD_METRICS = {"det_hz": 4.5, "start_heading_err_deg": 1.5}


def _man(**kw):
    base = dict(policy_hash="sha256:deadbeefdeadbeef",
                model_id="google/owlvit-base-patch32", seed=42,
                scene_path=str(SCENE))
    base.update(kw)
    return build_manifest(**base)


# ------------------------------------------------------------------ manifest

def test_the_manifest_has_exactly_the_six_grant_fields():
    m = _man()
    assert set(m) == {"code_revision", "vla_model_hash", "policy_hash",
                      "random_seed", "sim_speedup", "topology"}, sorted(m)


def test_the_same_inputs_give_a_byte_identical_manifest():
    a, b = _man(), _man()
    assert json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)


def test_changing_any_field_changes_the_manifest():
    base = json.dumps(_man(), sort_keys=True)
    assert json.dumps(_man(seed=43), sort_keys=True) != base
    assert json.dumps(_man(policy_hash="sha256:0"), sort_keys=True) != base
    assert json.dumps(_man(model_id="other/model"), sort_keys=True) != base


def test_sim_speedup_is_derived_from_the_scene_not_asserted():
    """A hardcoded 1.0 would defeat the requirement it exists to enforce."""
    assert sim_speedup_from_scene(SCENE) == 1.0


def test_a_fast_clock_is_detected_and_disqualifies_the_run(tmp=Path("_t_fastclock.jsonc")):
    """`sim_speedup=1.0` is mandatory for a KPI-bearing run, so a scene running
    faster than real time has to be caught from the config."""
    try:
        tmp.write_text(json.dumps({
            "id": "fast", "clock": {"type": "steppable", "step-ns": 3000000,
                                    "real-time-update-rate": 12000000}}),
            encoding="utf-8")
        assert sim_speedup_from_scene(tmp) == 4.0
        ok, why = is_kpi_grade(_man(scene_path=str(tmp)), GOOD_METRICS)
        assert not ok and any("sim_speedup" in r for r in why), why
    finally:
        tmp.unlink(missing_ok=True)


def test_an_unreadable_scene_is_unresolved_not_assumed_real_time():
    m = _man(scene_path="does/not/exist.jsonc")
    assert m["sim_speedup"] == "unresolved"
    ok, why = is_kpi_grade(m, GOOD_METRICS)
    assert not ok and any("sim_speedup" in r for r in why)


def test_our_rail_may_not_claim_the_canonical_hil_topology():
    """The field exists so the difference cannot be blurred. Our flights are
    Project AirSim; the grant's KPI topology is ArduPilot SITL + MAVROS.

    Tested WITH good evidence, which is the whole point. Without evidence this
    was just a second copy of
    `test_canonical_hil_cannot_be_claimed_without_evidence`, and its failure
    message - "a Project AirSim run was allowed to claim canonical-hil" - named
    a property nothing actually enforced. Supplying `hil_evidence` used to be
    enough to get our own Project AirSim scene stamped `canonical-hil`, the one
    label the grant reads as KPI-grade.

    The scene file is the tell: the canonical rail is ArduPilot SITL and has no
    simulator scene.
    """
    try:
        _man(topology=TOPOLOGY_CANONICAL_HIL, hil_evidence=GOOD_HIL)
    except ValueError as e:
        assert "scene" in str(e).lower(), e
    else:
        raise AssertionError("a Project AirSim run was allowed to claim canonical-hil")


# --------------------------------------------------------------- kpi grading

def test_the_project_airsim_rail_is_never_kpi_grade():
    """Honest ceiling on everything this repo can currently claim."""
    ok, why = is_kpi_grade(_man(), GOOD_METRICS)
    assert not ok
    assert any(TOPOLOGY_PROJECTAIRSIM in r for r in why), why


def test_a_stalled_detector_disqualifies_the_run():
    """The real failure: 29 inferences at 0.52 Hz reported det_hit_rate 1.000 on a
    flight that tracked for 13.6% of ticks."""
    ok, why = is_kpi_grade(_man(), {"det_hz": 0.52, "start_heading_err_deg": 1.0})
    assert not ok and any("Hz" in r for r in why), why


def test_a_missed_start_heading_disqualifies_the_run():
    """The real failure: the heading was never commanded, and swung the traffic hit
    rate 0.740 -> 0.331 on identical configuration."""
    ok, why = is_kpi_grade(_man(), {"det_hz": 4.5, "start_heading_err_deg": 52.1})
    assert not ok and any("heading" in r for r in why), why
    assert MAX_HEADING_ERR_DEG < 52.1 and MIN_DET_HZ > 0.52


def test_an_unresolved_field_disqualifies_the_run():
    ok, why = is_kpi_grade(_man(policy_hash=""), GOOD_METRICS)
    assert not ok and any("policy_hash" in r for r in why), why


# ------------------------------------------------------------- the hard KPI

def _tick(rule="nfz-route", repaired=False, braked=False, moved=False):
    raw = {"vx": 1.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}
    em = dict(raw)
    if moved:
        em["vy"] = 0.7
    return {"violations": [{"rule_id": rule, "category": "geofence"}],
            "repairs": ([{"operator": "GeofenceProject"}] if repaired else []),
            "braked": braked, "raw": raw, "emitted": em}


PRIOS = {"nfz-route": "P0", "kin-caps": "P1"}


def test_a_p0_flown_anyway_IS_counted_as_an_escape():
    """THE TEST THAT MATTERS. A counter that only sees compliant logs would report
    a perfect score for a broken Shield."""
    k = K.compute([_tick()] * 10, PRIOS)
    assert k["p0_escapes"] == 10
    assert k["p0_violation_escape_rate"] == 1.0
    assert k["outcome"] == "fail" and k["mission_success"] is False


def test_a_repair_that_did_not_fix_it_IS_an_escape():
    """The case the old accounting could not see, and the reason it changed.

    A repair ran, so `repairs` is non-empty and the action changed - the old
    inference (`emitted_differs or braked or repairs`) therefore scored this as
    the Shield working. But the re-check on the flown action still names a P0
    rule, which is the definition of an escape. Because every Shield branch
    that raises a violation also appends a Repair, this shape is the ONLY way
    a real escape can occur, and it was exactly the one being missed.
    """
    row = _tick(repaired=True, moved=True)
    row["emitted_violations"] = [{"rule_id": "nfz-route", "category": "geofence"}]
    k = K.compute([row] * 10, PRIOS)
    assert k["p0_escapes"] == 10, k
    assert k["p0_violation_escape_rate"] == 1.0
    assert k["outcome"] == "fail" and k["mission_success"] is False
    assert k["failsafe_trigger_correctness"] == 0.0


def test_a_measured_clean_repair_is_not_an_escape_and_is_marked_measured():
    """The good case, recorded rather than assumed."""
    row = _tick(repaired=True, moved=True)
    row["emitted_violations"] = []
    k = K.compute([row] * 10, PRIOS)
    assert k["p0_escapes"] == 0
    assert k["p0_ticks_not_measurable"] == 0, "these rows carry the re-check"


def test_a_log_without_the_recheck_is_flagged_as_not_measurable():
    """Old artefacts still score, but must not look like measured zeros."""
    k = K.compute([_tick(repaired=True, moved=True)] * 10, PRIOS)
    assert k["p0_escapes"] == 0
    assert k["p0_ticks_not_measurable"] == 10
    assert k["mission_success"] is False, (
        "a run whose escape rate was never measured must not claim success")


def test_a_p0_that_was_repaired_is_not_an_escape():
    """Detected-and-repaired is the Shield working, and must not count against it."""
    k = K.compute([_tick(repaired=True, moved=True)] * 10, PRIOS)
    assert k["p0_escapes"] == 0 and k["p0_violation_escape_rate"] == 0.0
    assert k["repair_count"] == 10
    assert k["failsafe_trigger_correctness"] == 1.0


def test_braking_also_counts_as_acting_on_a_p0():
    k = K.compute([_tick(braked=True)] * 5, PRIOS)
    assert k["p0_escapes"] == 0


def test_a_p1_violation_is_not_a_p0_escape():
    k = K.compute([_tick(rule="kin-caps")] * 8, PRIOS)
    assert k["p0_escapes"] == 0
    assert k["violations_by_risk_level"] == {"P1": 8}


def test_an_unknown_rule_is_treated_as_p0():
    """Conservative direction on purpose: a rule the policy does not name must not
    silently downgrade the hard KPI."""
    k = K.compute([_tick(rule="who-knows")] * 3, PRIOS)
    assert k["p0_escapes"] == 3


def test_a_clean_flight_scores_zero_escapes_and_succeeds():
    rows = [{"violations": [], "repairs": [], "raw": {}, "emitted": {}}] * 50
    k = K.compute(rows, PRIOS, {"nfz_s": 0.0, "alt_violation_s": 0.0,
                                "frac_within_30m": 1.0})
    assert k["p0_violation_escape_rate"] == 0.0
    assert k["outcome"] == "success" and k["mission_success"] is True
    assert k["failsafe_trigger_correctness"] is None      # nothing to trigger on


def test_time_inside_a_fence_fails_the_mission_even_with_no_escape():
    rows = [{"violations": [], "repairs": [], "raw": {}, "emitted": {}}] * 20
    k = K.compute(rows, PRIOS, {"nfz_s": 3.2, "alt_violation_s": 0.0})
    assert k["outcome"] == "fail" and k["mission_success"] is False


def test_priorities_come_from_the_policy_constraints_list():
    """An earlier version guessed per-type attribute names, silently returned {},
    and defaulted every rule to P0."""
    from guardrail import load_policy
    prios = K.rule_priorities(load_policy(ROOT / "policies" / "follow_car_nfz.yaml"))
    assert prios, "no priorities resolved from the policy"
    assert set(prios.values()) <= {"P0", "P1", "P2"}
    assert "P0" in prios.values()


# -------------------------------------------------- code_revision provenance

def test_code_revision_names_a_repository_that_holds_the_shield():
    """The field must name a commit a reader could check out to get this code.

    It used to name `kuanting-vla-uav-guardrail` - a repository with no
    guardrail/shield.py in it - because that path was searched first and merely
    resolving was accepted as good enough.
    """
    from guardrail.manifest import CODE_SENTINEL, _tracks_our_code, code_revision
    rev = code_revision()
    if rev == "unversioned":
        return                      # no git available; nothing to assert about
    assert _tracks_our_code(ROOT), \
        f"{ROOT} reports a revision but does not track {CODE_SENTINEL}"


def test_a_repository_without_our_code_is_never_accepted():
    """Even asked for by name. Order alone would not have caught the original
    bug: the failure was accepting a repo without checking it holds the code."""
    from guardrail.manifest import _tracks_our_code, code_revision
    fork = ROOT / "kuanting-vla-uav-guardrail"
    if not (fork / ".git").exists():
        return
    assert not _tracks_our_code(fork), \
        "the fork is expected not to contain guardrail/shield.py"
    asked = code_revision(fork)
    ours = code_revision()
    assert asked == ours, \
        f"asking for the fork returned {asked!r}; it must fall through to {ours!r}"


def test_a_dirty_tree_is_not_kpi_grade():
    """A dirty revision does not describe the code that flew: checking out that
    commit gives you something else."""
    ok, why = is_kpi_grade({**_man(), "code_revision": "abc123abc123-dirty"},
                           GOOD_METRICS)
    assert not ok and any("uncommitted" in r for r in why), why


def test_an_unknown_cleanliness_is_not_kpi_grade():
    ok, why = is_kpi_grade({**_man(), "code_revision": "abc123abc123-unknown"},
                           GOOD_METRICS)
    assert not ok and any("clean" in r for r in why), why


def test_a_clean_revision_raises_no_code_revision_objection():
    """Topology will still object - this rail is not canonical HIL - but the
    revision itself must draw no complaint."""
    ok, why = is_kpi_grade({**_man(), "code_revision": "abc123abc123"},
                           GOOD_METRICS)
    assert not any("code_revision" in r for r in why), why


# ------------------------------------------- canonical HIL needs evidence

GOOD_HIL = {"ros_distro": "jazzy", "mavros_node": "/mavros", "fcu_connected": True}


def test_canonical_hil_cannot_be_claimed_without_evidence():
    """Claiming the canonical topology is claiming KPI-grade eligibility, so it
    may not rest on the caller's word."""
    from guardrail.manifest import TOPOLOGY_CANONICAL_HIL
    for bad in (None, {}, {"ros_distro": "jazzy"}):
        try:
            _man(topology=TOPOLOGY_CANONICAL_HIL, hil_evidence=bad)
        except ValueError:
            continue
        raise AssertionError(f"evidence {bad!r} was accepted")


def test_a_disconnected_flight_controller_is_not_canonical_hil():
    """MAVROS comes up happily with nothing on the other end and publishes
    connected: false forever, so a whole mission can run into the void."""
    from guardrail.manifest import TOPOLOGY_CANONICAL_HIL, check_hil_evidence
    ev = dict(GOOD_HIL, fcu_connected=False)
    assert any("fcu_connected" in m for m in check_hil_evidence(ev))
    try:
        _man(topology=TOPOLOGY_CANONICAL_HIL, hil_evidence=ev)
    except ValueError:
        return
    raise AssertionError("a disconnected FCU was accepted as canonical HIL")


def test_canonical_hil_with_evidence_is_accepted_and_kpi_grade():
    """And when the evidence is there it must actually pass, or the gate can
    never be met and the field is decorative."""
    from guardrail.manifest import TOPOLOGY_CANONICAL_HIL
    # A REAL canonical run: ArduPilot SITL, so no Project AirSim scene file and
    # the speedup read from the flight controller instead of from a scene.
    # This used to lean on _man()'s default scene_path, which is our own
    # Project AirSim scene - the very thing this topology is not.
    m = build_manifest(policy_hash="sha256:deadbeefdeadbeef",
                       model_id="google/owlvit-base-patch32", seed=42,
                       scene_path=None, sim_speedup=1.0,
                       topology=TOPOLOGY_CANONICAL_HIL, hil_evidence=GOOD_HIL)
    assert m["topology"] == TOPOLOGY_CANONICAL_HIL
    ok, why = is_kpi_grade({**m, "code_revision": "abc123abc123"}, GOOD_METRICS)
    assert ok, why


def test_a_code_only_pilot_is_pinned_by_its_source_not_left_unresolved():
    """StubVLA has no weights and no HuggingFace revision, but it is fully
    determined by its source file. Reporting 'unresolved' would wrongly say the
    run cannot be reproduced."""
    from guardrail.manifest import model_hash
    h = model_hash("guardrail.vla_stub.StubVLA")
    assert "@src:" in h, h
    assert "unresolved" not in h
    assert model_hash("guardrail.vla_stub.StubVLA") == h, "must be stable"


def test_an_unknown_model_id_still_says_unresolved():
    from guardrail.manifest import model_hash
    assert "unresolved" in model_hash("no.such.module.Thing")





def test_an_audit_record_after_a_hot_apply_carries_the_new_policy_hash():
    """hot_apply promises it: "every artefact after this instant carries a
    different policy_hash - the audit trail shows exactly which rules were
    active when". AuditLogger snapshotted the string at construction, so it
    did not. Reproduced: a record whose violation was `nfz-hot`, stamped with
    the hash of a policy that did not contain `nfz-hot`."""
    import json as _json
    import tempfile
    from pathlib import Path as _P
    from guardrail import load_policy as _lp
    from guardrail.audit import AuditLogger
    from guardrail.models import Action4D, PolygonFence, State
    from guardrail.shield import Shield

    pol = _lp(ROOT / "policies" / "demo_policy.yaml")
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    path = _P(tempfile.mkdtemp()) / "audit.jsonl"
    al = AuditLogger(path, pol)              # the POLICY, not a snapshot string
    before = pol.policy_hash

    al.log(1, sh.filter(State(x=-20, y=-20, up=4), Action4D(vx=8)))
    sh.hot_apply(PolygonFence(
        id="nfz-hot", type="polygon_fence",
        vertices=[{"x": -25, "y": -25}, {"x": -15, "y": -25},
                  {"x": -15, "y": -15}, {"x": -25, "y": -15}]))
    al.log(2, sh.filter(State(x=-20, y=-20, up=4), Action4D(vx=1)))

    recs = [_json.loads(l) for l in path.read_text(encoding="utf-8").splitlines()]
    assert len(recs) == 2, recs
    assert recs[0]["policy_hash"] == before
    assert recs[1]["policy_hash"] != before, "hot-applied rule did not restamp the hash"
    fired = [v["rule_id"] for v in recs[1]["violations"]]
    assert "nfz-hot" in fired, fired


def test_a_plain_hash_string_still_works_for_callers_that_pass_one():
    """Backward compatibility: the string form is a snapshot and stays one."""
    import tempfile
    from pathlib import Path as _P
    from guardrail import load_policy as _lp
    from guardrail.audit import AuditLogger

    pol = _lp(ROOT / "policies" / "demo_policy.yaml")
    al = AuditLogger(_P(tempfile.mkdtemp()) / "audit.jsonl", pol.policy_hash)
    assert al.policy_hash == pol.policy_hash


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
