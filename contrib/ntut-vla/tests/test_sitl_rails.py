"""The ArduPilot rails, checked without a simulator.

Run either way:
    pytest tests/test_sitl_rails.py -v
    python tests/test_sitl_rails.py

WHY THIS FILE EXISTS

Nothing imported `sitl/` until 2026-08-28, and it cost exactly what you would
expect. Commit `01e22ca` made `AuditLogger.policy_hash` a read-only property so
that a hot-applied rule restamps the hash. Seven `demo/*.py` call sites were
updated. The two in `sitl/` were not, and both assign to that property right
after `hot_apply`:

    sitl/run_sitl_demo.py:198      audit.policy_hash = policy.policy_hash
    sitl/ros2_shield_node.py:206   self.audit.policy_hash = ...

So every `--dynamic` run died at t = 8 s with `AttributeError`, on the rail that
carries the grant's contractual KPI figures, and nothing noticed. The change was
verified against 242 tests; none of them touched this code.

These tests need no autopilot, no ROS, no MAVLink. They exercise the two things
the rails do that the AirSim demos do not: hot-apply a fence mid-flight, and
record the flown action's violations for a shield-OFF control run.

`ros2_shield_node.py` cannot be imported here (it needs `rclpy`, which is only
in the WSL ROS venv), so its equivalents are checked by reading the source. That
is weaker than an import, and it is what is available on this host.
"""
import ast
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import load_policy                                    # noqa: E402
from guardrail.audit import AuditLogger                              # noqa: E402
from guardrail.models import Action4D, PolygonFence, State, XY       # noqa: E402
from guardrail.shield import Shield                                  # noqa: E402

SITL = ROOT / "sitl"
POLICY = ROOT / "policies" / "sim_demo_policy.yaml"


def _fresh_audit():
    """Construct the logger the way the SITL rails construct it."""
    policy = load_policy(POLICY)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    path = Path(tempfile.mkdtemp()) / "audit.jsonl"
    return policy, shield, AuditLogger(path, policy), path


def test_a_hot_applied_fence_restamps_the_audit_hash():
    """The exact sequence the --dynamic runs perform at t = 8 s."""
    policy, shield, audit, path = _fresh_audit()
    before = policy.policy_hash

    audit.log(1, shield.filter(State(x=-20, y=-20, up=4), Action4D(vx=9)))
    shield.hot_apply(PolygonFence(
        id="nfz-dynamic", type="polygon_fence",
        vertices=[XY(x=-25, y=-25), XY(x=-15, y=-25),
                  XY(x=-15, y=-15), XY(x=-25, y=-15)], margin_m=1.0))
    audit.log(2, shield.filter(State(x=-20, y=-20, up=4), Action4D(vx=1)))

    recs = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines()]
    assert len(recs) == 2, recs
    assert recs[0]["policy_hash"] == before
    assert recs[1]["policy_hash"] != before, (
        "a hot-applied rule must restamp the hash; the audit trail is supposed "
        "to show which rules were in force for each record")


def test_neither_rail_assigns_to_the_read_only_policy_hash():
    """The regression itself, caught by reading the source.

    `policy_hash` is a property with no setter, so an assignment is an
    AttributeError at run time and completely invisible to import-time checks.
    """
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign):
                continue
            for tgt in node.targets:
                if isinstance(tgt, ast.Attribute) and tgt.attr == "policy_hash":
                    raise AssertionError(
                        f"{name}:{node.lineno} assigns to AuditLogger.policy_hash, "
                        f"which is read-only. Pass the policy object to "
                        f"AuditLogger instead and delete the assignment.")


def test_both_rails_construct_the_logger_with_the_policy_not_the_hash():
    """Passing the hash string is legal but silently loses hot-apply tracking."""
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        assert "AuditLogger(out / \"audit.jsonl\", policy)" in src \
            or "AuditLogger(out / \"audit.jsonl\", self.policy)" in src, (
                f"{name} must hand AuditLogger the POLICY, so the hash is read "
                f"live and a mid-flight rule change is visible in the audit log")


def test_both_rails_record_the_flown_actions_violations():
    """Without this the KPI falls back to an inference that cannot fail.

    Recomputing the 2026-08-24 artefacts with the current kpi.py reported
    `p0_ticks_not_measurable` equal to the FULL P0 tick count on every SITL run
    - 132 of 132 on `sitl_shield_on`. The headline "escape rate 0.0 on the real
    ArduPilot rail" was inferred, never measured.
    """
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        assert '"emitted_violations"' in src, (
            f"{name} does not log emitted_violations, so guardrail/kpi.py cannot "
            f"MEASURE a P0 escape and marks every P0 tick unmeasurable")
        assert "decision.emitted_violations if" in src, (
            f"{name} must pick the list by which action actually flew: the "
            f"re-check when the shield is ON, the violations on `raw` when it "
            f"is OFF - otherwise the control run scores clean and the A/B dies")


def test_the_control_arm_still_earns_a_nonzero_escape_rate():
    """The A/B only means something if shield-OFF can fail.

    Simulates the row-building both rails do, with the shield off: `raw` flies
    unmodified, so the violations found on `raw` are the flown action's.
    """
    from guardrail import kpi as K
    policy = load_policy(POLICY)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    st = State(x=27.0, y=10.0, up=6.0)         # inside sim_demo_policy's zone
    rows = []
    for tick in range(40):
        raw = Action4D(vx=4.0)
        d = shield.filter(st, raw)
        rows.append({
            "t": tick * 0.1, "tick": tick,
            "raw": raw.model_dump(), "emitted": raw.model_dump(),
            "violations": [v.model_dump() for v in d.violations],
            # shield OFF: `raw` flew, so its violations are the flown ones
            "emitted_violations": [v.model_dump() for v in d.violations],
            "repairs": [], "braked": False,
        })
    res = K.compute(rows, K.rule_priorities(policy), {})
    assert res["p0_ticks_not_measurable"] == 0, (
        "the control arm must be MEASURED, not inferred")
    assert res["p0_violation_escape_rate"] > 0.0, (
        "an unguarded flight straight into a no-fly zone must score escapes; "
        "if this is zero the A/B proves nothing")


def test_the_shielded_arm_is_measured_and_clean():
    from guardrail import kpi as K
    policy = load_policy(POLICY)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    st = State(x=27.0, y=10.0, up=6.0)
    rows = []
    for tick in range(40):
        raw = Action4D(vx=4.0)
        d = shield.filter(st, raw)
        rows.append({
            "t": tick * 0.1, "tick": tick,
            "raw": raw.model_dump(), "emitted": d.emitted.model_dump(),
            "violations": [v.model_dump() for v in d.violations],
            "emitted_violations": [v.model_dump() for v in d.emitted_violations],
            "repairs": [r.model_dump() for r in d.repairs],
            "braked": d.braked,
        })
    res = K.compute(rows, K.rule_priorities(policy), {})
    assert res["p0_ticks_not_measurable"] == 0
    assert res["p0_violation_escape_rate"] == 0.0, (
        f"the Shield let a P0 violation through: {res}")


def test_the_ros2_rail_reads_sim_speedup_instead_of_asserting_it():
    """build_manifest's docstring: "a hand-written 1.0 is exactly the number a
    broken run would also carry."

    The perverse part of the old code: the three runs that PASSED is_kpi_grade()
    asserted this value, while the pymavlink rail that reads it was refused for
    its topology.
    """
    src = (SITL / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "sim_speedup=1.0" not in src, (
        "ros2_shield_node.py still asserts sim_speedup=1.0")
    assert "def read_sim_speedup" in src and "sim_speedup=speedup" in src


def test_both_rails_write_the_kpi_grade_into_the_artefact():
    """A reader holding kpi.json must be able to tell whether its numbers are
    quotable as contractual figures."""
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        assert '"kpi_grade"' in src, f"{name} does not record the grade in kpi.json"


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
