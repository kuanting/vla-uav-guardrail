"""The monitor's output must not move.

Run either way:
    pytest tests/test_check_contract.py -v
    python tests/test_check_contract.py

`Shield._check` is what makes the grant's hard KPI true: an escape is a P0
violation still present in the action that was flown, and `_check` is the thing
that decides "still present". Every repair operator, the fixed-point loop's exit
condition, the final P0 guard and `emitted_violations` all read from it.

So when `_check` is refactored - as it was on 2026-08-26, split into per-rule
predicates so that monitors and repairs stop keeping two copies of the same
trend logic - "the tests still pass" is not evidence enough. The suite exercises
maybe a hundred hand-chosen states; the monitor has to answer identically on all
of them AND on everything nobody thought to write down.

This pins it: 1200 seeded (state, action) pairs per policy, across every policy
in `policies/`, hashed. The hash covers `rule_id`, `category`, `predicted_at_s`
and `detail` - the prose too, because an operator reads `detail` off the audit
log and a changed message is a changed artefact.

If this fails after a refactor that was meant to preserve behaviour, the
refactor is wrong. If a change to the monitor is INTENDED, regenerate:

    python tests/test_check_contract.py --regenerate

and the diff on the digest file is then a deliberate, reviewable record that the
safety monitor's behaviour changed.
"""
import hashlib
import json
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import load_policy                                   # noqa: E402
from guardrail.models import (Action4D, ObstacleClearance,          # noqa: E402
                              State, SubjectStandoff)
from guardrail.shield import Shield                                 # noqa: E402

FIXTURE = ROOT / "tests" / "fixtures" / "check_baseline_digests.json"
CITYMAP = ROOT / "demo" / "out" / "citymap" / "occ_day.npz"


def _obstacle_map():
    if not CITYMAP.is_file():
        return None
    import city_planner
    cm = city_planner.load_occ(str(CITYMAP))
    return {"occ": cm["occ"], "res": cm["res"], "ox": cm["ox"], "oy": cm["oy"]}


def _lines_for(name: str, smap, n: int, seed: int) -> list[str]:
    """The sampling is the fixture. Change it and every digest changes."""
    pol = load_policy(ROOT / "policies" / name)
    sh = Shield(pol, lookahead_s=3.0, dt=0.5,
                obstacle_map=smap if pol.by_type(ObstacleClearance) else None)
    rng = random.Random(seed)
    has_so = bool(pol.by_type(SubjectStandoff))
    out = []
    for i in range(n):
        if has_so and i % 2 == 0:
            # The subject goes NEAR the aircraft on purpose. Sampling both
            # uniformly over the map puts them ~50 m apart and no standoff ring
            # ever binds, so the rule this fixture most needs to cover would be
            # covered by nothing at all.
            sx, sy = rng.uniform(-60, 60), rng.uniform(-60, 60)
            st = State(x=sx + rng.uniform(-14, 14), y=sy + rng.uniform(-14, 14),
                       up=rng.uniform(0, 30))
            sh.set_subject(sx, sy, rng.choice(["pedestrian", "car", "thing"]))
        else:
            sh.set_subject(None)
            st = State(x=rng.uniform(-70, 70), y=rng.uniform(-70, 70),
                       up=rng.uniform(0, 30))
        a = Action4D(vx=rng.uniform(-8, 8), vy=rng.uniform(-8, 8),
                     vz_up=rng.uniform(-4, 4), yaw_rate=rng.uniform(-3, 3))
        vs = sh._check(st, a)
        out.append(json.dumps([
            [round(st.x, 9), round(st.y, 9), round(st.up, 9)],
            [round(a.vx, 9), round(a.vy, 9), round(a.vz_up, 9), round(a.yaw_rate, 9)],
            list(sh._subject) if sh._subject else None, sh._subject_class,
            [[v.rule_id, v.category, round(v.predicted_at_s or 0.0, 6), v.detail]
             for v in vs],
        ], separators=(",", ":")))
    return out


def _digests(spec) -> dict:
    smap = _obstacle_map()
    return {name: hashlib.sha256(
                "\n".join(_lines_for(name, smap, spec["n_per_policy"],
                                     spec["seed"])).encode()).hexdigest()[:32]
            for name in sorted(spec["digests"])}


def test_the_monitor_answers_exactly_as_it_did_before_the_refactor():
    spec = json.loads(FIXTURE.read_text(encoding="utf-8"))
    if _obstacle_map() is None:
        return                       # clearance rules inert; digests would differ
    got = _digests(spec)
    want = spec["digests"]
    moved = sorted(k for k in want if got.get(k) != want[k])
    assert not moved, (
        f"Shield._check changed its answer for {len(moved)} of {len(want)} "
        f"policies: {moved[:6]}. If that was intended, regenerate the fixture "
        f"with `python tests/test_check_contract.py --regenerate` so the change "
        f"is reviewable; if not, the refactor is wrong.")


def test_the_fixture_actually_covers_every_rule_type():
    """A digest over samples that never trip a rule proves nothing about it."""
    spec = json.loads(FIXTURE.read_text(encoding="utf-8"))
    smap = _obstacle_map()
    if smap is None:
        return
    seen = set()
    for name in sorted(spec["digests"]):
        for line in _lines_for(name, smap, 200, spec["seed"]):
            for v in json.loads(line)[4]:
                seen.add(v[1])                    # category
    for need in ("kinematic", "altitude", "geofence", "clearance", "standoff"):
        assert need in seen, f"no sample exercises a {need} rule; fixture is blind to it"


if __name__ == "__main__":
    if "--regenerate" in sys.argv:
        spec = json.loads(FIXTURE.read_text(encoding="utf-8"))
        spec["digests"] = _digests(spec)
        FIXTURE.write_text(json.dumps(spec, indent=2), encoding="utf-8")
        print(f"regenerated {len(spec['digests'])} digests -> {FIXTURE}")
        sys.exit(0)

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
