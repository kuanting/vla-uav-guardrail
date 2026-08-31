# Guardrail coverage: two defects a flight could not have found

**Date:** 2026-08-10
**Trigger:** the 2026-08-05 review flagged that safety evidence stopped at the
no-fly zone — minimum stand-off, altitude limits and lost-target behaviour had no
test cases.
**Suite:** `tests/test_guardrail_coverage.py`, 15 tests, offline, no simulator.

---

## Why flights were never going to close this gap

A flight **samples** the state space. Ten flights visit a few thousand states
along ten trajectories — and every one of those states is a state a *working*
controller chose to visit. The states that break a safety layer are, almost by
definition, the ones a working controller never picks.

So this suite attacks the Shield directly: 20 000 random state/action pairs, NaN
and infinity injected deliberately, every constraint type probed for whether it
can fire at all.

Two real defects fell out. Neither was reachable from any flight we have flown.

---

## Defect 1 — NaN passed straight through the guardrail

**Severity: high. Fixed.**

`Shield.filter()` began with `violations = self._check(state, raw)` and returned
an untouched passthrough when the list came back empty.

NaN loses **every** comparison. `hypot(nan, 0) > 4.0` is `False`. So an action
carrying a NaN raised zero violations, took the passthrough branch, and reached
the autopilot **byte-identical to the way it arrived**. The guardrail failed
*open* — the one direction it must never fail.

Infinity was no better, and worse in an interesting way: the speed clamp scales
by `cap / hypot(vx, vy)`, and `inf * 0.0` is NaN. A *bounded repair operator*
manufactured the poison from a merely-infinite input.

Measured before the fix, feeding six hostile actions:

```
nan vx        -> emitted vx=nan
nan all       -> emitted vx=nan vy=nan vz_up=nan yaw_rate=nan
+inf vy       -> emitted vy=nan
nan yaw only  -> emitted yaw_rate=nan
```

**This is reachable from a real pilot, not just a fuzzer.** `servo()` sizes
forward speed from the detector's box width; a zero-width box is one division
from NaN, and a detector failing mid-flight is a normal event.

**Fix** (`_sanitise()` in `guardrail/shield.py`, called first thing in
`filter()`): any non-finite channel is forced to `0.0` and a `contract`-category
violation with rule id `action-finite` is raised, so the substitution is
`touched`, audited, and never silent. Fail-safe reading: *a non-finite command
is not a command.*

All 16 existing shield tests and 13 clearance tests still pass.

## Defect 2 — a P1 rule no real action can trip

**Severity: low in effect, high in what it implies. Documented, not fixed.**

`Action4D.yaw_rate` is documented "deg/s" at `guardrail/models.py:36`. Every
producer feeds **rad/s**: `move_by_velocity_async(yaw_is_rate=True)` takes rad/s,
and the upstream contract (`vlaguard_common.frames`) says rad/s. The cap is
`yaw_rate_max_dps: 45.0`, compared against the raw number.

The widest yaw AerialVLA ever emits is ±1.1 rad/s = 63 °/s, which *should* be
clamped. As a raw number `1.1 < 45`, so nothing fires. **The cap has never fired
in any run in this repository.**

What makes this worth writing down is not the dead rule — it is that
`test_yaw_rate_clamped` in `tests/test_shield.py` has been **green the whole
time**. It passes `yaw_rate=90`, a degrees-scale number production never
produces. A test can be green for a rule that is dead in the field.

**Deliberately not fixed here.** A live yaw cap would make the Shield edit
`yaw_rate` for the first time, and the clean separation the follow demo rests on
— heading is the pilot's, track is the guardrail's, verified 0 yaw edits over 419
violation ticks — would no longer hold. That is a decision, not a typo fix.
`test_yaw_cap_is_unreachable_at_flight_scale_KNOWN_DEFECT` asserts the current
behaviour and turns red the day someone changes it, which is the moment to re-run
`experiments/verify_shield_yaw.py`.

---

## The KPI, as a property rather than an anecdote

The grant's hard KPI is **P0 violation escape rate = 0**. A flight demonstrates
that along one trajectory. The fuzz demonstrates it over the state space:

| condition | samples | P0 escapes |
|---|---|---|
| random states, demo policy, no map | 20 000 | **0** |
| **legal** starting states, urban policy, clearance live | 3 000 | **0** |
| illegal starting states (inside a building or a zone) | 5 564 | 13 |

The 13 are not swept under the rug — they have their own test and their own
section below.

## Known limit — wedged inside two overlapping zones

`urban_clearance.yaml` has `nfz-edge-1` (x 32–44, y 5–17) and `nfz-edge-2`
(x 36–48, y −6…6). They overlap in a strip around y 5–6. All 13 escapes landed
there, 11–14 m from the nearest building, so clearance was never the issue.

A clean escape exists in principle — far east or far west leaves both zones — but
every intermediate sample of the 3 s forecast is still inside one of them, so no
heading can be *certified* clean. The Shield falls back to its documented
best-effort branch, and the emitted action still violates one zone.

**All 13 kept moving.** That is the property that matters here: freezing inside a
zone re-raises the identical violation every tick forever, and the fail-safe
becomes the deadlock. The design already knows this; the fuzz confirms it holds
in the corner case.

**Reachability.** Not by flying — the Shield stops the aircraft entering, which
the 3 000-sample legal-start fuzz confirms. But `hot_apply()` can drop a new zone
on top of the aircraft mid-flight, so this is a real case, not a fuzzer artefact.
Whether overlapping zones should be merged at ingest is an open design question.

---

## Answered without flying

Two questions that would otherwise have cost simulator time:

**Does `follow_car_gap.yaml` actually leave a gap?** It had never been flown. The
policy comment claims 7 m of legal road survives east of the fence. Verified
offline by probing hover legality across x 26–50 at the band mid-altitude: the
gap exists and is wide enough, and the fence interior is genuinely enforced. The
flight is now worth running; before this it might have failed for policy reasons
and been read as a guardrail failure.

**Does `follow_car_nfz.yaml` really block the whole corridor?** It claims "no way
around it". Asserted against the 30–50 m corridor bounds, so a future edit that
narrows the fence fails the test instead of quietly producing the earlier
outcome — an x 30–40 fence the aircraft simply tracked around, 11 interventions
and nothing to see.

---

## What is still not covered

- **Lost-target behaviour near a boundary.** COAST → SEARCH → SCAN is controller
  logic, not Shield logic, and has no test. A target lost *while held at a fence*
  is the interesting case.
- **Hot-apply under load.** `hot_apply()` has one test. Repeated mid-flight
  policy swaps, and the overlapping-zone case above, are untested.
- **Frame contract against upstream.** Ours is world-frame (vx North, vy East);
  `vlaguard_common.Action4D` is body-frame with the rotation confined to the
  MAVLink adapter. Both cannot be right, and no test compares them.
