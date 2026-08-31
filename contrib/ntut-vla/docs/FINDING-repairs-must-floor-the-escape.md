# A repair must floor an escape, never cap it

**Date:** 2026-08-26
**Status:** fixed and measured. Suite 231 -> 242.

Three repair operators in `guardrail/shield.py` could make the aircraft leave a
P0 breach **more slowly** than it had asked to. This closes them, along with the
stale audit hash left open by the previous pass.

---

## The defect

The monitors are **trend-aware**. The repairs judged **position only**. The
standoff monitor states the principle in its own comment:

> "Already too close but opening the range" passes, because the alternative is
> to raise a violation on the very action that is fixing it.

The repairs never asked it. So once *any* unrelated violation dragged an action
into the repair loop, a position-only repair clobbered an action that was
already correctly escaping. Reproduced with the aircraft flying **directly away**
from the hazard in every case:

| operator | just under the cap | 0.2-0.3 % over it |
|---|---|---|
| `StandoffRecover` | 3.000 m/s | **0.500 m/s** — 6x slower |
| `GeofenceEscape` | 4.000 m/s | **2.000 m/s** — 2x slower |
| `ClearanceFix` | 5.000 m/s | **3.573 m/s** + a sideways component it never asked for |

A third of a percent of overspeed, and the aircraft crawls out of a ring it must
not be in. The KPI could not see it: every one of those actions re-checks clean.

## Why the obvious fix is wrong

"If the monitor is content, decline to act" is the natural reading, and it is
**unsafe**. A design review caught it before any code was written.

The monitor asks *is it escaping* — a sign test with a 0.1 m/s tolerance,
inherited from the reference implementation's `_ESCAPE_SPEED_MPS = 0.1`. The
repairs enforce *is it escaping fast enough to clear the violation* — 0.5 to
3.0 m/s. Gating the repair on the monitor silently swaps the second for the
first. Measured, 1 m inside an NFZ creeping out at 0.15 m/s with a yaw breach:

| | emitted | time to leave the P0 zone |
|---|---|---|
| before | 2.000 m/s | 0.5 s |
| decline (`continue`) | 0.150 m/s | **6.7 s** |
| floor (`max`) | 2.000 m/s | 0.5 s |

So the rule is **floor, never assign, never decline**:

```
out_new = min(cap, max(out_commanded, repair_recovery_rate))
```

It can only ever raise an outward component. The clearance monitor needed the
same treatment for a sharper reason: its forgiveness test is `d > prev + 1e-6`
per forecast sample — an epsilon, not a rate — so a 2 cm/s creep past a building
counts as escaping.

**`min(2.0, cap)` in `GeofenceEscape` stays.** It is not an oversight:
Prof Lai's reference implementation carries `escape_speed_mps: float = 2.0` as a
config default with `recovery = True` and a written rationale, and
`docs/prof-repo-study.md` records this repo adopting it deliberately. It became
a floor, not a ceiling.

## What else came with it

- **The cap now eats the tangential first.** Scaling the whole horizontal vector
  to fit under `speed_max_mps` shrinks the component getting the aircraft out.
  Measured: a 3.00 m/s standoff recovery arrived at 2.12 m/s while 2.12 m/s of
  *mission* motion was preserved untouched. `_cap_sparing_radial` reserves the
  escape and spends what is left.
- **`before` and `after` are now measured the same way.** The clearance repair's
  "never leave the forecast worse" invariant compared `min(dists)` (which
  includes t=0) against `_clear_min_dist` (which excludes it). At (38, 20.5)
  flying away at 5 m/s that is 0.886 against 3.696 — a 4.2x gap, so the guard
  passed automatically exactly where it mattered.
- **`AuditLogger` reads the hash live.** It snapshotted the string at
  construction, so every record after a `hot_apply` carried the hash of a policy
  that no longer applied — the opposite of what `hot_apply`'s docstring
  promises. Reproduced: a record whose violation was `nfz-hot`, stamped with the
  hash of a policy that did not contain `nfz-hot`. It now accepts the policy
  object; the string form still works, so no call site broke.
- **`nfz_hold_ticks` stopped counting buildings.** Teaching `gate()` about
  obstacles the day before made `fblocked` mean "fence *or* building", and the
  counter reported 7 no-fly-zone holds on a policy that declares no fence. Split
  into `nfz_hold_ticks` and `guard_hold_ticks`. The mission verdict reads
  `nfz_s`, which is measured separately and was never affected.

## The refactor, and how it was made safe

`_check` is split into `_check_kinematic`, `_check_altitude`, `_check_standoff`,
`_check_fence` and `_check_clearance`, so the monitor and the repairs share one
definition instead of keeping two that drift. `_check` folds them.

`_check` is what makes the KPI true, so "the tests still pass" is not evidence
enough. `tests/test_check_contract.py` hashes its answers over **28 800 seeded
(state, action) pairs across all 24 policies** — rule id, category, predicted
time and the prose `detail`, because an operator reads `detail` off the audit
log. The digests are unchanged by the refactor. A deliberate future change to
the monitor regenerates them, and the diff is then a reviewable record.

## Measured effect

21 600 adversarial (state, action) pairs, same seed, before and after:

| | before | after |
|---|---|---|
| mean escape speed inside a fence | 1.843 m/s | **2.171 m/s** |
| residual escapes | 96 | 98 |
| breach depth, mean | 2.883 m | **2.676 m** |
| breach depth, p95 | 15.495 m | **13.459 m** |
| breach depth, max | 21.366 m | 21.366 m |

Escapes are **shallower** on average and at the tail, and the aircraft leaves
18 % faster.

**The +2 is reported rather than explained away.** Two samples were fixed and
four became flagged: 94 of 98 are unchanged. Inspecting all four, **three have
an identical breach depth** before and after (1.389, 1.152 and 0.960 m) — the
aircraft is in exactly the same place, now leaving at 5.0 m/s instead of 3.7.
`_clear_horizon_s` scales the forecast with speed, so a faster escape looks
further ahead and can trip clearance on an obstacle further out. That is the
monitor being more conservative about a faster action, not the aircraft being in
more danger. The fourth deepens from 0.591 m to 0.792 m for the same reason.

All four land in the `best is not None` branch, the only path that can emit a
P0-violating action and a documented known limit
(`tests/test_guardrail_coverage.py`). This change neither creates nor closes it.

## Flight

Full KPI set re-flown on the fixed Shield:

| | det_hz | hit rate | ticks seen | sep_end | interventions | P0 escape | unmeasurable |
|---|---|---|---|---|---|---|---|
| `demo_follow` | 3.63 | 1.000 | 1.000 | 16.8 m | 0 | 0.0 | 0 |
| `demo_nfz` | 4.15 | 0.711 | 0.872 | 63.4 m | 0 | 0.0 | 0 |
| `demo_traffic` | 4.03 | 0.977 | 1.000 | 16.9 m | 0 | 0.0 | 0 |

NFZ 0.0 s and altitude 0.0 s on all three. `demo_nfz` still holds at the zone —
454 hold ticks, `fence_mode` `hold` on 449 — so the fence is enforced; the
Shield simply no longer has to, because the controller stops first.

`demo_follow` went from 54 Shield interventions to 0. That is mostly the
building-awareness fix from the day before rather than this one, and the videos
were re-encoded since re-flying replaced their frames.

## Still open

- **`det_hz` under recording** spans 3.4-4.2 and does not reliably clear the
  4.0 Hz gate. Pre-existing, unrelated to the Shield.
- **The `best is not None` branch.** Untouched here and still the only way an
  emitted action can violate. Now visible per-tick in `emitted_violations`,
  which is what makes the numbers in this document measurable at all.
