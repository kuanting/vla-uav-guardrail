# Five defects, found by auditing rather than by a failure

**Date:** 2026-08-25
**Status:** all five fixed and verified. Suite 220 -> 231.

Nothing here announced itself. Each was found by reading code against its own
documented intent, and each is recorded with the measurement that proves it.

---

## 1. Every delivered video played about 1.3x too slow

`demo/recorder.py::summary()` divided the frame count by the recorder's **entire
lifetime**. The recorder starts at scene setup and writes nothing until the
control loop first calls `set_hud()` - after arming and the climb, about 27 s
later. `_hud` is never cleared once set, so every skipped slot falls in that one
opening stretch and the writing window is a contiguous block at the end.

Those 27 s of deliberate silence counted as recording time.

| run | frames | reported | true rate |
|---|---|---|---|
| demo_follow | 1724 | 15.57 | **20.01** |
| demo_nfz | 1716 | 15.74 | **20.00** |
| demo_traffic | 1725 | 15.59 | **20.00** |
| people_final | 1731 | 15.12 | **20.00** |
| city_people | 1793 | 14.56 | **20.00** |
| *(six others)* | | 15.0-16.7 | **20.00-20.01** |

Every run hit its 20 Hz target exactly. `tools/make_demo_video.py` takes the
video's fps straight from this number, so every clip shipped before today ran
1.3x slow while the tool printed that the duration was correct.

Fixed by stamping `t_first_write` and dividing by the writing window. The five
referenced deliverables were re-encoded from their existing frames - the frames
were always right:

| | before | after |
|---|---|---|
| demo_follow | 110.7 s | **86.2 s** |
| demo_nfz | 109.0 s | **85.8 s** |
| demo_traffic | 110.6 s | **86.2 s** |
| people_final | 114.5 s | **86.6 s** |
| city_people | 123.1 s | **89.7 s** |

The `WARNING` in the sidecar had fired on every run of the midterm campaign and
was a false alarm every time. A warning that is always on teaches its reader to
ignore it, so it now judges the real rate. `tests/test_recorder.py` covers both
directions: a recorder meeting its target must not warn, and a genuinely slow
one still must.

**This retracts a claim made earlier the same day** - that there was a
"systematic recorder shortfall". There was not. The recorder was fine; the
measurement was wrong.

## 2. Grid indexing violated the documented convention in three places

`demo/city_planner.py:7-9` states it and `demo/build_voxel_map.py:89` uses it:
cell `i` is **centred** at `origin + i*res`, so a point's index is
`round((v - origin) / res)`. Three point-lookups truncated instead, reading the
grid shifted by up to half a cell - 1.0 m at this map's 2.0 m resolution:

- `demo/build_street_mask.py::is_street`
- `demo/follow_vlm.py::FenceGuard._on_street`
- `demo/pedestrians.py::_pace_segment`

Over 40 000 random points the two forms disagreed about whether a point was on a
road **9.4 % of the time**. This became load-bearing the day before, when
`slide()` began running on unfenced policies - that is, on every tracking flight.

## 3. Pedestrians were placed 1 m off, and 5 of 12 stood off-street

`demo/pedestrians.py` computed the cell centre as `ox + (i + 0.5) * res`. It was
the only place in the repository doing so; the other twelve conversions all use
`ox + i * res`. With `ox = -80` and `res = 2` the true centres are even, so
every spot it produced was odd - exactly on a cell boundary, 1 m from the cell
that had just been validated as pavement.

Checked against a correctly indexed lookup, **5 of the 12 figures** in the
`city_people` flight were not on street at all. After the fix: **0 of 12**, with
the camera passing 8.9-25.2 m from them.

## 4. The headline KPI could not report a non-zero escape rate

`guardrail/kpi.py` opens by defining an escape as a P0 violation "still present
in the action the aircraft FLEW", and warns that getting it wrong in the
optimistic direction "would report a perfect score for a broken system".

It then inferred the answer from whether the Shield had done anything at all:

```python
acted = _emitted_differs(r) or bool(r.get("braked")) or bool(reps)
if not acted:
    n_p0_escapes += 1
```

Every Shield branch that raises a violation also appends a `Repair`, so `acted`
was unconditionally true. The one case the KPI exists to catch - a repair that
produces another illegal action - was invisible to it.

**The delivered numbers were nevertheless right.** Re-checking every emitted
action on the delivered flights against `Shield._check` gives 0 P0 escapes,
matching every `kpi.json`. The measurement was wrong, not the result.

Fixed by recording the fact instead of inferring it: `ShieldDecision` now
carries `emitted_violations`, the Shield's own re-check of what it flew, written
to both the flight log and the audit log. Logs that predate the field fall back
to the old inference and are counted in `p0_ticks_not_measurable`, so a zero
that was never measured cannot pass for one that was.

## 5. A Project AirSim run could claim the grant's canonical HIL topology

`guardrail/manifest.py` gates `canonical-hil` on evidence, deliberately - "the
guard opens on EVIDENCE, never on the caller's word". But `scene_path` is itself
evidence and was unused. The canonical rail is ArduPilot SITL and has no
simulator scene; the module already encodes that assumption, since `sim_speedup`
may only be passed "for a rail with no scene file".

So our own `demo/pas_config/scene_semantic.jsonc` plus a well-formed evidence
dict was stamped `canonical-hil` - the one label the grant reads as KPI-grade.
No production caller does this, so nothing shipped was mislabelled.

`build_manifest` now refuses the combination. The test that claimed to cover
this was a duplicate of the evidence test with an unreachable failure message;
it now asserts the property its name always claimed.

---

## Also fixed

- **Four vacuous assertions.** `test_yaw_rate_clamped` compared a rad/s value
  against a degree bound and would have passed at 2578 deg/s - it was the only
  test named for the yaw clamp and was blind to the exact 57.3x units defect
  this project fixed on 2026-08-17. Two depth tests asserted only
  finite-and-positive, so a regression reading the 300 m sky instead of the 25 m
  car would have passed both. `test_standoff_violations_are_repaired_not_escaped`
  asserted that something was attempted rather than that the result was legal.
- **Three direct-invocation runners** caught only `AssertionError`, so any other
  exception silently skipped every remaining test in the file and printed no
  summary - while pytest ran them all. The two modes disagreed about coverage.
- **A gap-width check** measured `max - min` over a possibly-disjoint set. Move
  the fence into the middle of the corridor and two 4 m slivers on opposite
  sides would score as a 24 m gap.
- **`tools/inspect_glb.py` ignored node transforms**, which is the one thing it
  exists to report. The raw Quaternius file measured 1.415 m wide; with the
  armature applied it is 5.197 m. The pedestrian height constant is unaffected -
  it was derived from baked files, which carry no node transforms.
- **`tools/build_report_results.py`**: guarded two files then read a third,
  shadowed a live variable, and passed generated content as an `re.sub`
  replacement string, where one Windows path would raise `bad escape`.

## Verification

Suite **231** (215 + 16 coverage), up from 220. One re-flight with 12
pedestrians and 5 walkers: 0 of 12 off-street, `det_hit_rate` 1.000,
`frac_ticks_seen` 1.000, `sep_end` 16.7 m, Shield interventions 0, P0 escape
rate 0.0 with `p0_ticks_not_measurable` 0 - measured for the first time.
`det_hz` 4.41 unrecorded, clearing the 4.0 gate.

`det_hz` on RECORDED runs spans 3.43-4.07 and does not reliably clear 4.0. That
is unchanged by this work - the lowest reading, 3.43, is `people_control`, flown
before any of it - and remains open.

## Closed since

The three repair operators and the audit hash were fixed on 2026-08-26. See
`docs/FINDING-repairs-must-floor-the-escape.md`.

## Still open

- **`det_hz` under recording**, above.
