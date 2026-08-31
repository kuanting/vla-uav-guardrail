# The grant's hard KPI had never been measured

**Date:** 2026-08-28
**Status:** fixed. The canonical rail now produces measured, KPI-grade numbers.

## What the claim was, and what it rested on

The project's headline result is *P0 violation escape rate = 0.0 on the grant's
canonical ArduPilot topology*. That number existed, on disk, from 2026-08-24.

It had never been measured.

`guardrail/kpi.py` reads a P0 escape from `emitted_violations` — the Shield's
own re-check of the action that actually flew. Neither SITL rail wrote that
field, so every tick fell back to an inference that, as established on
2026-08-26, is structurally incapable of returning a non-zero answer. Recomputed
with the current code, the stored artefacts report:

| run | P0 rate | unmeasurable ticks | mission_success |
|---|---|---|---|
| `sitl_shield_on` | 0.0 | **132 of 132** | True → **False** |
| `ros2_shield_on` | 0.0 | **132 of 132** | True → **False** |
| `ros2_shield_on_dynamic` | 0.0 | **220 of 220** | True → **False** |

100 % of P0 ticks unmeasurable, on the rail whose whole purpose is to produce
contractual figures.

## Three defects, one of them mine

**I broke the dynamic-NFZ path two days earlier.** Commit `01e22ca` made
`AuditLogger.policy_hash` a read-only property so a hot-applied rule restamps
the hash. Seven `demo/*.py` call sites were updated; the two in `sitl/` were
not, and both assign to that property immediately after `hot_apply`. Every
`--dynamic` run died at t = 8 s with `AttributeError`. The change was verified
against 242 tests and none of them touched this code, because **nothing
imported `sitl/` at all**.

**Neither rail logged the flown action's violations.** Which list that is
differs by arm, and getting it wrong would have destroyed the comparison rather
than merely weakened it:

- shield **ON** — `decision.emitted` flew, so its re-check applies.
- shield **OFF** — `raw` flew unmodified, so the violations already found on
  `raw` *are* the flown action's. That is what earns the control run its escape
  rate instead of scoring it clean.

**The ROS 2 rail asserted `sim_speedup=1.0`** as a literal, which
`build_manifest`'s own docstring forbids: *"a hand-written 1.0 is exactly the
number a broken run would also carry."* The effect was perverse — the three runs
that **passed** `is_kpi_grade()` were the ones that asserted the value, while
the pymavlink rail, which reads it from the autopilot, was refused for its
topology.

## Reading SIM_SPEEDUP took three wrong turns

Each looked like it had worked, which is why they are recorded.

1. **`mavros_msgs/ParamGet` is advertised but never returns.** MAVROS 2
   surfaces the vehicle's parameters as ROS 2 *node* parameters on
   `/mavros/param`, so the client must be `rcl_interfaces/GetParameters`.
   `ros2 param get /mavros/param SIM_SPEEDUP` answering *"Double value is: 1.0"*
   is what pointed at this.
2. **Forcing `ParamPull` first made it worse.** MAVROS pulls on FCU connect by
   itself — 1382 parameters on this host — and an extra forced pull raced the
   refill, so the read came back **0.0**, a speedup meaning time had stopped.
   Removed; the read now polls for a positive value and gives up after 20 s.
3. **The `type` field lies.** MAVROS answers `SIM_SPEEDUP` with `type=3`
   (INTEGER) while leaving `integer_value=0` and putting the real number in
   `double_value=1.0`. Trusting `type` read zero — and since `is_kpi_grade()`
   only checks `!= 1.0`, that would have failed every canonical run with a
   nonsense reason instead of an obvious one.

The read also moved out of `_finish()` into `bring_up()`: `_finish()` runs
inside a timer callback, and `spin_until_future_complete()` from within a
callback is a nested spin that never completes.

`None` on any failure is deliberate throughout. An unread speedup surfaces as
unresolved and fails the gate; it never passes as an assumed 1.0.

## The result

Canonical topology, flown 2026-08-28 on a clean tree at `4bafc63fab21`:

| run | P0 escape | P0 ticks | unmeasurable | fail-safe | success | KPI-grade |
|---|---|---|---|---|---|---|
| `ros2_shield_off` | **0.626506** | 52 | **0** | 0.0 | False | **yes** |
| `ros2_shield_on` | **0.0** | 134 | **0** | 1.0 | True | **yes** |
| `ros2_shield_on_dynamic` | **0.0** | 217 | **0** | 1.0 | True | **yes** |

All six manifest fields resolved: `topology=canonical-hil`, `sim_speedup=1.0`
**derived**, `code_revision=4bafc63fab21` clean, `policy_hash`,
`vla_model_hash`, `random_seed`.

The A/B still separates, and now both halves are measurements. The control arm
flies into the zone and 52 of its P0 ticks escape. The shielded arm sees **134
P0 violations and lets none through** — and the dynamic run, which crashed for
two days, sees 217 and lets none through.

The pymavlink rail was re-flown too (0.626506 / 0.0 / 0.0, all with 0
unmeasurable ticks) at `3c31886ceb02`, one commit behind. It is functional-rail
evidence by design — `is_kpi_grade()` refuses its topology — so it is a
cross-check, not a quotable figure.

## What this cost, and the cheap thing that would have prevented it

`tests/test_sitl_rails.py` is new and needs no autopilot, ROS or MAVLink.
Checked against the pre-fix source it catches all seven defects, including the
assignment to a read-only property, which no import-time check can see. It also
exercises both arms of the A/B end to end: the control arm must earn a non-zero
escape rate and the shielded arm must come back measured and clean, so a change
that silently makes the comparison meaningless fails there rather than in a
report.

Suite 242 → 250.

## Still open

- **Perception on this rail.** ArduPilot SITL has no renderer, so all tracking
  evidence stays AirSim-only and outside the gate. The midterm report already
  names this as "the largest remaining piece of work".
- **Scenario sweep harness (WP4)** — the report says verbatim "not built".
- **Corridor and time-window constraints (WP1)** — in the DSL spec, absent from
  our five types.
- **`mean repair magnitude` and `mean time to safe`** — named as grant KPIs,
  never measured, no target ever recorded.
- **Mission Planner** is a locked L4 requirement in the reference architecture
  (`mavlink-router` fan-out, so a GCS can attach while MAVROS is live). Not in
  our control path and nothing is blocked by its absence, but the fan-out is
  cheap and would give the report a real GCS view of the flight.
