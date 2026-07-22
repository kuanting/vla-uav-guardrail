# Finding Report — "Already inside the NFZ" Deadlock: Root Cause & Fix
NTUT AIoT Lab · Guardrail work package · 2026-07-08

A defect in the Phase-1 Safety Shield that the design docs and unit suite did
not cover: a vehicle that is **already inside** a P0 no-fly zone cannot recover.
This report records the proof, the fix, and the verification.

## The defect

The Phase-1 Shield (`safety_shield.repair`) repairs an *approaching* trajectory:
`LateralProjection` projects the predicted path off the fence and reports the
penetration depth as its `magnitude_m`. The repair loop caps that magnitude at
the lateral threshold θ (default 2.0 m); over-cap ⇒ repair fails ⇒ the Shield
brakes.

That is correct for an approaching vehicle. It is wrong for a vehicle **already
inside**: its penetration depth is tens of metres by construction, so the cap
always fires, the Shield brakes — and braking while inside locks the violation
in place **forever**. There is no operator that drives an inside vehicle out, so
the state machine can never return to `Normal`. This is a deadlock, not a stall.

Reproduce on the original code: place the vehicle at the school-yard centroid
(signed distance −39 m), give it a "fly straight out" action. The Shield emits a
full stop every tick; the vehicle never moves. (Tests
`test_inside_vehicle_flying_outward_is_not_in_violation` and
`test_inside_vehicle_actually_exits_the_zone` are red on the original code.)

## Why the design didn't catch it

`safety-shield.md` frames repair around the *approaching* case ("project the
predicted point off the fence"). The escalation FSM (`Normal → Brake → …`)
assumes the vehicle is *near* a violation, not *in* one. The unit suite's
geometric tests all start outside and fly toward the zone. "What if it starts
inside?" — e.g. a NFZ hot-applied on top of the vehicle, or a GPS jump that
relocates it across the boundary — was an untested gap.

## The fix (two parts, ported from empirical findings)

### 1. Trend-aware checking (`checker.py`)

A vehicle inside a polygon whose **signed distance is rising** is escaping
toward some exit wall, not violating. The checker now skips flagging such a
case. The trend test probes one second of motion and uses the **signed-distance
delta** (next − now): negative inside, rising toward zero/positive on exit. This
is frame-correct and has no centre-point ambiguity — unlike an outward-normal
dot product, which flips between equidistant boundary walls at the polygon
centre and can flag a genuine escape as a violation.

### 2. `GeofenceEscape` repair operator (`repair.py`)

For a vehicle inside that is **not** escaping (a VLA insisting on diving
deeper), `LateralProjection` now defers, and a new `GeofenceEscape` operator
emits a "fly straight out toward the nearest exit at a moderate speed" action.
It is flagged `recovery = True`, so the repair loop **exempts it from the
magnitude cap** — its large magnitude is the depth being recovered, which the
projection threshold was never meant to govern. Without that exemption the cap
would re-introduce the deadlock.

## Verification

| Check | Result |
|---|---|
| `make check` (ruff + mypy strict) | clean, 20 files |
| `pytest` | **26 passed** (22 original + 4 new, no regression) |
| `make demo` (mid-term gate) | **PASS** — P0 escape rate 0, hash matches, projection fires |
| `make sim` recovery scenario | vehicle starts inside, **exits in 9.6 s** (was: never) |

New regression tests (`tests/test_deadlock_recovery.py`) prove the deadlock on
the original code and the end-to-end recovery of both an escaping and a
deeper-heading inside vehicle.

## Scope & follow-ups

This closes the inside-zone case for the two Phase-1 constraint classes
(`polygon_fence`, `altitude_envelope`). The full escalation FSM
(`Brake → Loiter → RTL → Land` with N-in-T thresholds) and the Phase-2 hot-apply
classes (`dynamic_nfz` on top of the vehicle) remain the documented next steps;
the trend-aware + recovery pattern generalises to them directly.
