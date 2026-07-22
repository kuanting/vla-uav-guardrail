# Finding Report — Flight Wobble During NFZ Avoidance: Root Cause & Fix
NTUT AIoT Lab · Guardrail work package · 2026-07-07
*Closes open item #1 from the 2026-07-07 demo meeting ("path not smooth, cause under investigation").*

## Symptom (as observed in the demo)

While avoiding a no-fly zone — especially the dynamic one injected mid-flight —
the drone's attitude was visibly unstable: it wobbled near the zone edge,
repeatedly approached and pulled back, and occasionally dipped as if falling.

## Root cause (confirmed with measurements)

**A 10 Hz tug-of-war between the pilot and the guardrail.**

1. The VLA (currently a deliberately reckless stub) re-plans every tick and
   always demands "straight to the target, 6 m/s" — a heading that crosses the
   no-fly zone.
2. The Safety Shield, every tick, cancels the into-zone component of that
   velocity (the *slide* repair operator).
3. Net effect: the commanded velocity vector changed direction drastically up
   to 10 times per second at the zone boundary. Measured worst single-tick
   velocity swing: **4.8 m/s per 0.1 s tick**.
4. A quadrotor turns by tilting. Violent command reversals → violent tilt
   changes → momentary loss of vertical thrust → the visible dips ("falling").
5. Separately, the post-mission descent used the autopilot's fast `land`
   behaviour, which also read as "falling" on video.

This behaviour is not random and not a controller fault — it is the expected
cost of *reactive per-tick repair*, and it is exactly what the grant's
**mean repair magnitude** KPI is designed to measure.

## Fix applied (two parts)

1. **Rate limiter at the adapter** (jerk shaping). The emitted velocity may
   change by at most 0.25 m/s horizontally / 0.15 m/s vertically per tick.
   Crucially, the smoothed action is **re-checked against the Shield**; if
   smoothing would delay a safety action (e.g. a brake), the strict unsmoothed
   action is sent instead — comfort never wins over safety.
2. **Gentle landing**: controlled 0.7 m/s descent easing to 0.35 m/s, replacing
   the abrupt land command.

## Results (same urban scenario, A/B)

| Configuration | Worst per-tick velocity swing | Mission | P0 KPI |
|---|---|---|---|
| No smoothing (as demoed) | 4.8 m/s | completes | PASS (0 s in NFZ) |
| Rate limiter, default | 0.9 m/s | completes | PASS |
| Rate limiter, current tuning | **0.5 m/s** | completes | PASS |

Safety metrics unchanged in all configurations; only smoothness improved.

## Remaining work (known, scheduled)

Small residual oscillation at the zone edge is architectural: reactive repair
cannot fully anticipate. The fundamental fix is the **PathRepair** operator
(the Shield plans a short detour seconds ahead instead of patching every
100 ms) — already on the grant roadmap for Q3. The measured repair-magnitude
data above (4.8 → 0.5 m/s) will serve as the baseline for evaluating it.
