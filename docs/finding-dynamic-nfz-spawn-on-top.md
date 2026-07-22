# Finding Report — Dynamic NFZ Spawn-on-Top: Stress Case Verified
NTUT AIoT Lab · Guardrail work package · 2026-07-09

The design docs make `dynamic_nfz` hot-apply a Phase-2 feature. The hardest
placement of a dynamic NFZ — one that appears **directly on top of the vehicle**,
so the vehicle is instantly inside a zone it never approached — was an untested
stress case for the Phase-1 Shield + the trend-aware recovery fix. This note
records that the fix generalises to it, and adds a regression suite + a visible
demo so the case stays covered.

## The stress case

A vehicle cruising north over open ground. At t = 8 s a `dynamic_nfz` is
hot-applied **centred on the vehicle's current position** (a 60 m square, P0).
The vehicle is now at the polygon's geometric centre — exactly the ambiguous
point where a naive outward-normal test can point the wrong way.

Two sub-cases matter:

* the vehicle's heading already carries it **out** of the new zone;
* the vehicle's heading carries it **deeper** into the new zone.

## Result

Both pass with the existing trend-aware checker + `GeofenceEscape` operator —
**no new fix needed**. The trend test (signed-distance delta over 1 s of motion)
is unambiguous even at the polygon centre: rising ⇒ escaping (sub-case A passes
through); falling ⇒ flagged and `GeofenceEscape` drives the vehicle out
(sub-case B). The real hot-apply path was also exercised: `STRtree` rebuild,
`generation` bump, `policy_hash` re-derive all behave correctly.

| Sub-case | Outcome |
|---|---|
| heading out of spawned zone | not flagged; vehicle continues and clears the north edge |
| heading deeper into spawned zone | `GeofenceEscape` fires; vehicle exits in ~5 s |

## Verification added

* `tests/test_dynamic_nfz_spawn_on_top.py` — 3 regression tests covering both
  sub-cases + the end-to-end exit (red on the pre-fix code, green now).
* `make sim` `spawn_on_top` scenario — a visible plot + report: the dynamic NFZ
  appears centred on the vehicle at t = 8 s; the vehicle is inside ~6 s then
  exits and reaches the target (KPI = exited the NFZ).

## Scope note

This validates the Phase-1 *constraint classes* against the spawn-on-top
placement. The Phase-2 `dynamic_nfz` hot-apply REST endpoint (the delivery
mechanism) and motion (translate/rotate after spawn) remain the documented
Phase-2 work; the recovery logic they rely on is now proven on this placement.
