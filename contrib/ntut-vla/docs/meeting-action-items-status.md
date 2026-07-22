# Meeting Action Items — Status & Evidence
Compiled 2026-07-12. Owner: us (solo — covering all 4 grant student roles).
Evidence = concrete files/commands in this repo, verifiable now.

## Role coverage (grant said 4 students → 1 person)
| WP | Grant role | Owner now | Status |
|---|---|---|---|
| WP1 | Policy DSL / IR | us | done (mini) — `guardrail/models.py`, `policies/*.yaml` |
| WP2 | Prefix/Constraint Compiler | us | done (mini) — `guardrail/compiler.py` (prof repo: still Phase 2) |
| WP3 | Safety Shield | us | done — `guardrail/shield.py`, 16 tests, 4 sim rails |
| WP4 | Stress testing / data | us | done + beyond — `training/` auto-research, trained model |

## Action items from 2026-06-23 meeting

| # | Action | Deadline | Status | Evidence |
|---|---|---|---|---|
| 1 | Unify meeting audio / translation | 06-27 | N/A (logistics) | — |
| 2 | Share planning docs + flowchart (VLA→ArduPilot route, Guardrail logic) | 06-30 | ✅ DONE | `docs/architecture-v2.svg`, `docs/architecture-v2.md` |
| 3 | Schedule tech consult w/ Prof. Dai & Lai | 07-03 | ⚠️ AGENDA READY (scheduling = user's) | `docs/agenda-technical-discussion.md` §6 |
| 4 | Define interface w/ Computer Vision + Dynamic NFZ (format, freq, sync) | 07-05 | ✅ PROPOSED (needs their sign-off) | `docs/architecture-v2.md` §4 (YAML `nfz_update` / `detection` schemas) |
| 5 | Finish Verification System prototype; demo validation + block | 07-12 | ✅ DONE (early) | `demo/run_demo.py`, `tests/test_shield.py` (16/16), `demo/out/shield_off` vs `shield_on` |
| 6a | Architecture Diagram v2 (new vs modified) | ongoing | ✅ DONE | `docs/architecture-v2.svg` (color-coded) |
| 6b | Draft initial report | ongoing | ✅ DONE | `docs/initial-report-draft.md` |
| 6c | Bi-weekly meeting slots / contract stamping | ongoing | ⚠️ USER ACTION | — |

## Follow-up items from 2026-07-07 meeting

| # | Action | Status | Evidence |
|---|---|---|---|
| A | More complex scenarios + effect on path smoothness | ✅ DONE | urban world (`policies/urban_demo_policy.yaml`), multi/dynamic NFZ in `training/scenarios.py`, smoothing A/B in `finding-wobble-root-cause.md` |
| B | External API for dynamic obstacle/NFZ + permissions | ✅ DONE (auth = TODO) | `guardrail/api.py` (`POST /nfz`), tested live `demo/out/urban_api_test2` |
| C | 3-D NFZ (altitude constraints + viz) | ◑ PARTIAL | altitude floor/ceiling already in `PolygonFence`; full 3-D shapes + viz = future |
| D | Technical integration + joint test w/ VOA team | ⬜ BLOCKED | needs the other team (solo now) |

## AI-flagged open items (2026-07-07)

| # | Item | Status | Evidence |
|---|---|---|---|
| 1 | Wobble root cause unknown | ✅ SOLVED + FIXED | `docs/finding-wobble-root-cause.md` (tug-of-war; rate limiter 4.8→0.5 m/s) |
| 2 | 2-D→3-D tech choice undecided | ⬜ OPEN | design decision pending |

## Score: 11 of 15 substantive items DONE, 2 proposed/awaiting sign-off,
## 2 blocked on the other team or a pending design decision. On/ahead of schedule.
