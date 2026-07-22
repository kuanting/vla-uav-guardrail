# Study — Prof. Lai's reference repo `kuanting-vla-uav-guardrail/`
Studied 2026-07-12. This is the AUTHORITATIVE implementation of the grant; our
own code (`guardrail/`, `training/`, `demo/`, `sitl/`) was built independently
and converged on the same design. This doc maps the two.

## What the repo is
Professional monorepo (uv workspace, Python 3.11, ruff+mypy+pytest, MkDocs site).
29 unit tests pass; `make demo` passes the mid-term gate (P0 escape = 0). It IS
the 7 PDFs we studied on day 1, turned into working code.

Layout:
- `packages/vlaguard-common` — contracts: `Action4D` (frozen), `body_to_local_ned` (single boundary), `DeterminismManifest` (6 fields; HIL must be sim_speedup=1.0)
- `packages/policy-dsl` (WP1) — YAML → validated IR → signed bundle (tar.gz, policy_hash = SHA-256 over canonical IR)
- `packages/safety-shield` (WP3) — ROS-free core: checker → repair stack → FSM → audit
- `ros2_ws/` (WP3) — ROS 2 Humble nodes wrapping the core
- `sim/` — Gazebo Harmonic + SITL + MAVROS compose
- `demo/` — VLA stub + mid_term_demo + **projectairsim_demo** (UE5 perception rail) + sitl_flight_demo + ros_vla_stub
- WP2 (prefix-compiler) + WP4 (stress-harness) = Phase 2/3, not yet built

## Independent convergence (we found the same things)
| Insight | Prof repo | Our code |
|---|---|---|
| 4-D body-frame action, single body→NED boundary | `vlaguard_common.frames` | `guardrail/models.py` + adapter |
| policy_hash reproducibility | signed tar.gz bundle | `Policy.policy_hash` |
| Bounded repair loop (3 iters) | `repair_action` | `Shield.filter` |
| LateralProjection = approaching, **GeofenceEscape = inside, exempt from magnitude cap** | `repair.py` (explicit `recovery=True`) | our "trend-aware + GeofenceEscape" (found via the deadlock bug) |
| "brake while inside = deadlock" | documented in `GeofenceEscape` docstring | our finding-report |
| VLA stub = reckless straight-at-target | `demo/vla_stub.py` | `guardrail/vla_stub.py` |
| Project AirSim UE5 perception rail | `demo/projectairsim_demo.py` | our `docs/projectairsim-setup.md` (installed + flew hello_drone) |
| Gazebo functional rail + SITL + MAVROS | `sim/` | our `sitl/` (4 rails run) |

**Two teams, same architecture, same hard-won bug fixes — strong mutual validation.**

## Where OURS goes beyond the reference (net-new, not in prof repo)
- **WP2 Prefix/Constraint Compiler**: we have a working `guardrail/compiler.py` (NL → mission + CSP). Prof repo lists it as Phase 2 (unbuilt).
- **WP4 Stress harness + trained model**: our `training/` auto-research loop, gate metrics, and the deployable `models/vla_policy_v2.pt/.onnx` — prof repo has no learned model at all (stub only). (Caveat: that's VLA-track scope, a bonus.)
- **REST hot-apply API** (`guardrail/api.py`).
- **Four fully-run rails end-to-end** (AirSim + native SITL + ROS2/MAVROS + Gazebo) with A/B + dynamic NFZ, plus AirSimNH urban world.

## Where the REFERENCE is stronger / more canonical (adopt from it)
- **Coordinate frame**: prof uses true WGS84 lat/lon + a projection layer; ours uses local meters. Theirs is grant-canonical (the DSL spec says WGS84).
- **Packaging discipline**: uv workspace, separate installable packages, mypy strict, MkDocs design site. Ours is a flat script tree.
- **Signed bundle as tar.gz** (policy_id/hash/generation/changelog/signature) — ours hashes in-memory only.
- **ROS 2 = Humble** (grant/AI-Wings parity); we used Jazzy.
- **FSM** matches the spec's names exactly (Normal/Brake/RTL/Land).

## Recommended action
Treat the prof repo as the trunk. Our net-new pieces (Compiler, stress-harness,
trained model, REST API, extra rails) are candidate CONTRIBUTIONS to fold in —
but align to its contracts first (WGS84 frame, Action4D field names: their `vz`
is up-positive in body just like ours; import `vlaguard_common` instead of our
`guardrail.models`). Do NOT diverge into a parallel fork.
