# Scope Clarification — What This Project Delivers (and what it does NOT)
NTUT AIoT Lab · Constrained VLA for ArduPilot · 2026-07-13
One-page alignment note. Source of truth: the 7 grant design PDFs + Prof. Lai's
reference repo `kuanting-vla-uav-guardrail/`. Both agree.

## The project name, decoded
> *"**Semantic-Spatial Translation** and **Safety-Constrained VLA** for ArduPilot UAVs"*
- **Semantic-Spatial Translation** = the Constraint Compiler (natural language → structured mission). WP2.
- **Safety-Constrained VLA** = making a VLA obey hard limits. WP1 + WP3.
- The word **"VLA" names the technology we CONSTRAIN — not a thing we build.**

## What IS the deliverable (contractual, WP1–WP4)
| WP | Output artefact |
|---|---|
| WP1 | Policy DSL → signed policy bundle (GeoFence, envelope, corridor, time windows) |
| WP2 | Prefix / Constraint Compiler → Constraint Summary Pack |
| WP3 | Safety Shield (ROS 2 node) → MAVLink to ArduPilot |
| WP4 | Stress-testing harness → KPI report + replay bundles |
Plus: KPI numbers (P0 escape rate = 0, mean repair magnitude, mean time to safe)
and a signed final report.

## What is NOT the deliverable
- ❌ Training / building a deployable VLA model.
- The grant states the VLA backend is **switchable** — *"(CognitiveDrone, OpenVLA,
  BitVLA, in-house stubs)... the project does not commit to any one as a default."*
  → the VLA is an **external, pluggable dependency**, supplied off-the-shelf.
- The grant mentions VLA training **only as OPTIONAL**: the stress harness
  *"collect[s] labelled data for analysis and (optionally) VLA training."*
- Prof. Lai's reference repo ships a **VLA stub only** — no model, no training.
  That is the authoritative interpretation.

## Where our trained model (`vla_policy_v2`) fits
NOT "our VLA." It is: (a) an advanced stub that proves the swappable slot works
end-to-end, and (b) WP4 evidence — quantitative proof that upstream constraints
reduce downstream repairs (interventions 4.3% → 0.04%). Present it as
*"a learned flight policy + stress-testing evidence,"* never as "our VLA model."

## Corrected direction
1. Primary focus stays the **Guardrail** (WP1–4). We are on/ahead of schedule,
   and ahead of the reference repo on WP2 (Compiler) and WP4 (harness + model).
2. To show a REAL VLA in a demo: **plug an existing one** (AeroVLA / CognitiveDrone)
   into the slot — do not train from scratch. This is exactly the "switchable
   backend" design. See `docs/vla-backend-howto.md`.
3. Highest-value next work: align our code to the reference contracts (WGS84
   frame, `vlaguard_common`), then fold our Compiler + harness + model into the
   trunk as contributions.

## Solo + agentic-AI + simulation
Adequate for this scope: the deliverable is software + KPIs + report, all
producible in simulation. No physical drone or full team required for the
contractual gates. Real-hardware (Jetson) flight is a stretch goal, not a gate.
