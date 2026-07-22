# Agenda — Next Technical Discussion (with team / Prof. Lai)
Prepared 2026-07-07, following the 07-07 demo meeting.

## 1. VIO / VSLAM integration — **POSTPONED → FUTURE WORK** (decided 2026-07-07)

> Requires perception hardware and the perception team's stack; project is
> currently executed solo. Kept here as future work; the adaptive-margin idea
> below remains the guardrail-side contribution when this resumes.

- Ownership confirmed as perception team (per kickoff split) — we consume, not build.
- Questions to settle:
  - Output format: pose only, detections (class + position + confidence), or occupancy map?
  - Frequency & latency budget (meeting mentioned 100–1000 ms zone-update rate — confirm)
  - Coordinate frame + datum (we use local meters; WGS84 planned)
  - Drift / uncertainty reporting: does VIO expose pose covariance?
- **Our proposal to bring:** adaptive safety margin — `margin_m` grows with pose
  uncertainty instead of being a fixed number. Directly answers the "5 m vs 1 m"
  margin discussion; research-worthy.
- Interface draft already exists: `docs/architecture-v2.md` §4 (`detection` /
  `nfz_update` YAML events).

## 2. Training a real model for the VLA slot
- Principle to state clearly: **Shield stays deterministic (never learned); only
  the pilot (VLA slot) is trainable.**
- Proposed ladder:
  - A. Behavior-cloning policy trained on OUR sim rollouts (obs → shield-corrected
    action). Feasible on lab RTX 4080. First learned model in the slot.
  - B. Fine-tune real VLA (CognitiveDrone recipe, OpenVLA + LoRA) — coordinate
    with the 2 VLA-track students; who owns this?
  - C. (out of scope) training from scratch.
- Data pipeline already exists: every run saves frames + YAML prompt + audit
  (raw vs corrected action) = labelled training data. Grant Stress-Testing page
  explicitly lists "collect labelled data for (optionally) VLA training".
- Ask: dataset size target, storage location, who trains what.

## 3. Simulator direction: AirSim (current) vs Project AirSim (iamaisim)
- Facts checked 2026-07-07: Project AirSim = MIT open source, actively maintained
  by ex-Microsoft AirSim engineers (IAMAI), UE 5.2/5.7, Windows 11 + Ubuntu 22,
  prebuilt Windows binaries available (v0.2.0: Blocks, Neighborhood,
  LandscapeMountains, DynamicCity), ArduPilot/PX4/ROS supported, headless flags.
- Cost of switching: new client API (our AirSim adapter rewritten, ~days);
  guardrail core untouched (adapter isolation proven 4×).
- Proposed strategy: keep current stack as workhorse; pilot Project AirSim for
  the H2 photoreal mountain demo; decide jointly.
- Question for team: which simulator do THEY target for integration testing?

## 4. Follow-ups we owe from the 07-07 meeting
- Report: wobble root cause found + fixed (VLA-vs-Shield tug-of-war; rate limiter
  worst per-tick swing 4.8 → 0.5 m/s) — closes AI-flagged item #1.
- External API for dynamic obstacles/NFZ: wrap `Shield.hot_apply()` into a REST
  endpoint (FastAPI) per their request; define auth/permission model.
- 3D NFZ: extend PolygonFence (already has altitude floor/ceiling) toward richer
  3-D shapes; visualization tech undecided (their AI-flagged item #2).
