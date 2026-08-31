"""
Fase 4 — the SAME Guardrail pipeline, now through a REAL autopilot.

    User Command -> Constraint Compiler -> YAML Prompt -> VLA (stub)
                 -> Safety Shield -> MAVLink SET_POSITION_TARGET_LOCAL_NED
                 -> ArduPilot SITL (real flight code) -> simulated copter

What changed vs demo/run_demo.py: ONLY the bottom adapter. AirSim's toy
velocity API is replaced by pymavlink talking to ArduPilot's GUIDED mode —
exactly the grant's "MAVLink adapter" box. Guardrail code is untouched,
which is itself the point: the Shield doesn't care what flies under it.

Run inside WSL (SITL listening on tcp:127.0.0.1:5760):

    ~/venv-ap/bin/python run_sitl_demo.py --shield on
    ~/venv-ap/bin/python run_sitl_demo.py --shield off
    ~/venv-ap/bin/python run_sitl_demo.py --shield on --dynamic

Frames note: SITL has no camera — that stays AirSim's job (perception rail vs
functional rail, same split the grant makes).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import AuditLogger, Shield, State, load_policy          # noqa: E402
from guardrail.compiler import ConstraintCompiler                      # noqa: E402
from guardrail.geometry import fence_polygon                           # noqa: E402
from guardrail.kpi import compute_from_dir                             # noqa: E402
from guardrail.manifest import (TOPOLOGY_ARDUPILOT_SITL,               # noqa: E402
                                build_manifest, is_kpi_grade,
                                sim_speedup_from_mavlink)
from guardrail.models import AltitudeEnvelope, PolygonFence, XY        # noqa: E402
from guardrail.vla_stub import StubVLA                                 # noqa: E402

from shapely.geometry import Point                                     # noqa: E402

TICK = 0.1
MAX_S = 120
REACH_M = 2.0

DYNAMIC_AT_S = 8.0
DYNAMIC_FENCE = PolygonFence(
    id="nfz-dynamic", type="polygon_fence",
    vertices=[XY(x=23, y=6), XY(x=31, y=6), XY(x=31, y=14), XY(x=23, y=14)],
    margin_m=1.0,
)

# type_mask: use velocity + yaw_rate, ignore position/accel/force/yaw.
# bits(1=ignore): x y z | vx vy vz | ax ay az | force yaw yaw_rate
VEL_YAWRATE_MASK = 0b0101_1100_0111  # 1479


class MavlinkAdapter:
    """The single body/up-positive <-> NED boundary (grant rule: conversion
    lives in the adapter, never in the Shield)."""

    def __init__(self, url: str):
        print(f"[mavlink]  connecting {url} ...")
        self.m = mavutil.mavlink_connection(url)
        self.m.wait_heartbeat()
        print(f"[mavlink]  heartbeat from sys {self.m.target_system}")
        # Without a GCS attached, SITL streams nothing by default — ask for
        # everything at 10 Hz or state() would block forever.
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    def _ack(self, cmd: int, timeout: float = 3.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
            if msg and msg.command == cmd:
                return msg.result == 0
        return False

    def prepare(self, alt_m: float) -> None:
        """State-machine bring-up: verify GUIDED via heartbeat, keep (re)arming,
        retry takeoff — whatever the EKF timing, converge or time out."""
        mav = self.m.mav
        mode_id = self.m.mode_mapping()["GUIDED"]
        deadline = time.time() + 120
        while time.time() < deadline:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
            if hb is None:
                continue
            if hb.custom_mode != mode_id:              # 1. ensure GUIDED (verified)
                self.m.set_mode(mode_id)
                time.sleep(1)
                continue
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:                              # 2. ensure armed (EKF may refuse)
                mav.command_long_send(self.m.target_system, self.m.target_component,
                                      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                      0, 1, 0, 0, 0, 0, 0, 0)
                time.sleep(2)
                continue
            mav.command_long_send(self.m.target_system, self.m.target_component,
                                  mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                  0, 0, 0, 0, 0, 0, 0, alt_m)  # 3. takeoff
            if self._ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
                print(f"[mavlink]  armed + takeoff accepted, climbing to {alt_m} m ...")
                break
            time.sleep(2)
        else:
            raise RuntimeError("bring-up timed out (mode/arm/takeoff)")

        t0 = time.time()
        last_up = 0.0
        while time.time() - t0 < 60:
            st = self.state(timeout=2)
            if st:
                last_up = st.up
                if st.up >= alt_m - 1.0:
                    print(f"[mavlink]  at {st.up:.1f} m")
                    return
        raise RuntimeError(f"takeoff did not reach altitude (last {last_up:.1f} m)")

    def state(self, timeout: float = 1.0) -> State | None:
        msg = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                                timeout=timeout)
        if msg is None:
            return None
        return State(x=msg.x, y=msg.y, up=-msg.z)      # NED z-down -> up-positive

    def send_velocity(self, vx: float, vy: float, vz_up: float,
                      yaw_rate_dps: float) -> None:
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, VEL_YAWRATE_MASK,
            0, 0, 0,                       # position (ignored)
            vx, vy, -vz_up,                # velocity, NED
            0, 0, 0,                       # accel (ignored)
            0, math.radians(yaw_rate_dps))

    def land_disarm(self) -> None:
        self.m.set_mode(self.m.mode_mapping()["LAND"])
        print("[mavlink]  LAND")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--command", default="fly to the northeast pad at 6 m/s")
    ap.add_argument("--policy", default=str(ROOT / "policies" / "sim_demo_policy.yaml"))
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--dynamic", action="store_true")
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--tag", default=None)
    args = ap.parse_args()

    shield_on = args.shield == "on"
    tag = args.tag or (f"sitl_shield_{args.shield}" + ("_dynamic" if args.dynamic else ""))
    out = ROOT / "demo" / "out" / tag
    out.mkdir(parents=True, exist_ok=True)

    policy = load_policy(args.policy)
    compiler = ConstraintCompiler(policy)
    mission = compiler.parse_command(args.command)
    (out / "prompt.yaml").write_text(compiler.build_prompt(mission), encoding="utf-8")
    print(f"[compiler] target=({mission.target_x:.0f},{mission.target_y:.0f}) "
          f"alt={mission.cruise_alt_m:.0f}m speed_pref={mission.speed_pref_mps:.0f}m/s")
    print(f"[policy]   {policy.policy_id}  {policy.policy_hash}")

    vla = StubVLA(mission)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    # The POLICY, not a snapshot of its hash. This rail hot-applies a fence
    # mid-flight, and AuditLogger reads the hash live so records written after
    # that carry the generation that was actually in force.
    audit = AuditLogger(out / "audit.jsonl", policy)

    link = MavlinkAdapter(args.url)
    link.prepare(mission.cruise_alt_m)
    print(f"[flight]   shield={'ON' if shield_on else 'OFF'} — mission start")

    traj: list[dict] = []
    rows: list[dict] = []
    n_touched = n_braked = 0
    spawn_t = spawn_pos = None
    t0 = time.time()
    tick = 0
    reached = False
    while time.time() - t0 < MAX_S:
        tick += 1
        now = time.time() - t0
        state = link.state()
        if state is None:
            continue

        if args.dynamic and spawn_t is None and now >= DYNAMIC_AT_S:
            shield.hot_apply(DYNAMIC_FENCE)
            # No restamp needed: AuditLogger holds the policy and reads the hash
            # at log time. Assigning here raised AttributeError once policy_hash
            # became a read-only property, which killed every --dynamic run at
            # exactly this line.
            spawn_t, spawn_pos = now, (state.x, state.y)
            print(f"[dynamic]  t={now:.1f}s '{DYNAMIC_FENCE.id}' applied "
                  f"-> generation {policy.generation}")

        dist = math.hypot(mission.target_x - state.x, mission.target_y - state.y)
        if dist < REACH_M:
            reached = True
            print(f"[flight]   target reached at tick {tick} (t={now:.1f}s)")
            break

        raw = vla.act(state)

        # The Shield ALWAYS evaluates; only enforcement is conditional.
        #
        # It used to run only when the shield was on, which left the control run
        # with no record of which rules the flown action broke. guardrail/kpi.py
        # counts a P0 escape from the violations seen on a tick, so a
        # guardrail-free flight with no violations logged scores as perfectly
        # clean - the opposite of what the A/B is for.
        decision = shield.filter(state, raw)
        emitted = decision.emitted if shield_on else raw
        audit.log(tick, decision)
        if shield_on and decision.touched:
            n_touched += 1
            n_braked += int(decision.braked)

        touched_now = bool(shield_on and decision.touched)
        traj.append({"t": round(now, 2), "x": state.x, "y": state.y, "up": state.up,
                     "touched": touched_now})

        # Per-tick row in the shape guardrail/kpi.py reads. NOT audit.jsonl:
        # AuditLogger writes `raw_action`/`emitted_action` and only for touched
        # ticks, so feeding it to compute() would make _emitted_differs() false
        # on every row and report every P0 tick as an escape.
        #
        # repairs and braked record what ACTUALLY happened. With the shield off
        # nothing was repaired, so they stay empty even though the monitor saw
        # the violation - which is precisely how that run earns a non-zero
        # escape rate.
        rows.append({
            "t": round(now, 3), "tick": tick,
            "x": round(state.x, 3), "y": round(state.y, 3), "up": round(state.up, 3),
            "raw": raw.model_dump(),
            "emitted": emitted.model_dump(),
            "violations": [v.model_dump() for v in decision.violations],
            # The check on what was FLOWN - what guardrail/kpi.py counts a P0
            # escape from. Without it every P0 tick scores "not measurable" and
            # the rate falls back to an inference that cannot return non-zero.
            #
            # Which list depends on the arm of the A/B:
            #   shield ON  -> `decision.emitted` flew, so its re-check applies.
            #   shield OFF -> `raw` flew unmodified, so the violations already
            #                 found on `raw` ARE the flown action's violations.
            #                 That is what earns the control run its escape rate
            #                 rather than scoring it clean.
            "emitted_violations": [
                v.model_dump() for v in
                (decision.emitted_violations if shield_on else decision.violations)
            ],
            "repairs": [r.model_dump() for r in decision.repairs] if shield_on else [],
            "braked": bool(shield_on and decision.braked),
            "touched": touched_now,
        })
        link.send_velocity(emitted.vx, emitted.vy, emitted.vz_up, emitted.yaw_rate)
        time.sleep(TICK)

    link.land_disarm()

    # ---- KPI ----
    fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]

    def _active(f, p) -> bool:
        if f.id == DYNAMIC_FENCE.id:
            return spawn_t is not None and p["t"] >= spawn_t
        return True

    inside = sum(1 for p in traj for f, poly in fences
                 if _active(f, p)
                 and f.altitude_floor_m <= p["up"] <= f.altitude_ceiling_m
                 and poly.contains(Point(p["x"], p["y"])))
    nfz_seconds = inside * TICK

    # Seconds outside the altitude envelope, on the same footing as NFZ time so
    # both rails report the same quantities under the same names.
    envelopes = policy.by_type(AltitudeEnvelope)
    alt_bad = sum(1 for p in traj for e in envelopes
                  if not (e.alt_min_m <= p["up"] <= e.alt_max_m))
    alt_seconds = alt_bad * TICK

    # metrics.json first, because compute_from_dir() reads it to decide the
    # mission outcome.
    #
    # frac_within_30m is the follow rails' proximity measure, and compute() uses
    # it as the "did the mission actually happen" signal - below 0.05 the run
    # fails regardless of rule compliance. A waypoint mission has no such
    # fraction, so `reached` is mapped onto it rather than left absent: omitting
    # it would let an aircraft that never left the pad score mission_success,
    # since sitting still breaks no rules. Same contract, one rail's answer
    # expressed in the other's units.
    metrics = {
        "tag": tag,
        "topology": TOPOLOGY_ARDUPILOT_SITL,
        "ticks": len(rows),
        "shield": args.shield,
        "reached": reached,
        "frac_within_30m": 1.0 if reached else 0.0,
        "nfz_s": round(nfz_seconds, 2),
        "alt_violation_s": round(alt_seconds, 2),
        "interventions": n_touched,
        "brakes": n_braked,
    }
    (out / "flight_log.jsonl").write_text(
        "".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    # The determinism manifest. sim_speedup is READ from the autopilot rather
    # than assumed: an unread speedup must surface as "unresolved", because a
    # hand-written 1.0 is exactly what a fast-forwarded run would also carry.
    manifest = build_manifest(
        policy_hash=policy.policy_hash,
        model_id="guardrail.vla_stub.StubVLA",
        seed=0,
        scene_path=None,
        topology=TOPOLOGY_ARDUPILOT_SITL,
        sim_speedup=sim_speedup_from_mavlink(link.m),
    )
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    kpi = compute_from_dir(out, policy)
    # The grade belongs IN the artefact, not only in the prose report. A reader
    # holding kpi.json could not previously tell whether its numbers were
    # quotable as contractual figures - which is the single thing the grade
    # exists to say. demo/follow_vlm.py has recorded it this way all along.
    graded, why = is_kpi_grade(manifest, metrics)
    kpi["kpi_grade"] = graded
    kpi["kpi_grade_reasons"] = why
    kpi["manifest"] = manifest
    (out / "kpi.json").write_text(json.dumps(kpi, indent=2), encoding="utf-8")
    kpi_ok = kpi["p0_violation_escape_rate"] == 0.0

    # ---- plot (if matplotlib present) ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 7))
        for f, poly in fences:
            dyn = f.id == DYNAMIC_FENCE.id
            color = "purple" if dyn else "red"
            xs, ys = poly.exterior.xy
            ax.fill(ys, xs, alpha=0.25, color=color,
                    label=("dynamic NFZ" if dyn else f"NFZ {f.id}"))
        if spawn_pos:
            ax.plot(spawn_pos[1], spawn_pos[0], "X", color="purple", markersize=12)
        ax.plot([p["y"] for p in traj], [p["x"] for p in traj], "-",
                color="tab:blue", linewidth=2, label="flight path")
        tx = [p["y"] for p in traj if p["touched"]]
        if tx:
            ax.plot(tx, [p["x"] for p in traj if p["touched"]], ".",
                    color="orange", markersize=4, label="shield active")
        ax.plot(traj[0]["y"], traj[0]["x"], "go", markersize=10, label="start")
        ax.plot(mission.target_y, mission.target_x, "k*", markersize=16, label="target")
        ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
        ax.set_title(f"SITL (real ArduPilot) — shield {'ON' if shield_on else 'OFF'}\n"
                     f"NFZ time: {nfz_seconds:.1f}s | {'reached' if reached else 'NOT reached'}")
        ax.legend(loc="upper left", fontsize=9); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(out / "trajectory.png", dpi=130)
        print(f"[plot]     {out / 'trajectory.png'}")
    except ImportError:
        print("[plot]     matplotlib missing — skipped")

    (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")
    (out / "report.md").write_text(f"""# SITL Guardrail report — shield {'ON' if shield_on else 'OFF'}

| Item | Value |
|------|-------|
| Autopilot | ArduPilot SITL (real flight code, GUIDED mode) |
| Adapter | MAVLink SET_POSITION_TARGET_LOCAL_NED @ 10 Hz |
| Command | `{args.command}` |
| Policy | `{policy.policy_id}` `{policy.policy_hash}` |
| Target reached | {'yes' if reached else 'NO'} |
| Ticks | {len(traj)} |
| Shield interventions | {n_touched} |
| Brakes | {n_braked} |
| Dynamic NFZ | {f'hot-applied t={spawn_t:.1f}s, generation {policy.generation}' if spawn_t else 'not used'} |
| Topology | `{TOPOLOGY_ARDUPILOT_SITL}` |
| Code revision | `{manifest['code_revision']}` |
| Sim speedup | `{manifest['sim_speedup']}` |
| **Time inside NFZ** | **{nfz_seconds:.1f} s** |
| Time outside altitude envelope | {alt_seconds:.1f} s |
| **P0 violation escape rate** | **{kpi['p0_violation_escape_rate']}** |
| Repairs | {kpi['repair_count']} |
| Mission outcome | {kpi['outcome']} |
| **KPI-grade** | **{'yes' if graded else 'no'}** |

{'' if graded else 'Not KPI-grade because:' + chr(10) + chr(10) + chr(10).join('- ' + r for r in why)}
""", encoding="utf-8")
    print(f"[report]   NFZ {nfz_seconds:.1f}s | alt {alt_seconds:.1f}s "
          f"| P0 escape rate {kpi['p0_violation_escape_rate']} "
          f"-> {'PASS' if kpi_ok else 'FAIL'} | touched {n_touched} | braked {n_braked}")
    print(f"[kpi]      grade={'yes' if graded else 'no'}"
          + ("" if graded else "  (" + "; ".join(why) + ")"))
    print(f"[out]      {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
