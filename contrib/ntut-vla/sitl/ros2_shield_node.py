"""
ROS 2 node: Safety Shield — the grant's WP3 deliverable shape, in miniature.

Topology (matches the architecture diagram):

    [vla_stub node] --/vla/action_4d--> [THIS NODE] --Twist--> [mavros] --MAVLink--> ArduPilot SITL

Per 10 Hz tick: take the latest VLA action, Shield.filter() it against the
policy, publish ONLY the safe action to /mavros/setpoint_velocity/cmd_vel_unstamped.
Bring-up (GUIDED/arm/takeoff) and landing run through mavros services.

Run (after `source /opt/ros/jazzy/setup.bash`):

    ~/venv-ros/bin/python sitl/ros2_shield_node.py --shield on [--dynamic]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import State as MavState
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from rcl_interfaces.srv import GetParameters

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import Action4D, AuditLogger, Shield, State, load_policy   # noqa: E402
from guardrail.compiler import ConstraintCompiler                         # noqa: E402
from guardrail.geometry import fence_polygon                              # noqa: E402
from guardrail.kpi import compute_from_dir                                # noqa: E402
from guardrail.manifest import (TOPOLOGY_ARDUPILOT_SITL,                  # noqa: E402
                                TOPOLOGY_CANONICAL_HIL, build_manifest,
                                is_kpi_grade)
from guardrail.models import AltitudeEnvelope, PolygonFence, XY           # noqa: E402

from shapely.geometry import Point                                        # noqa: E402

TICK = 0.1
MAX_S = 120
REACH_M = 2.0
DYNAMIC_AT_S = 8.0
DYNAMIC_FENCE = PolygonFence(
    id="nfz-dynamic", type="polygon_fence",
    vertices=[XY(x=23, y=6), XY(x=31, y=6), XY(x=31, y=14), XY(x=23, y=14)],
    margin_m=1.0,
)


class ShieldNode(Node):
    def __init__(self, shield_on: bool, dynamic: bool, out: Path) -> None:
        super().__init__("safety_shield")
        self.shield_on = shield_on
        self.dynamic = dynamic
        self.out = out

        self.policy = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
        compiler = ConstraintCompiler(self.policy)
        self.mission = compiler.parse_command("fly to the northeast pad at 6 m/s")
        (out / "prompt.yaml").write_text(compiler.build_prompt(self.mission),
                                         encoding="utf-8")
        self.shield = Shield(self.policy, lookahead_s=3.0, dt=0.5)
        # The POLICY, not a snapshot of its hash: this rail hot-applies a
        # fence mid-flight and the audit trail must show which generation
        # was in force for each record.
        self.audit = AuditLogger(out / "audit.jsonl", self.policy)

        self.state: State | None = None
        self.raw_action = Action4D()
        self.mav_connected = False
        self.traj: list[dict] = []
        self.rows: list[dict] = []
        # Evidence that this really is the grant's canonical topology, gathered
        # from the live system rather than asserted. build_manifest() refuses
        # canonical-hil without it, and fcu_connected is the load-bearing part:
        # MAVROS comes up happily with nothing on the other end and publishes
        # connected: false forever, so a node can fly a whole mission into the
        # void and look healthy.
        self.hil_evidence: dict = {
            "ros_distro": os.environ.get("ROS_DISTRO"),
            "mavros_node": None,
            "fcu_connected": None,
        }
        self.sim_speedup: float | None = None   # read once, during bring-up
        self.n_touched = self.n_braked = 0
        self.spawn_t = self.spawn_pos = None
        self.mission_t0: float | None = None
        self.done = False
        self.reached = False

        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(MavState, "/mavros/state", self._on_state, 10)
        self.create_subscription(Float32MultiArray, "/vla/action_4d",
                                 self._on_action, 10)
        self.pub = self.create_publisher(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10)

        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_tol = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.cli_rate = self.create_client(StreamRate, "/mavros/set_stream_rate")
        self.cli_param = self.create_client(GetParameters,
                                            "/mavros/param/get_parameters")

    # ---------------- subscriptions ---------------- #

    def _on_pose(self, msg: PoseStamped) -> None:
        # mavros ENU -> our up-positive frame (x=N, y=E). Adapter-edge rule.
        self.state = State(x=msg.pose.position.y, y=msg.pose.position.x,
                           up=msg.pose.position.z)

    def _on_state(self, msg: MavState) -> None:
        self.mav_connected = msg.connected
        # Receiving /mavros/state at all is proof a MAVROS node is on the graph,
        # and msg.connected is MAVROS's own answer about the flight controller.
        # Recorded on every message so the manifest reflects the end of the
        # mission, not an optimistic moment during bring-up.
        self.hil_evidence["mavros_node"] = "/mavros"
        self.hil_evidence["fcu_connected"] = bool(msg.connected)

    def _on_action(self, msg: Float32MultiArray) -> None:
        d = list(msg.data)
        if len(d) == 4:
            self.raw_action = Action4D(vx=d[0], vy=d[1], vz_up=d[2], yaw_rate=d[3])

    # ---------------- helpers ---------------- #

    def _call(self, client, req, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def read_sim_speedup(self) -> float | None:
        """Read SIM_SPEEDUP from the autopilot, or None.

        This rail used to pass a literal 1.0 into build_manifest(), which that
        function's own docstring forbids: "a hand-written 1.0 is exactly the
        number a broken run would also carry." The effect was perverse - the
        runs that PASSED is_kpi_grade() were the ones that asserted the value,
        while the pymavlink rail, which reads it, was refused for its topology.

        Two things about the route, both learned by probing a live MAVROS:

        1. MAVROS 2 surfaces autopilot parameters as ROS 2 NODE PARAMETERS on
           the `/mavros/param` node, not through `mavros_msgs/ParamGet`. That
           service exists and is advertised, and calling it simply never
           returns - which read as None on every flight and looked like a
           timeout. `ros2 param get /mavros/param SIM_SPEEDUP` answers
           "Double value is: 1.0", so `rcl_interfaces/GetParameters` is the
           correct client.
        2. MAVROS populates that parameter set itself on FCU connect (1382
           parameters on this host). Forcing an extra pull raced the refill and
           returned 0.0, so this polls instead.

        Returning None on any failure is deliberate: an unread speedup must
        surface as "unresolved" and fail the gate, never pass as an assumed 1.0.
        """
        # MAVROS pulls the vehicle's parameters itself once the FCU
        # connects, and forcing a second pull here made it WORSE: the read
        # raced the refill and came back 0.0, a speedup that would mean time
        # had stopped. So poll the node parameter instead and accept the first
        # strictly positive answer.
        #
        # Treating 0.0 as "not populated yet" rather than as a value is safe in
        # the direction that matters: if it never resolves this returns None,
        # the manifest records an unresolved speedup, and is_kpi_grade()
        # refuses the run. An unread speedup must never pass as an assumed 1.0.
        deadline = time.time() + 20.0
        while time.time() < deadline:
            try:
                res = self._call(self.cli_param,
                                 GetParameters.Request(names=["SIM_SPEEDUP"]),
                                 timeout=5.0)
            except Exception as e:                            # noqa: BLE001
                self.get_logger().warn(f"[speedup] {type(e).__name__}: {e}")
                return None
            if res is not None and res.values:
                v = res.values[0]
                # DO NOT TRUST `type` HERE. MAVROS answers SIM_SPEEDUP with
                # type=3 (INTEGER) while leaving integer_value=0 and putting
                # the real number in double_value=1.0. Believing the type field
                # read a speedup of zero - which would mean time had stopped -
                # and, because is_kpi_grade() only checks != 1.0, would have
                # quietly failed every canonical run with a nonsense reason.
                #
                # So take whichever field actually carries a value. Both empty
                # means not populated yet; keep polling.
                val = (float(v.double_value) or float(v.integer_value))
                if val > 0.0:
                    return round(val, 4)
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().warn("[speedup] SIM_SPEEDUP never resolved")
        return None

    def bring_up(self) -> None:
        log = self.get_logger()
        t_end = time.time() + 180
        while time.time() < t_end and not self.mav_connected:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not self.mav_connected:
            raise RuntimeError("mavros never connected to FCU")
        log.info("FCU connected")

        # ArduPilot streams nothing until asked (found empirically: pose topic
        # stays silent without this — same lesson as the pymavlink rail).
        self._call(self.cli_rate,
                   StreamRate.Request(stream_id=0, message_rate=10, on_off=True))
        log.info("stream rate 10 Hz requested")

        while time.time() < t_end:
            r = self._call(self.cli_mode, SetMode.Request(custom_mode="GUIDED"))
            if r and r.mode_sent:
                break
            time.sleep(1)
        log.info("GUIDED requested")

        while time.time() < t_end:
            r = self._call(self.cli_arm, CommandBool.Request(value=True))
            if r and r.success:
                log.info("armed")
                break
            time.sleep(2)
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            raise RuntimeError("arming timed out (EKF)")

        alt = self.mission.cruise_alt_m
        while time.time() < t_end:
            r = self._call(self.cli_tol, CommandTOL.Request(altitude=float(alt)))
            if r and r.success:
                log.info(f"takeoff accepted -> {alt} m")
                break
            time.sleep(2)
        else:
            raise RuntimeError("takeoff never accepted")

        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.state and self.state.up >= alt - 1.0:
                log.info(f"at {self.state.up:.1f} m — mission start")
                # Read SIM_SPEEDUP HERE, not in _finish().
                #
                # _finish() runs inside a timer callback, and
                # spin_until_future_complete() from within a callback is a
                # nested spin: the future never completes and the read returned
                # None on every flight. bring_up() owns the main thread and
                # spins explicitly, so the service call resolves normally.
                # Cached because the value cannot change mid-flight.
                self.sim_speedup = self.read_sim_speedup()
                log.info(f"SIM_SPEEDUP read from autopilot: {self.sim_speedup}")
                return
        raise RuntimeError("never reached takeoff altitude")

    # ---------------- mission tick ---------------- #

    def start_mission(self) -> None:
        self.mission_t0 = time.time()
        self.create_timer(TICK, self._tick)

    def _tick(self) -> None:
        if self.done or self.state is None:
            return
        now = time.time() - self.mission_t0
        if now > MAX_S:
            self.get_logger().warn("mission time cap")
            self._finish()
            return
        st = self.state

        if self.dynamic and self.spawn_t is None and now >= DYNAMIC_AT_S:
            self.shield.hot_apply(DYNAMIC_FENCE)
            # No restamp: AuditLogger holds the policy and reads the hash at log
            # time. Assigning here raised AttributeError once policy_hash became
            # a read-only property, killing every --dynamic run at this line.
            self.spawn_t, self.spawn_pos = now, (st.x, st.y)
            self.get_logger().info(
                f"dynamic NFZ applied t={now:.1f}s -> generation {self.policy.generation}")

        dist = math.hypot(self.mission.target_x - st.x, self.mission.target_y - st.y)
        if dist < REACH_M:
            self.reached = True
            self.get_logger().info(f"target reached t={now:.1f}s")
            self._finish()
            return

        raw = self.raw_action

        # The Shield ALWAYS evaluates; only enforcement is conditional. It used
        # to run only when the shield was on, so the control flight recorded no
        # violations at all - and guardrail/kpi.py counts an escape from the
        # violations seen on a tick, which made a guardrail-free run score as
        # perfectly clean. demo/out/ros2_shield_off has no audit.jsonl for
        # exactly that reason.
        decision = self.shield.filter(st, raw)
        emitted = decision.emitted if self.shield_on else raw
        self.audit.log(len(self.traj), decision)
        touched = bool(self.shield_on and decision.touched)
        if touched:
            self.n_touched += 1
            self.n_braked += int(decision.braked)

        self.traj.append({"t": round(now, 2), "x": st.x, "y": st.y,
                          "up": st.up, "touched": touched})

        # Per-tick row in the shape guardrail/kpi.py reads. Not audit.jsonl:
        # AuditLogger writes raw_action/emitted_action and only for touched
        # ticks, so feeding it to compute() would report every P0 tick as an
        # escape. repairs and braked record what ACTUALLY happened, so with the
        # shield off they stay empty and that run earns its escape rate.
        self.rows.append({
            "t": round(now, 3), "tick": len(self.traj),
            "x": round(st.x, 3), "y": round(st.y, 3), "up": round(st.up, 3),
            "raw": raw.model_dump(),
            "emitted": emitted.model_dump(),
            "violations": [v.model_dump() for v in decision.violations],
            # The check on what was FLOWN, which is what guardrail/kpi.py counts
            # a P0 escape from. Without it every P0 tick here scores as "not
            # measurable" and the escape rate falls back to an inference that
            # cannot return a non-zero answer.
            #
            # Which list that is depends on the arm of the A/B, and getting it
            # wrong would destroy the comparison:
            #   shield ON  -> `decision.emitted` flew, so its re-check applies.
            #   shield OFF -> `raw` flew unmodified, so the violations already
            #                 found on `raw` ARE the violations of the flown
            #                 action. That is what earns the control run its
            #                 escape rate instead of scoring it clean.
            "emitted_violations": [
                v.model_dump() for v in
                (decision.emitted_violations if self.shield_on else decision.violations)
            ],
            "repairs": [r.model_dump() for r in decision.repairs] if self.shield_on else [],
            "braked": bool(self.shield_on and decision.braked),
            "touched": touched,
        })

        # our frame -> ENU Twist (adapter edge): E=vy, N=vx, U=vz_up.
        tw = Twist()
        tw.linear.x = float(emitted.vy)
        tw.linear.y = float(emitted.vx)
        tw.linear.z = float(emitted.vz_up)
        tw.angular.z = float(-math.radians(emitted.yaw_rate))  # CW dps -> CCW rad/s
        self.pub.publish(tw)

    # ---------------- wrap-up ---------------- #

    def _finish(self) -> None:
        self.done = True
        # Fire-and-forget: we are INSIDE a timer callback here, so a blocking
        # spin_until_future_complete would raise "Executor is already spinning".
        # The main loop keeps spinning ~3 s after done=True so this goes out.
        if self.cli_mode.service_is_ready():
            self.cli_mode.call_async(SetMode.Request(custom_mode="LAND"))
        self.get_logger().info("LAND requested")
        self._report()

    def _report(self) -> None:
        fences = [(f, fence_polygon(f)) for f in self.policy.by_type(PolygonFence)]

        def active(f, p):
            if f.id == DYNAMIC_FENCE.id:
                return self.spawn_t is not None and p["t"] >= self.spawn_t
            return True

        inside = sum(1 for p in self.traj for f, poly in fences
                     if active(f, p)
                     and f.altitude_floor_m <= p["up"] <= f.altitude_ceiling_m
                     and poly.contains(Point(p["x"], p["y"])))
        nfz_s = inside * TICK
        kpi = "PASS" if nfz_s == 0 else "FAIL"

        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(7, 7))
            for f, poly in fences:
                dyn = f.id == DYNAMIC_FENCE.id
                c = "purple" if dyn else "red"
                xs, ys = poly.exterior.xy
                ax.fill(ys, xs, alpha=0.25, color=c,
                        label="dynamic NFZ" if dyn else f"NFZ {f.id}")
            if self.spawn_pos:
                ax.plot(self.spawn_pos[1], self.spawn_pos[0], "X",
                        color="purple", markersize=12)
            if self.traj:
                ax.plot([p["y"] for p in self.traj], [p["x"] for p in self.traj],
                        "-", color="tab:blue", linewidth=2, label="flight path")
                tx = [p["y"] for p in self.traj if p["touched"]]
                if tx:
                    ax.plot(tx, [p["x"] for p in self.traj if p["touched"]], ".",
                            color="orange", markersize=4, label="shield active")
                ax.plot(self.traj[0]["y"], self.traj[0]["x"], "go",
                        markersize=10, label="start")
            ax.plot(self.mission.target_y, self.mission.target_x, "k*",
                    markersize=16, label="target")
            ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
            ax.set_title(f"ROS 2 + MAVROS rail — shield "
                         f"{'ON' if self.shield_on else 'OFF'}\n"
                         f"NFZ time: {nfz_s:.1f}s | "
                         f"{'reached' if self.reached else 'NOT reached'}")
            ax.legend(loc="upper left", fontsize=9)
            ax.set_aspect("equal"); ax.grid(alpha=0.3)
            fig.tight_layout(); fig.savefig(self.out / "trajectory.png", dpi=130)
        except ImportError:
            pass

        (self.out / "trajectory.json").write_text(json.dumps(self.traj),
                                                  encoding="utf-8")

        # ---- WP4 artefacts -------------------------------------------------
        envelopes = self.policy.by_type(AltitudeEnvelope)
        alt_bad = sum(1 for p in self.traj for e in envelopes
                      if not (e.alt_min_m <= p["up"] <= e.alt_max_m))
        alt_s = alt_bad * TICK

        metrics = {
            "tag": self.out.name,
            "topology": TOPOLOGY_CANONICAL_HIL,
            "ticks": len(self.rows),
            "shield": "on" if self.shield_on else "off",
            "reached": self.reached,
            # compute() reads frac_within_30m as the "did the mission happen"
            # signal. A waypoint mission has no such fraction, so reaching the
            # target is expressed in its units - otherwise an aircraft that
            # never left the pad scores mission_success, since sitting still
            # breaks no rules.
            "frac_within_30m": 1.0 if self.reached else 0.0,
            "nfz_s": round(nfz_s, 2),
            "alt_violation_s": round(alt_s, 2),
            "interventions": self.n_touched,
            "brakes": self.n_braked,
            # Kept here rather than in the manifest: the manifest is exactly six
            # fields by the grant's definition and stays that way.
            "hil_evidence": self.hil_evidence,
        }
        (self.out / "flight_log.jsonl").write_text(
            "".join(json.dumps(r) + "\n" for r in self.rows), encoding="utf-8")
        (self.out / "metrics.json").write_text(json.dumps(metrics, indent=2),
                                               encoding="utf-8")

        # This is the rail the grant names for contractual KPI figures, so it is
        # the one allowed to claim canonical-hil - and only on the evidence
        # gathered above. If MAVROS never reported a connected flight
        # controller, build_manifest() refuses and the run is recorded as what
        # it actually was.
        # READ, never assert. None here fails the KPI gate, which is correct:
        # a speedup nobody measured is not evidence of real-time flight.
        speedup = self.sim_speedup

        try:
            manifest = build_manifest(
                policy_hash=self.policy.policy_hash,
                model_id="guardrail.vla_stub.StubVLA",
                seed=0, scene_path=None, sim_speedup=speedup,
                topology=TOPOLOGY_CANONICAL_HIL,
                hil_evidence=self.hil_evidence)
        except ValueError as e:
            self.get_logger().warn(f"not canonical HIL: {e}")
            manifest = build_manifest(
                policy_hash=self.policy.policy_hash,
                model_id="guardrail.vla_stub.StubVLA",
                seed=0, scene_path=None, sim_speedup=speedup,
                topology=TOPOLOGY_ARDUPILOT_SITL)
        (self.out / "manifest.json").write_text(json.dumps(manifest, indent=2),
                                                encoding="utf-8")

        kpi_res = compute_from_dir(self.out, self.policy)
        # The grade belongs IN the artefact. This is the rail whose numbers are
        # meant to be quotable, and its kpi.json said nothing about whether they
        # were - nor did its report.md, which omitted the manifest entirely.
        graded, why = is_kpi_grade(manifest, {**metrics, "det_hz": None,
                                              "start_heading_err_deg": None})
        kpi_res["kpi_grade"] = graded
        kpi_res["kpi_grade_reasons"] = why
        kpi_res["manifest"] = manifest
        (self.out / "kpi.json").write_text(json.dumps(kpi_res, indent=2),
                                           encoding="utf-8")
        self.get_logger().info(
            f"P0 escape rate {kpi_res['p0_violation_escape_rate']} | "
            f"NFZ {nfz_s:.1f}s | alt {alt_s:.1f}s | interventions {self.n_touched}")
        self.get_logger().info(
            f"KPI-GRADE: {'YES' if graded else 'no'}"
            + ("" if graded else "  (" + "; ".join(why) + ")"))
        (self.out / "report.md").write_text(f"""# ROS 2 rail report — shield {'ON' if self.shield_on else 'OFF'}

| Item | Value |
|------|-------|
| Path | vla_stub node -> /vla/action_4d -> shield node -> mavros -> ArduPilot SITL |
| ROS 2 | Jazzy · rclpy · MAVROS 2 |
| Policy | `{self.policy.policy_id}` `{self.policy.policy_hash}` |
| Target reached | {'yes' if self.reached else 'NO'} |
| Ticks | {len(self.traj)} |
| Shield interventions | {self.n_touched} |
| Brakes | {self.n_braked} |
| Dynamic NFZ | {f'hot-applied t={self.spawn_t:.1f}s, generation {self.policy.generation}' if self.spawn_t else 'not used'} |
| **Time inside NFZ** | **{nfz_s:.1f} s** |
| **P0 KPI** | **{kpi}** |
""", encoding="utf-8")
        print(f"[report] NFZ {nfz_s:.1f}s -> {kpi} | touched {self.n_touched} "
              f"| braked {self.n_braked} | ticks {len(self.traj)}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--dynamic", action="store_true")
    ap.add_argument("--tag", default=None)
    args, ros_args = ap.parse_known_args()

    tag = args.tag or (f"ros2_shield_{args.shield}" + ("_dynamic" if args.dynamic else ""))
    out = ROOT / "demo" / "out" / tag
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = ShieldNode(args.shield == "on", args.dynamic, out)
    try:
        node.bring_up()
        node.start_mission()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
        # keep spinning briefly so LAND goes out
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 3:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
