"""Flight controller node — arm + takeoff so the Shield actually flies.

The Phase-1 ROS nodes publish/consume the action path, but nothing arms or takes
off the vehicle — so a bare bring-up leaves the copter on the ground (which the
Shield then reads as an altitude-envelope violation). This node closes that gap:
it waits for MAVROS to connect, sets GUIDED, arms, takes off to a cruise
altitude, then idles (the VLA stub + Shield + adapter do the flying). It exits 0
once cruise altitude is reached, so a bring-up script can sequence on it.

MAVROS service/topic names follow the mavros default (Jazzy):
    /mavros/state                        (mavros_msgs/State)
    /mavros/cmd/arming                   (mavros_msgs/srv/CommandBool)
    /mavros/set_mode                     (mavros_msgs/srv/SetMode)
    /mavros/cmd/takeoff                  (mavros_msgs/srv/CommandTOL)
    /mavros/local_position/pose          (geometry_msgs/PoseStamped)  [altitude check]

The state-machine bring-up (verify GUIDED via State, retry arming until EKF is
ready, retry takeoff until accepted) mirrors the in-house prototype's
pymavlink bring-up — ported to the MAVROS service surface.
"""

from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped  # isort: skip
from mavros_msgs.msg import State  # isort: skip
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL  # isort: skip

# MAVROS publishes the two topics this node needs with *different* QoS (verified
# via `ros2 topic info -v` on this Jazzy build). A subscription with the wrong
# QoS gets zero messages (logged as "incompatible QoS"), so takeoff stalls in
# wait_connect. Match each one exactly.
_SENSOR_QOS = QoSProfile(
    depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
)
# /mavros/state is RELIABLE + TRANSIENT_LOCAL (latched: the last state arrives
# immediately on subscription, so connected/mode are seen even between heartbeats).
_STATE_QOS = QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class FlightController(Node):
    def __init__(self) -> None:
        super().__init__("flight_controller")

        self.declare_parameter("cruise_alt_m", 50.0)
        self.declare_parameter("armed_timeout_s", 120.0)
        self.cruise_alt = float(self.get_parameter("cruise_alt_m").value)
        self.armed_timeout = float(self.get_parameter("armed_timeout_s").value)

        self._state = State()
        self._alt_agl = 0.0
        self._phase = "wait_connect"  # wait_connect -> guided -> arm -> takeoff -> cruise -> done
        self._done = False

        self.create_subscription(State, "/mavros/state", self._on_state, _STATE_QOS)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._on_pose, _SENSOR_QOS
        )

        self._arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self._takeoff_cli = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        # 2 Hz state-machine tick: cheap, well under the 10 Hz action rate.
        self.create_timer(0.5, self._tick)
        self._last_attempt = 0.0
        self._last_log = 0.0

    # --- subscriptions ----------------------------------------------------- #

    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        self._alt_agl = -msg.pose.position.z  # NED z-down -> up-positive AGL

    # --- state machine ----------------------------------------------------- #

    def _tick(self) -> None:
        if self._done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9

        # periodic heartbeat of progress so a bring-up can see where we are
        if now - self._last_log > 5.0:
            self._last_log = now
            self.get_logger().info(
                f"phase={self._phase} connected={self._state.connected} "
                f"mode={self._state.mode!r} armed={self._state.armed} alt={self._alt_agl:.1f}m"
            )

        if self._phase == "wait_connect":
            if not self._state.connected:
                return
            self.get_logger().info("MAVROS connected")
            self._phase = "guided"

        if self._phase == "guided":
            if self._state.mode == "GUIDED":
                self._phase = "arm"
                return
            if now - self._last_attempt > 1.0:
                self._call_async(self._mode_cli, SetMode.Request(custom_mode="GUIDED"))
                self._last_attempt = now
            return

        if self._phase == "arm":
            if self._state.armed:
                self.get_logger().info("armed")
                self._phase = "takeoff"
                return
            if now - self._last_attempt > 2.0:  # EKF may refuse for a while
                self._call_async(self._arm_cli, CommandBool.Request(value=True))
                self._last_attempt = now
            return

        if self._phase == "takeoff":
            if now - self._last_attempt > 2.0:
                req = CommandTOL.Request(
                    min_pitch=0.0, yaw=math.nan,
                    latitude=0.0, longitude=0.0, altitude=self.cruise_alt,
                )
                self._call_async(self._takeoff_cli, req)
                self._last_attempt = now
                self.get_logger().info(f"takeoff commanded -> {self.cruise_alt} m")
            self._phase = "cruise"
            return

        if self._phase == "cruise":
            # takeoff is a one-shot command; once climbing, wait for altitude.
            if self._alt_agl >= self.cruise_alt - 1.0:
                self.get_logger().info(
                    f"reached cruise altitude {self._alt_agl:.1f} m — handing off to VLA/Shield"
                )
                self._done = True
                self._phase = "done"

    def _call_async(self, client, request) -> None:
        if client.service_is_ready():
            client.call_async(request)

    @property
    def done(self) -> bool:
        return self._done


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FlightController()
    import time

    deadline = time.time() + node.armed_timeout
    try:
        # spin until cruise altitude reached, timeout, or user interrupt
        while rclpy.ok() and not node.done and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.done:
            node.get_logger().error(
                f"takeoff timed out after {node.armed_timeout:.0f}s (phase={node._phase})"
            )
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
