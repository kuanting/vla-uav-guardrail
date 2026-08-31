"""
ROS 2 node: VLA stub publisher.

Publishes the grant's action contract on a ROS topic:

    /vla/action_4d   std_msgs/Float32MultiArray   [vx, vy, vz_up, yaw_rate] @ 10 Hz

Subscribes /mavros/local_position/pose for its own observation. A real VLA
backend replaces THIS NODE ONLY — same topic, same message, nothing else moves.
That is the swappable-slot rule from the architecture.
"""
from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import State, load_policy                       # noqa: E402
from guardrail.compiler import ConstraintCompiler              # noqa: E402
from guardrail.vla_stub import StubVLA                         # noqa: E402


class VlaStubNode(Node):
    def __init__(self) -> None:
        super().__init__("vla_stub")
        policy = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
        mission = ConstraintCompiler(policy).parse_command(
            "fly to the northeast pad at 6 m/s")
        self.vla = StubVLA(mission)
        self.state: State | None = None

        # mavros publishes sensor topics best-effort; QoS must match.
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32MultiArray, "/vla/action_4d", 10)
        self.create_timer(0.1, self._tick)                     # 10 Hz — the contract
        self.get_logger().info("VLA stub up, publishing /vla/action_4d")

    def _on_pose(self, msg: PoseStamped) -> None:
        # ENU (mavros) -> our up-positive local frame: x=N, y=E.
        self.state = State(x=msg.pose.position.y, y=msg.pose.position.x,
                           up=msg.pose.position.z)

    def _tick(self) -> None:
        if self.state is None:
            return
        a = self.vla.act(self.state)
        self.pub.publish(Float32MultiArray(data=[a.vx, a.vy, a.vz_up, a.yaw_rate]))


def main() -> None:
    rclpy.init()
    node = VlaStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
