"""Explicit synthetic robot/mechanism state source for headless tests only."""

from __future__ import annotations

from uuid import uuid4

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unloading_interfaces.msg import MechanismState

from .common import require_humble_python310


class MockStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("unloading_mock_state_publisher")
        self.declare_parameter("joint_names", [f"joint_{index}" for index in range(1, 7)])
        self.declare_parameter("period_seconds", 0.1)
        self.declare_parameter("robot_model_fingerprint", "synthetic-robot-v1")
        self.declare_parameter("world_model_fingerprint", "synthetic-world-v1")
        self.declare_parameter("config_identity", "synthetic_metric_v1")
        self.joints = self.create_publisher(JointState, "/joint_states", 10)
        self.mechanism = self.create_publisher(MechanismState, "/unloading/mechanism_state", 10)
        self.epoch = str(uuid4())
        self.sequence = 0
        self.timer = self.create_timer(float(self.get_parameter("period_seconds").value), self.publish_state)

    def publish_state(self) -> None:
        stamp = self.get_clock().now().to_msg()
        names = list(self.get_parameter("joint_names").value)
        joint = JointState(name=names, position=[0.0] * len(names), velocity=[0.0] * len(names))
        joint.header.stamp = stamp
        self.joints.publish(joint)
        self.mechanism.publish(MechanismState(
            schema_version="1.1.0", source_epoch=self.epoch, sequence=self.sequence,
            source_restart=self.sequence == 0,
            observed_time=stamp, clock_domain="ros", tool_state_identity="synthetic-vacuum-v1",
            payload_state_identity="synthetic-no-payload", base_state_identity="synthetic-base-fixed",
            conveyor_state_identity="synthetic-conveyor-stopped",
            config_identity=str(self.get_parameter("config_identity").value),
            robot_model_fingerprint=str(self.get_parameter("robot_model_fingerprint").value),
            world_model_fingerprint=str(self.get_parameter("world_model_fingerprint").value),
            tool_state_json='{"verified":"synthetic_fixture"}', payload_state_json='{"object_id":null}',
            base_state_json='{"position_m":[0.0,0.0,0.0]}', conveyor_state_json='{"running":false}',
        ))
        self.sequence += 1


def main(args=None) -> None:
    require_humble_python310()
    rclpy.init(args=args)
    node = MockStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
