from __future__ import annotations

from pathlib import Path
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unloading_interfaces.msg import PerceptionObservation

from unloading_perception.backends import CargoJsonReplayBackend

from .common import require_humble_python310
from .mapping import observation_to_msg


class PerceptionReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("unloading_perception")
        self.declare_parameter("backend", "replay")
        self.declare_parameter("replay_path", "")
        self.declare_parameter("publish_period_seconds", 1.0)
        self.declare_parameter("upstream_commit", "1d208f2ed380a207e6e46b4a62d2ac640edfe477")
        if self.get_parameter("backend").value != "replay":
            raise RuntimeError("this node only starts the explicit CPU replay backend; pipeline workers use a separate process")
        replay_path = str(self.get_parameter("replay_path").value)
        if not replay_path:
            raise RuntimeError("replay_path is required; simulation truth is never an implicit fallback")
        self.path = Path(replay_path)
        if not self.path.is_file():
            raise RuntimeError(f"replay_path does not exist: {self.path}")
        self.backend = CargoJsonReplayBackend(str(self.get_parameter("upstream_commit").value))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PerceptionObservation, "/unloading/perception", qos)
        self.sequence = 0
        self.epoch = str(uuid4())
        self.timer = self.create_timer(float(self.get_parameter("publish_period_seconds").value), self.publish)

    def publish(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        observation = self.backend.read(
            self.path, replay_time=now, replay_sequence=self.sequence,
            replay_epoch=self.epoch,
        )
        self.publisher.publish(observation_to_msg(observation))
        self.sequence += 1


def main(args=None) -> None:
    require_humble_python310()
    rclpy.init(args=args)
    node = PerceptionReplayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
