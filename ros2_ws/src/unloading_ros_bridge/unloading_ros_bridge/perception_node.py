from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
from pathlib import Path
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unloading_contracts import ImageMapping, ResourceReference, SensorFrame
from unloading_interfaces.msg import PerceptionObservation

from unloading_perception.backends import CargoJsonReplayBackend, CargoPipelineBackend

from .common import require_humble_python310
from .mapping import observation_to_msg


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class PerceptionNode(Node):
    """Lightweight replay or asynchronous external-worker transport node."""

    def __init__(self) -> None:
        super().__init__("unloading_perception")
        self.declare_parameter("backend", "replay")
        self.declare_parameter("replay_path", "")
        self.declare_parameter("input_image", "")
        self.declare_parameter("input_width", 0)
        self.declare_parameter("input_height", 0)
        self.declare_parameter("input_frame_id", "camera_optical")
        self.declare_parameter("worker_command", [""])
        self.declare_parameter("worker_cwd", "")
        self.declare_parameter("worker_allowed_roots", [""])
        self.declare_parameter("worker_timeout_seconds", 1800.0)
        self.declare_parameter("publish_period_seconds", 0.1)
        self.declare_parameter("upstream_commit", "1d208f2ed380a207e6e46b4a62d2ac640edfe477")
        self.mode = str(self.get_parameter("backend").value)
        self.sequence = 0
        self.epoch = str(uuid4())
        self.future = None
        self.submitted = False
        # rclpy.Node.executor is reserved for the ROS executor association.
        self.worker_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="cargo-gpu-worker")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PerceptionObservation, "/unloading/perception", qos)
        if self.mode == "replay":
            self.path = Path(str(self.get_parameter("replay_path").value)).resolve()
            if not self.path.is_file():
                raise RuntimeError(f"replay_path does not exist: {self.path}")
            self.backend = CargoJsonReplayBackend(str(self.get_parameter("upstream_commit").value))
        elif self.mode == "pipeline":
            self.path = Path(str(self.get_parameter("input_image").value)).resolve()
            cwd = Path(str(self.get_parameter("worker_cwd").value)).resolve()
            command = tuple(item for item in self.get_parameter("worker_command").value if item)
            if not self.path.is_file() or not command or not cwd.is_dir():
                raise RuntimeError("pipeline backend requires an input image, worker command, and worker cwd")
            width, height = int(self.get_parameter("input_width").value), int(self.get_parameter("input_height").value)
            if width <= 0 or height <= 0:
                raise RuntimeError("pipeline input_width/input_height must be positive")
            configured_roots = tuple(
                Path(str(item)).resolve()
                for item in self.get_parameter("worker_allowed_roots").value
                if str(item)
            )
            if any(not root.is_dir() for root in configured_roots):
                raise RuntimeError("every worker_allowed_roots entry must be an existing directory")
            self.backend = CargoPipelineBackend(
                command, cwd=cwd,
                timeout_seconds=float(self.get_parameter("worker_timeout_seconds").value),
                worker_epoch=str(uuid4()),
                allowed_roots=(self.path.parent, cwd, *configured_roots),
            )
        else:
            raise RuntimeError("backend must be exactly 'replay' or 'pipeline'; simulation truth is never an implicit fallback")
        self.timer = self.create_timer(float(self.get_parameter("publish_period_seconds").value), self.poll)

    def poll(self) -> None:
        if self.mode == "replay":
            now = self.get_clock().now().nanoseconds / 1e9
            observation = self.backend.read(self.path, replay_time=now, replay_sequence=self.sequence, replay_epoch=self.epoch)
            self.publisher.publish(observation_to_msg(observation))
            self.sequence += 1
            return
        if not self.submitted:
            modified = self.path.stat().st_mtime
            received = self.get_clock().now().nanoseconds / 1e9
            frame = SensorFrame(
                "file-image", self.path.name, self.epoch, self.sequence, modified,
                max(modified, received), "unix-file-mtime", str(self.get_parameter("input_frame_id").value),
                int(self.get_parameter("input_width").value), int(self.get_parameter("input_height").value),
                "encoded-file", ResourceReference(self.path.as_uri(), _sha256(self.path)),
                image_mapping=ImageMapping(int(self.get_parameter("input_width").value), int(self.get_parameter("input_height").value)),
            )
            self.future = self.worker_pool.submit(self.backend.infer, frame)
            self.submitted = True
            return
        if self.future is not None and self.future.done():
            observation = self.future.result()
            self.publisher.publish(observation_to_msg(observation))
            self.get_logger().info(f"published worker observation status={observation.status.value} id={observation.observation_id}")
            self.future = None

    def destroy_node(self):
        if isinstance(self.backend, CargoPipelineBackend):
            self.backend.shutdown()
        self.worker_pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None) -> None:
    require_humble_python310()
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
