"""Validate live ROS timestamp consistency for an Isaac keyframe sequence."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage
from unloading_interfaces.msg import PerceptionObservation


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__(
            "isaac_ros_time_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.clock_stamps: list[int] = []
        self.sensor_stamps = {name: set() for name in ("rgb", "depth", "camera_info", "pointcloud", "tf")}
        self.observations: list[tuple[str, int, int]] = []
        self.observation_keys: set[tuple[str, int]] = set()
        self.create_subscription(Clock, "/clock", self._clock, qos)
        self.create_subscription(Image, "/isaac/front_camera/rgb", lambda msg: self._sensor("rgb", msg.header.stamp), qos)
        self.create_subscription(Image, "/isaac/front_camera/depth", lambda msg: self._sensor("depth", msg.header.stamp), qos)
        self.create_subscription(CameraInfo, "/isaac/front_camera/camera_info", lambda msg: self._sensor("camera_info", msg.header.stamp), qos)
        self.create_subscription(PointCloud2, "/isaac/front_camera/pointcloud", lambda msg: self._sensor("pointcloud", msg.header.stamp), qos)
        self.create_subscription(TFMessage, "/tf", self._tf, qos)
        self.create_subscription(PerceptionObservation, "/unloading/perception", self._observation, qos)

    def _clock(self, message: Clock) -> None:
        self.clock_stamps.append(stamp_ns(message.clock))

    def _sensor(self, name: str, stamp) -> None:
        self.sensor_stamps[name].add(stamp_ns(stamp))

    def _tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self.sensor_stamps["tf"].add(stamp_ns(transform.header.stamp))

    def _observation(self, message: PerceptionObservation) -> None:
        key = (message.source_epoch, int(message.source_sequence))
        if key not in self.observation_keys:
            self.observation_keys.add(key)
            self.observations.append((key[0], key[1], stamp_ns(message.capture_time)))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-keyframes", type=int, default=6)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    deadline = monotonic() + args.timeout
    try:
        while monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(node.observations) >= args.expected_keyframes and len(node.clock_stamps) >= args.expected_keyframes * 2:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if len(node.observations) != args.expected_keyframes:
        raise RuntimeError(f"expected {args.expected_keyframes} keyframes, got {len(node.observations)}")
    missing = {
        f"{epoch}:{sequence}": [name for name, stamps in node.sensor_stamps.items() if stamp not in stamps]
        for epoch, sequence, stamp in node.observations
    }
    missing = {key: value for key, value in missing.items() if value}
    clock_set = set(node.clock_stamps)
    missing_clock = [f"{epoch}:{sequence}" for epoch, sequence, stamp in node.observations if stamp not in clock_set]
    first_epoch = node.observations[0][0]
    first_run = [item for item in node.observations if item[0] == first_epoch]
    progressed = all(
        right[1] > left[1] and right[2] > left[2]
        for left, right in zip(first_run, first_run[1:])
    )
    restarted = any(
        right[0] != left[0] and right[1] == 0 and right[2] < left[2]
        for left, right in zip(node.observations, node.observations[1:])
    )
    duplicates = sum(count - 1 for count in Counter(node.clock_stamps).values() if count > 1)
    status = "PASS" if progressed and restarted and duplicates > 0 and not missing and not missing_clock else "FAIL"
    report = {
        "schema_version": "isaac_ros_time_probe_v1",
        "status": status,
        "use_sim_time": True,
        "keyframes": [
            {"simulation_epoch": epoch, "frame_sequence": sequence, "stamp_ns": stamp}
            for epoch, sequence, stamp in node.observations
        ],
        "clock_progressed": progressed,
        "pause_duplicate_clock_samples": duplicates,
        "restart_created_new_epoch_and_reset_clock": restarted,
        "missing_clock_bindings": missing_clock,
        "missing_sensor_or_tf_stamp_bindings": missing,
        "claim_boundary": "the external adapter is the simulation-clock authority and therefore uses a wall timer; consuming perception/world nodes use use_sim_time=true",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if status == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
