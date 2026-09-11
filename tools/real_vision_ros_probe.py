"""Capture and verify one real-worker observation and its matching ROS world snapshot."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from unloading_interfaces.msg import PerceptionObservation, PlanningWorldSnapshot

from unloading_ros_bridge.mapping import observation_from_msg, snapshot_from_msg


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("real_vision_ros_probe")
        self.observation = None
        self.world = None
        self.create_subscription(PerceptionObservation, "/unloading/perception", self._observation, 10)
        self.create_subscription(PlanningWorldSnapshot, "/unloading/world_snapshot", self._world, 10)

    def _observation(self, message) -> None:
        observation = observation_from_msg(message)
        if observation.provider in (
            "cargo-real-image-gpu-worker",
            "cargo-real-image-gpu-resident-worker",
        ):
            self.observation = observation

    def _world(self, message) -> None:
        if self.observation is not None and message.source_epoch == self.observation.source_epoch:
            self.world = (message, snapshot_from_msg(message))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=1900.0)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    started = time.monotonic()
    try:
        while node.observation is None or node.world is None:
            if time.monotonic() - started > args.timeout:
                raise TimeoutError("timed out waiting for matching perception and world messages")
            rclpy.spin_once(node, timeout_sec=0.1)
        world_message, world = node.world
        observation = node.observation
        summary = {
            "schema_version": observation.schema_version,
            "provider": observation.provider,
            "status": observation.status.value,
            "source_epoch": observation.source_epoch,
            "source_sequence": observation.source_sequence,
            "cargo_count": len(observation.cargo),
            "unknown_region_count": len(observation.unknown_regions),
            "raw_image_automatic": False,
            "world_fingerprint": world.fingerprint,
            "world_round_trip_fingerprint_verified": world.fingerprint == world_message.world_fingerprint,
            "planning_admissible": world_message.planning_admissible,
            "blocking_reasons": list(world_message.blocking_reasons),
            "wall_seconds": time.monotonic() - started,
        }
        if world_message.planning_admissible:
            raise AssertionError("uncalibrated monocular real-image evidence unexpectedly became executable")
        if not world_message.blocking_reasons:
            raise AssertionError("rejected world snapshot must carry blocking reasons")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
