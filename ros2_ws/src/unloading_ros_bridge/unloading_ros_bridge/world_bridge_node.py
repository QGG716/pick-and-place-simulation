from __future__ import annotations

from dataclasses import replace
import math
from uuid import uuid4

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from unloading_contracts import ObservationStatus, RobotStateRevision
from unloading_interfaces.msg import MechanismState, PerceptionObservation, PlanningWorldSnapshot
from visualization_msgs.msg import Marker, MarkerArray

from unloading_perception.geometry import rotation_from_quaternion, transform_pose
from unloading_perception.scene import (
    ObservationTracker, SnapshotAssembler, SourceEpochGuard,
    build_scene_update, parse_mechanism_bundle,
)

from .common import require_humble_python310, time_to_float
from .mapping import observation_from_msg, snapshot_to_msg


class WorldBridgeNode(Node):
    """Transport/TF adapter around the shared domain tracker and assembler."""

    def __init__(self) -> None:
        super().__init__("unloading_world_bridge")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("expected_joint_names", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        self.declare_parameter("tf_timeout_seconds", 0.2)
        self.declare_parameter("snapshot_freshness_seconds", 2.0)
        self.declare_parameter("robot_state_freshness_seconds", 0.5)
        self.declare_parameter("mechanism_state_freshness_seconds", 2.0)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tracker = ObservationTracker()
        self.assembler = SnapshotAssembler()
        self.mechanism_guard = SourceEpochGuard()
        self.mechanism_stamp = None
        self.robot_sequence = 0
        self.mechanism_sequence = 0
        self.last_robot_content = None
        self.last_mechanism_content = None
        self.last_joint_stamp: float | None = None
        self.last_observation = None
        self.last_tracked = None
        self.last_snapshot = None
        self.stale_key = None
        self.publisher_epoch = str(uuid4())
        self.publisher_sequence = 0
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(MechanismState, "/unloading/mechanism_state", self.on_mechanism, reliable)
        self.create_subscription(PerceptionObservation, "/unloading/perception", self.on_observation, reliable)
        self.publisher = self.create_publisher(PlanningWorldSnapshot, "/unloading/world_snapshot", reliable)
        self.marker_publisher = self.create_publisher(MarkerArray, "/unloading/markers", reliable)
        self.watchdog = self.create_timer(0.1, self.check_freshness)

    def on_mechanism(self, message: MechanismState) -> None:
        if message.schema_version != "1.1.0" or message.clock_domain != "ros" or not all((message.source_epoch, message.tool_state_identity, message.payload_state_identity, message.base_state_identity, message.conveyor_state_identity, message.config_identity, message.robot_model_fingerprint, message.world_model_fingerprint)):
            self.get_logger().error("rejecting incomplete mechanism state")
            return
        stamp = time_to_float(message.observed_time)
        if stamp <= 0.0:
            self.get_logger().error("rejecting mechanism state without sample time")
            return
        try:
            bundle = parse_mechanism_bundle(
                tool_identity=message.tool_state_identity, tool_json=message.tool_state_json,
                payload_identity=message.payload_state_identity, payload_json=message.payload_state_json,
                base_identity=message.base_state_identity, base_json=message.base_state_json,
                conveyor_identity=message.conveyor_state_identity, conveyor_json=message.conveyor_state_json,
                source_epoch=message.source_epoch, source_sequence=int(message.sequence), sample_time=stamp,
            )
            if (
                message.source_epoch == self.mechanism_guard.current_epoch
                and self.mechanism_stamp is not None and stamp <= self.mechanism_stamp
            ):
                raise ValueError("mechanism sample time is duplicate or out of order")
            self.mechanism_guard.accept(
                message.source_epoch, int(message.sequence), restart=bool(message.source_restart)
            )
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"rejecting malformed or stale mechanism state: {exc}")
            return
        # One commit after every field and source takeover rule has passed.
        self.assembler.tool_attachment = bundle["tool_attachment"]
        self.assembler.payload_attachment = bundle["payload_attachment"]
        self.assembler.base_state = bundle["base_state"]
        self.assembler.conveyor_state = bundle["conveyor_state"]
        self.assembler.config_identity = {
            "identity": message.config_identity,
            "robot_model_fingerprint": message.robot_model_fingerprint,
            "world_model_fingerprint": message.world_model_fingerprint,
            "source": "mechanism_state_topic", "source_epoch": message.source_epoch,
        }
        content = (
            self.assembler.tool_attachment, self.assembler.payload_attachment,
            self.assembler.base_state, self.assembler.conveyor_state,
            self.assembler.config_identity,
        )
        content_changed = content != self.last_mechanism_content
        was_stale = self.mechanism_stamp is None or (
            self.get_clock().now().nanoseconds / 1e9 - self.mechanism_stamp
            > float(self.get_parameter("mechanism_state_freshness_seconds").value)
        )
        self.last_mechanism_content = content
        if content_changed:
            self.mechanism_sequence += 1
        self.mechanism_stamp = stamp
        if content_changed or was_stale:
            self._commit_snapshot("mechanism")

    def on_joints(self, message: JointState) -> None:
        expected = tuple(str(name) for name in self.get_parameter("expected_joint_names").value)
        names = tuple(message.name)
        positions = tuple(message.position)
        velocities = tuple(message.velocity)
        stamp = time_to_float(message.header.stamp)
        invalid = (
            names != expected or len(names) != len(set(names)) or len(positions) != len(names)
            or len(velocities) != len(names) or (message.effort and len(message.effort) != len(names))
            or not all(math.isfinite(value) for value in positions + velocities + tuple(message.effort))
            or stamp <= 0.0 or (self.last_joint_stamp is not None and stamp <= self.last_joint_stamp)
        )
        if invalid:
            self.get_logger().error("rejecting malformed, unordered, non-finite, or stale JointState")
            return
        was_stale = self.last_joint_stamp is None or (
            self.get_clock().now().nanoseconds / 1e9 - self.last_joint_stamp
            > float(self.get_parameter("robot_state_freshness_seconds").value)
        )
        self.last_joint_stamp = stamp
        content = (positions, velocities, tuple(message.effort), names)
        content_changed = content != self.last_robot_content
        if content != self.last_robot_content:
            self.robot_sequence += 1
            self.last_robot_content = content
        self.assembler.robot_state = RobotStateRevision(self.robot_sequence, positions, {
            "joint_names": names, "actual_velocities": velocities,
            "actual_efforts": tuple(message.effort),
        }, sample_time=stamp, clock_domain="ros", source="joint_states")
        if content_changed or was_stale:
            self._commit_snapshot("robot")

    def _transform_observation(self, observation):
        world_frame = str(self.get_parameter("world_frame").value)
        transformed = []
        transform_cache = {}
        for cargo in observation.cargo:
            if cargo.pose is None or cargo.pose.frame_id == world_frame:
                transformed.append(cargo)
                continue
            source_frame = cargo.pose.frame_id
            if source_frame not in transform_cache:
                try:
                    transform_cache[source_frame] = self.buffer.lookup_transform(
                        world_frame, source_frame,
                        rclpy.time.Time(nanoseconds=int(round(observation.capture_time * 1_000_000_000))),
                        timeout=Duration(seconds=float(self.get_parameter("tf_timeout_seconds").value)),
                    )
                except TransformException as exc:
                    self.get_logger().warning(str(exc))
                    transform_cache[source_frame] = None
            transform = transform_cache[source_frame]
            if transform is None:
                transformed.append(cargo)
                continue
            q = transform.transform.rotation
            rotation = rotation_from_quaternion((q.x, q.y, q.z, q.w))
            t = transform.transform.translation
            matrix = tuple(tuple(rotation[row][col] for col in range(3)) + ((t.x, t.y, t.z)[row],) for row in range(3)) + ((0.0, 0.0, 0.0, 1.0),)
            pose = transform_pose(matrix, cargo.pose, world_frame)
            corners = None if cargo.corners_3d_m is None else tuple(tuple(sum(rotation[row][col] * point[col] for col in range(3)) + (t.x, t.y, t.z)[row] for row in range(3)) for point in cargo.corners_3d_m)
            axes = None if cargo.axes_3d_rows is None else tuple(tuple(sum(rotation[row][col] * axis[col] for col in range(3)) for row in range(3)) for axis in cargo.axes_3d_rows)
            transformed.append(replace(cargo, pose=pose, corners_3d_m=corners, axes_3d_rows=axes))
        return replace(observation, cargo=tuple(transformed))

    def on_observation(self, message: PerceptionObservation) -> None:
        try:
            observation = self._transform_observation(observation_from_msg(message))
            tracked = self.tracker.update(observation)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"rejecting invalid perception observation: {exc}")
            return
        self.last_observation, self.last_tracked = observation, tracked
        self._commit_snapshot("perception")

    def _commit_snapshot(self, event: str, *, now: float | None = None) -> None:
        """Single entry for evaluating and publishing every world-state event."""
        if self.last_observation is None or self.last_tracked is None or self.assembler.robot_state is None:
            return
        current = self.get_clock().now().nanoseconds / 1e9 if now is None else now
        update = build_scene_update(
            self.last_observation, self.last_tracked, now=current,
            max_age_seconds=float(self.get_parameter("snapshot_freshness_seconds").value),
        )
        if self.last_joint_stamp is None or current - self.last_joint_stamp > float(self.get_parameter("robot_state_freshness_seconds").value) or current < self.last_joint_stamp:
            update = replace(update, planning_admissible=False, blocking_reasons=tuple(dict.fromkeys(update.blocking_reasons + ("ROBOT_STATE_STALE_OR_TIME_JUMP",))))
        if self.mechanism_stamp is None or current - self.mechanism_stamp > float(self.get_parameter("mechanism_state_freshness_seconds").value) or current < self.mechanism_stamp:
            update = replace(update, planning_admissible=False, blocking_reasons=tuple(dict.fromkeys(update.blocking_reasons + ("MECHANISM_STATE_STALE_OR_TIME_JUMP",))))
        self.assembler.update = update
        result = self.assembler.assemble()
        if result.snapshot is None:
            self.get_logger().warning("world snapshot blocked: " + ",".join(result.missing))
            return
        output = snapshot_to_msg(
            result.snapshot, source_epoch=self.last_observation.source_epoch,
            source_capture_time=self.last_observation.capture_time,
            blocking_reasons=update.blocking_reasons,
            obstacles=update.accepted_obstacles, unknown_regions=update.unknown_regions,
            publisher_epoch=self.publisher_epoch,
            publisher_sequence=self.publisher_sequence,
            publisher_restart=self.publisher_sequence == 0,
            published_time=current,
            robot_sample_time=self.last_joint_stamp,
            mechanism_sample_time=self.mechanism_stamp,
            mechanism_revision_sequence=self.mechanism_sequence,
        )
        self.publisher.publish(output)
        self.publisher_sequence += 1
        self.last_snapshot = result.snapshot
        self.publish_markers(output, str(self.get_parameter("world_frame").value))

    def check_freshness(self) -> None:
        if self.last_observation is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        observation_age = now - self.last_observation.capture_time
        robot_age = math.inf if self.last_joint_stamp is None else now - self.last_joint_stamp
        mechanism_age = math.inf if self.mechanism_stamp is None else now - self.mechanism_stamp
        stale_key = (observation_age > float(self.get_parameter("snapshot_freshness_seconds").value) or observation_age < 0.0,
                     robot_age > float(self.get_parameter("robot_state_freshness_seconds").value) or robot_age < 0.0,
                     mechanism_age > float(self.get_parameter("mechanism_state_freshness_seconds").value) or mechanism_age < 0.0)
        if stale_key != self.stale_key:
            self._commit_snapshot("freshness", now=now)
        self.stale_key = stale_key

    def publish_markers(self, snapshot: PlanningWorldSnapshot, frame_id: str) -> None:
        markers = MarkerArray()
        for index, cargo in enumerate(snapshot.obstacles):
            if not cargo.has_pose or not cargo.has_full_dimensions or cargo.pose_frame_id != frame_id:
                continue
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = snapshot.source_capture_time
            marker.ns, marker.id, marker.type, marker.action = "cargo", index, Marker.CUBE, Marker.ADD
            marker.pose = cargo.pose
            marker.scale.x, marker.scale.y, marker.scale.z = cargo.full_dimensions_m
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.9, 0.5, 0.1, 0.5)
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)


def main(args=None) -> None:
    require_humble_python310()
    rclpy.init(args=args)
    node = WorldBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
