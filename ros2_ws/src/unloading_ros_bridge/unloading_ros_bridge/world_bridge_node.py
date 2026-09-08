from __future__ import annotations

import hashlib

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from unloading_interfaces.msg import PerceptionObservation, PlanningWorldSnapshot
from visualization_msgs.msg import Marker, MarkerArray

from .common import require_humble_python310


class WorldBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("unloading_world_bridge")
        for name, default in (
            ("world_frame", "world"), ("robot_model_fingerprint", ""),
            ("world_model_fingerprint", ""), ("tool_state_identity", ""),
            ("payload_state_identity", ""), ("base_state_identity", ""),
            ("conveyor_state_identity", ""), ("config_identity", ""),
        ):
            self.declare_parameter(name, default)
        self.declare_parameter("tf_timeout_seconds", 0.2)
        self.declare_parameter("snapshot_freshness_seconds", 2.0)
        required = [name for name in ("robot_model_fingerprint", "world_model_fingerprint", "tool_state_identity", "payload_state_identity", "base_state_identity", "conveyor_state_identity", "config_identity") if not self.get_parameter(name).value]
        if required:
            raise RuntimeError("world bridge refuses incomplete mechanism/config state: " + ", ".join(required))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.latest_joint_state = None
        self.sequence = 0
        self.last_fingerprint = ""
        self.last_epoch = None
        self.last_source_sequence = -1
        self.last_snapshot = None
        self.stale_published = False
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(PerceptionObservation, "/unloading/perception", self.on_observation, reliable)
        self.publisher = self.create_publisher(PlanningWorldSnapshot, "/unloading/world_snapshot", reliable)
        self.marker_publisher = self.create_publisher(MarkerArray, "/unloading/markers", reliable)
        self.watchdog = self.create_timer(0.1, self.check_freshness)

    def on_joints(self, message: JointState) -> None:
        if len(message.name) != len(message.position) or not message.name:
            self.get_logger().error("rejecting malformed JointState")
            return
        self.latest_joint_state = message

    def on_observation(self, observation: PerceptionObservation) -> None:
        if observation.source_epoch != self.last_epoch:
            self.last_epoch = observation.source_epoch
            self.last_source_sequence = -1
            self.last_fingerprint = ""
        if observation.source_sequence <= self.last_source_sequence:
            self.get_logger().warning("rejecting duplicate/out-of-order perception observation")
            return
        self.last_source_sequence = observation.source_sequence
        if self.latest_joint_state is None:
            self.get_logger().warning("world snapshot blocked: actual JointState missing")
            return
        output = PlanningWorldSnapshot()
        output.schema_version = observation.schema_version
        output.source_epoch = observation.source_epoch
        output.source_capture_time = observation.capture_time
        output.obstacles = list(observation.cargo)
        output.unknown_regions = list(observation.unknown_regions)
        output.joint_names = list(self.latest_joint_state.name)
        output.actual_joint_positions = list(self.latest_joint_state.position)
        blocking = []
        world_frame = str(self.get_parameter("world_frame").value)
        for cargo in output.obstacles:
            if not cargo.has_pose:
                blocking.append(cargo.source_instance_id + ":POSE_MISSING")
                continue
            stamped = PoseStamped()
            stamped.header.frame_id = cargo.pose_frame_id
            stamped.header.stamp = observation.capture_time
            stamped.pose = cargo.pose
            try:
                transform = self.buffer.lookup_transform(world_frame, cargo.pose_frame_id, rclpy.time.Time.from_msg(observation.capture_time), timeout=Duration(seconds=float(self.get_parameter("tf_timeout_seconds").value)))
                cargo.pose = do_transform_pose(stamped.pose, transform)
                cargo.pose_frame_id = world_frame
            except TransformException as exc:
                blocking.append(cargo.source_instance_id + ":TF_AT_CAPTURE_MISSING")
                self.get_logger().warning(str(exc))
            if cargo.metric_scale_validity != "VALID" or cargo.geometry_validity != "VALID":
                blocking.append(cargo.source_instance_id + ":GEOMETRY_NOT_ADMISSIBLE")
        output.blocking_reasons = sorted(set(blocking))
        output.planning_admissible = not output.blocking_reasons and all(item.candidate_eligible for item in output.obstacles if item.category in ("box", "cardboard_box"))
        fingerprint_payload = repr([(item.source_instance_id, item.pose_frame_id, tuple(item.full_dimensions_m), item.geometry_validity) for item in output.obstacles] + [(item.region_id, item.reason) for item in output.unknown_regions])
        fingerprint = hashlib.sha256(fingerprint_payload.encode("utf-8")).hexdigest()
        if fingerprint != self.last_fingerprint:
            self.sequence += 1
            self.last_fingerprint = fingerprint
        output.scene_revision_sequence = self.sequence
        output.scene_fingerprint = fingerprint
        output.robot_model_fingerprint = str(self.get_parameter("robot_model_fingerprint").value)
        output.world_model_fingerprint = str(self.get_parameter("world_model_fingerprint").value)
        output.tool_state_identity = str(self.get_parameter("tool_state_identity").value)
        output.payload_state_identity = str(self.get_parameter("payload_state_identity").value)
        output.base_state_identity = str(self.get_parameter("base_state_identity").value)
        output.conveyor_state_identity = str(self.get_parameter("conveyor_state_identity").value)
        output.config_identity = str(self.get_parameter("config_identity").value)
        output.world_fingerprint = hashlib.sha256((fingerprint + repr(tuple(output.actual_joint_positions)) + output.config_identity).encode("utf-8")).hexdigest()
        self.publisher.publish(output)
        self.last_snapshot = output
        self.stale_published = False
        self.publish_markers(output, world_frame)

    def check_freshness(self) -> None:
        if self.last_snapshot is None or self.stale_published:
            return
        now_ns = self.get_clock().now().nanoseconds
        capture_ns = self.last_snapshot.source_capture_time.sec * 1_000_000_000 + self.last_snapshot.source_capture_time.nanosec
        age = (now_ns - capture_ns) / 1e9
        maximum = float(self.get_parameter("snapshot_freshness_seconds").value)
        if age <= maximum and age >= 0.0:
            return
        reason = "ROS_TIME_JUMP" if age < 0.0 else "OBSERVATION_STALE"
        self.last_snapshot.planning_admissible = False
        self.last_snapshot.blocking_reasons = sorted(set(list(self.last_snapshot.blocking_reasons) + [reason]))
        self.sequence += 1
        self.last_snapshot.scene_revision_sequence = self.sequence
        self.last_snapshot.scene_fingerprint = hashlib.sha256((self.last_snapshot.scene_fingerprint + reason).encode("utf-8")).hexdigest()
        self.last_snapshot.world_fingerprint = hashlib.sha256((self.last_snapshot.world_fingerprint + reason).encode("utf-8")).hexdigest()
        self.publisher.publish(self.last_snapshot)
        self.stale_published = True

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
