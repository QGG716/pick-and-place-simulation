from __future__ import annotations

from dataclasses import replace
import math
from uuid import uuid4

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from unloading_contracts import ObservationStatus, RobotStateRevision, canonical_fingerprint
from unloading_interfaces.msg import MechanismState, PerceptionObservation, PlanningWorldSnapshot
from visualization_msgs.msg import MarkerArray

from unloading_perception.geometry import rotation_from_quaternion, transform_pose
from unloading_perception.replay import validate_replay_observation
from unloading_perception.scene import (
    ObservationTracker, SnapshotAssembler, SourceEpochGuard,
    build_scene_update, parse_mechanism_bundle, observation_time_reasons, validate_observation_time_config,
)

from .common import require_humble_python310, time_to_float
from .mapping import observation_from_msg, observation_source_times, snapshot_to_msg
from .marker_display import MarkerScene, marker_qos


class WorldBridgeNode(Node):
    """Transport/TF adapter around the shared domain tracker and assembler."""

    def __init__(self) -> None:
        super().__init__("unloading_world_bridge")
        self.declare_parameter('observation_mode', 'online', ParameterDescriptor(read_only=True))
        self.observation_mode = str(self.get_parameter('observation_mode').value)
        if self.observation_mode not in ('online', 'replay_display_only'):
            raise ValueError('observation_mode must be online or replay_display_only')
        self.replay_guard = SourceEpochGuard()
        self.replay_binding = None
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("expected_joint_names", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        self.declare_parameter("tf_timeout_seconds", 0.2)
        self.declare_parameter("snapshot_freshness_seconds", 2.0)
        self.declare_parameter("observation_future_tolerance_seconds", 0.0)
        self.declare_parameter("robot_state_freshness_seconds", 0.5)
        self.declare_parameter("mechanism_state_freshness_seconds", 2.0)
        initial = self._validate_time_parameters(self.get_parameters([
            'snapshot_freshness_seconds', 'observation_future_tolerance_seconds',
            'robot_state_freshness_seconds', 'mechanism_state_freshness_seconds']))
        if not initial.successful:
            raise ValueError(initial.reason)
        self.add_on_set_parameters_callback(self._validate_time_parameters)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tracker = ObservationTracker()
        self.assembler = SnapshotAssembler()
        self.mechanism_guard = SourceEpochGuard()
        self.algorithm_observation_guard = SourceEpochGuard()
        self.mechanism_stamp = None
        self.robot_sequence = 0
        self.mechanism_sequence = 0
        self.last_robot_content = None
        self.last_mechanism_content = None
        self.last_joint_stamp: float | None = None
        self.last_observation = None
        self.last_tracked = None
        self.last_snapshot = None
        self.time_admission_blocking_reasons = ()
        self.last_time_rejection = None
        self.stale_key = None
        self.publisher_epoch = str(uuid4())
        self.publisher_sequence = 0
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(MechanismState, "/unloading/mechanism_state", self.on_mechanism, reliable)
        self.create_subscription(PerceptionObservation, "/unloading/perception", self.on_observation, reliable)
        self.publisher = self.create_publisher(PlanningWorldSnapshot, "/unloading/world_snapshot", reliable)
        self.marker_scene = MarkerScene(self.get_fully_qualified_name())
        self.marker_publisher = self.create_publisher(MarkerArray, "/unloading/markers", marker_qos())
        # Only the wake-up uses steady time. All age calculations use ROS time.
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.watchdog = self.create_timer(0.1, self.check_freshness, clock=self.watchdog_clock)

    def _validate_time_parameters(self, parameters):
        try:
            for parameter in parameters:
                if parameter.name == 'observation_future_tolerance_seconds':
                    validate_observation_time_config(1.0, parameter.value)
                elif parameter.name in ('snapshot_freshness_seconds', 'robot_state_freshness_seconds',
                                        'mechanism_state_freshness_seconds'):
                    validate_observation_time_config(parameter.value, 0.0)
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _time_context(self, now):
        simulation = bool(self.get_parameter('use_sim_time').value)
        return dict(now=now, max_age_seconds=float(self.get_parameter('snapshot_freshness_seconds').value),
            future_tolerance_seconds=float(self.get_parameter('observation_future_tolerance_seconds').value),
            clock_domain='ros_sim_time' if simulation else 'ros',
            clock_initialized=(now > 0.0 if simulation else True))

    def _reject_observation_time(self, message, reasons):
        self.time_admission_blocking_reasons = tuple(reasons)
        self.last_time_rejection = {'blocking_reasons': tuple(reasons),
            'source_epoch': message.source_epoch, 'source_sequence': int(message.source_sequence),
            'clock_domain': message.clock_domain}
        self.get_logger().error('rejecting observation time: ' + ','.join(reasons))
        # Keep the last valid geometry/source but revoke its planning admission.
        self._commit_snapshot('observation_time_rejected')

    def on_mechanism(self, message: MechanismState) -> None:
        expected_clock = "ros_sim_time" if self.get_parameter("use_sim_time").value else "ros"
        if message.schema_version != "1.1.0" or message.clock_domain != expected_clock or not all((message.source_epoch, message.tool_state_identity, message.payload_state_identity, message.base_state_identity, message.conveyor_state_identity, message.config_identity, message.robot_model_fingerprint, message.world_model_fingerprint)):
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
        }, sample_time=stamp, clock_domain="ros_sim_time" if self.get_parameter("use_sim_time").value else "ros", source="joint_states")
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
        if self.observation_mode == 'replay_display_only':
            self._on_replay_observation(message)
            return
        # Before TF lookup, tracking or either source sequence/epoch guard.
        try:
            capture_time, _ = observation_source_times(message)
            reasons = observation_time_reasons(capture_time, observation_clock_domain=message.clock_domain,
                **self._time_context(self.get_clock().now().nanoseconds / 1e9))
        except (ValueError, TypeError, OverflowError):
            reasons = ('OBSERVATION_TIME_INVALID',)
        if reasons:
            self._reject_observation_time(message, reasons)
            return
        try:
            observation = observation_from_msg(message)
            if 'replay' in observation.coverage:
                self._reject_observation_time(message, ('OBSERVATION_REPLAY_TIME_NOT_ONLINE',))
                return
            observation = self._transform_observation(observation)
            if observation.provider == "registered-rgbd-fused-algorithm":
                from unloading_perception.algorithm_handoff import validate_algorithm_capture
                validate_algorithm_capture(observation)
                self.algorithm_observation_guard.accept(observation.source_epoch, observation.source_sequence, restart=bool(observation.coverage.get("source_restart", False)))
                # Cross-camera image bboxes are incomparable. These fusion IDs
                # remain frame-local until a world-space tracker is available.
                tracked = observation.cargo
            else:
                tracked = self.tracker.update(observation)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"rejecting invalid perception observation: {exc}")
            return
        self.last_observation, self.last_tracked = observation, tracked
        # Only a time-valid observation that also passes existing source guards
        # may clear a time fault. Clock recovery or a joint heartbeat cannot.
        self.time_admission_blocking_reasons = ()
        self._commit_snapshot("perception")

    def _on_replay_observation(self, message):
        try:
            observation = observation_from_msg(message)
            metadata = validate_replay_observation(observation, require_publication=True)
            identity = canonical_fingerprint({
                'record': {k: v for k, v in metadata.items() if k not in (
                    'publication_sequence', 'session_elapsed_seconds', 'published_time', 'publication_clock_domain')},
                'cargo': observation.cargo, 'unknown_regions': observation.unknown_regions})
            if (self.replay_binding is not None and self.replay_binding[0] == observation.source_epoch
                    and self.replay_binding[1] != identity):
                raise ValueError('REPLAY_SESSION_RECORD_CHANGED')
            self.replay_guard.accept(observation.source_epoch, observation.source_sequence,
                restart=bool(observation.coverage.get('source_restart', False)))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self.get_logger().error(f'rejecting replay observation: {exc}')
            return
        self.replay_binding = (observation.source_epoch, identity)
        # No current TF or image tracking can supply missing historical evidence.
        self.last_observation, self.last_tracked = observation, observation.cargo
        self._commit_snapshot('historical_replay')

    def _commit_snapshot(self, event: str, *, now: float | None = None) -> None:
        """Single entry for evaluating and publishing every world-state event."""
        if self.last_observation is None or self.last_tracked is None or self.assembler.robot_state is None:
            return
        current = self.get_clock().now().nanoseconds / 1e9 if now is None else now
        replay = self.observation_mode == 'replay_display_only'
        context = {} if replay else self._time_context(current)
        reasons = () if replay else observation_time_reasons(self.last_observation.capture_time,
            observation_clock_domain=self.last_observation.clock_domain, **context)
        self.time_admission_blocking_reasons = tuple(dict.fromkeys(self.time_admission_blocking_reasons + reasons))
        update = build_scene_update(
            self.last_observation, self.last_tracked, replay_display_only=replay, **context,
        )
        if self.time_admission_blocking_reasons:
            update = replace(update, planning_admissible=False,
                blocking_reasons=tuple(dict.fromkeys(update.blocking_reasons + self.time_admission_blocking_reasons)))
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
        reasons = () if self.observation_mode == 'replay_display_only' else observation_time_reasons(self.last_observation.capture_time,
            observation_clock_domain=self.last_observation.clock_domain, **self._time_context(now))
        robot_age = math.inf if self.last_joint_stamp is None else now - self.last_joint_stamp
        mechanism_age = math.inf if self.mechanism_stamp is None else now - self.mechanism_stamp
        stale_key = (reasons,
                     robot_age > float(self.get_parameter("robot_state_freshness_seconds").value) or robot_age < 0.0,
                     mechanism_age > float(self.get_parameter("mechanism_state_freshness_seconds").value) or mechanism_age < 0.0)
        # Coalesce unchanged high-rate samples at the existing 0.1 s steady
        # watchdog cadence. Reassemble from admitted source samples; never
        # refresh their timestamps or copy/re-date an old world message.
        self._commit_snapshot("freshness" if stale_key != self.stale_key else "heartbeat", now=now)
        self.stale_key = stale_key

    def publish_markers(self, snapshot: PlanningWorldSnapshot, frame_id: str) -> None:
        self.marker_publisher.publish(self.marker_scene.render(snapshot, frame_id))
        for diagnostic in self.marker_scene.diagnostics:
            self.get_logger().warning('display: ' + diagnostic)

    def destroy_node(self):
        # Normal shutdown while DDS is alive clears only our known markers.
        # A killed process / unavailable DDS context cannot guarantee delivery.
        try:
            if rclpy.ok(context=self.context):
                self.marker_publisher.publish(self.marker_scene.clear())
                if not self.marker_publisher.wait_for_all_acked(Duration(seconds=.5)):
                    self.get_logger().warning('display cleanup was not acknowledged')
        except Exception as exc:
            self.get_logger().warning(f'display cleanup failed: {exc}')
        finally:
            result = super().destroy_node()
        return result


def main(args=None) -> None:
    require_humble_python310()
    # Keep the context alive for owned-marker cleanup on ordinary Ctrl-C.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WorldBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
