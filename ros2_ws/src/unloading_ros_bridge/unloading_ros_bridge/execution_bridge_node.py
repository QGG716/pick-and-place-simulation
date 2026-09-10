from __future__ import annotations

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unloading_contracts import (
    ControllerStopFact as DomainStopFact,
    ExecutionCommand,
    ExecutionEvent as DomainEvent,
    ExecutionEventKind,
    ExecutionGrant as DomainGrant,
    PlanArtifactKind,
    TimedJointPoint,
    TimedJointTrajectory,
)
from unloading_interfaces.msg import (
    ControllerStopFact, ExecutionAuthorization, ExecutionCancel, ExecutionContext,
    ExecutionEvent, ExecutionGrant, PlanningWorldSnapshot, StopAcknowledgement,
)

from unloading_perception.execution import (
    DuplicateCallbackError, ExecutionGate, StaleCallbackError,
    StopNotReadyError,
)
from unloading_perception.scene import SourceEpochGuard

from .common import float_to_time, require_humble_python310, time_to_float
from .mapping import snapshot_from_msg


class ExecutionBridgeNode(Node):
    """ROS action adapter backed by the shared fail-closed ExecutionGate."""

    def __init__(self) -> None:
        super().__init__("unloading_execution_bridge")
        self.declare_parameter("enable_hardware", False)
        self.declare_parameter("controller_action", "/mock_controller/follow_joint_trajectory")
        self.declare_parameter("controller_id", "mock-follow-joint-trajectory")
        self.declare_parameter("controller_epoch", "")
        self.declare_parameter("stop_fact_max_age_seconds", 1.0)
        self.declare_parameter("world_receive_max_age_seconds", 1.0)
        self.declare_parameter("context_receive_max_age_seconds", 1.0)
        self.declare_parameter("robot_state_max_age_seconds", 0.5)
        self.declare_parameter("mechanism_state_max_age_seconds", 2.0)
        if bool(self.get_parameter("enable_hardware").value):
            raise RuntimeError("enable_hardware=true is refused: no verified FANUC hardware adapter is supplied")
        if not str(self.get_parameter("controller_epoch").value):
            raise RuntimeError("controller_epoch must be supplied by the launched controller")
        # Every state transition, including action futures, runs on this one
        # callback group and one executor thread.  No callback holds a lock or
        # synchronously waits for action, TF, or worker completion.
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.client = ActionClient(self, FollowJointTrajectory, str(self.get_parameter("controller_action").value), callback_group=self.callback_group)
        self.gate = ExecutionGate(enable_hardware=False)
        self.current_world = None
        self.current_world_received_at = None
        self.current_robot_sample_time = None
        self.current_mechanism_sample_time = None
        self.world_source = SourceEpochGuard()
        self.current_context = None
        self.current_context_received_at = None
        self.command_messages = {}
        self.handles = {}
        self.goal_to_command = {}
        self.pending_cancel = set()
        self.cancel_requested = set()
        self.buffered_stop_facts = {}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.events = self.create_publisher(ExecutionEvent, "/unloading/execution_events", qos)
        self.stop_acknowledgements = self.create_publisher(StopAcknowledgement, "/unloading/stop_acknowledgements", qos)
        self.create_subscription(PlanningWorldSnapshot, "/unloading/world_snapshot", self.on_world, qos, callback_group=self.callback_group)
        self.create_subscription(ExecutionContext, "/unloading/execution_context", self.on_context, qos, callback_group=self.callback_group)
        self.create_subscription(ExecutionGrant, "/unloading/execution_grant", self.on_grant, qos, callback_group=self.callback_group)
        self.create_subscription(ExecutionAuthorization, "/unloading/execution_authorization", self.on_command, qos, callback_group=self.callback_group)
        self.create_subscription(ExecutionCancel, "/unloading/execution_cancel", self.on_cancel, qos, callback_group=self.callback_group)
        self.create_subscription(ControllerStopFact, "/unloading/controller_stop_facts", self.on_stop_fact, qos, callback_group=self.callback_group)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def publish_event(self, event: DomainEvent) -> None:
        message = ExecutionEvent(
            command_id=event.command_id, plan_id=event.plan_id, epoch=event.epoch,
            planning_generation=event.planning_generation, kind=event.kind.value,
            event_time=float_to_time(event.event_time), clock_domain=event.clock_domain,
            actual_positions=[] if event.actual_positions is None else list(event.actual_positions),
            actual_velocities=[] if event.actual_velocities is None else list(event.actual_velocities),
            message=event.message,
        )
        self.events.publish(message)

    def reject(self, message, reason: str) -> None:
        event = DomainEvent(message.command_id, message.plan_id, message.epoch, int(message.planning_generation), ExecutionEventKind.REJECTED, self._now(), "ros", message=reason)
        self.publish_event(event)

    def on_world(self, message: PlanningWorldSnapshot) -> None:
        try:
            if message.schema_version != "1.1.0" or not message.publisher_epoch:
                raise ValueError("world publisher identity is missing")
            world = snapshot_from_msg(message)
            self.world_source.accept(
                message.publisher_epoch, int(message.publisher_sequence),
                restart=bool(message.publisher_restart),
            )
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().error(f"rejecting malformed world snapshot: {exc}")
            return
        if self.current_world is not None and world.fingerprint != self.current_world.fingerprint:
            self.gate.revoke_all(now=self._now(), clock_domain="ros")
        self.current_world = world
        self.current_world_received_at = self._now()
        self.current_robot_sample_time = time_to_float(message.robot_sample_time)
        self.current_mechanism_sample_time = time_to_float(message.mechanism_sample_time)

    def on_context(self, message: ExecutionContext) -> None:
        if message.schema_version != "1.1.0" or message.clock_domain != "ros" or not message.session_id or not message.epoch:
            self.get_logger().error("rejecting invalid execution context")
            return
        observed = time_to_float(message.observed_time)
        now = self._now()
        if observed <= 0.0 or observed > now:
            self.get_logger().error("rejecting execution context with invalid observed time")
            return
        previous = self.current_context
        if previous is not None and (
            message.session_id == previous.session_id and message.epoch == previous.epoch
            and int(message.planning_generation) < int(previous.planning_generation)
        ):
            self.get_logger().error("rejecting stale execution context generation")
            return
        self.current_context = message
        self.current_context_received_at = self._now()
        if previous is not None and (previous.session_id, previous.epoch, previous.planning_generation, previous.allowed_plan_id) != (message.session_id, message.epoch, message.planning_generation, message.allowed_plan_id):
            self.gate.revoke_all(now=self._now(), clock_domain="ros")

    def on_grant(self, message: ExecutionGrant) -> None:
        try:
            grant = DomainGrant(
                message.grant_id, message.command_id, message.plan_id, message.request_id,
                message.session_id, message.epoch, int(message.planning_generation),
                message.predecessor_plan_id or None, message.world_fingerprint,
                message.robot_model_fingerprint, message.config_identity,
                message.validation_reference, int(message.validation_generation),
                message.trajectory_fingerprint, time_to_float(message.expires_at),
                message.clock_domain, message.mock_only,
            )
        except ValueError as exc:
            self.get_logger().error(f"rejecting invalid execution grant: {exc}")
            return
        self.gate.register_grant(grant)

    @staticmethod
    def _trajectory(message: ExecutionAuthorization) -> TimedJointTrajectory:
        points = []
        for point in message.trajectory.points:
            time_from_start = time_to_float(point.time_from_start)
            points.append(TimedJointPoint(
                tuple(point.positions), time_from_start,
                tuple(point.velocities) if point.velocities else None,
                tuple(point.accelerations) if point.accelerations else None,
            ))
        return TimedJointTrajectory(
            tuple(message.trajectory.joint_names), tuple(points),
            PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY, message.validation_reference,
            message.robot_model_fingerprint, message.config_identity,
        )

    def _command(self, message: ExecutionAuthorization) -> ExecutionCommand:
        return ExecutionCommand(
            message.command_id, message.plan_id, message.request_id, message.session_id,
            message.epoch, int(message.planning_generation), message.predecessor_plan_id or None,
            message.world_fingerprint, message.robot_model_fingerprint, message.config_identity,
            message.validation_reference, int(message.validation_generation), self._trajectory(message),
        )

    def on_command(self, message: ExecutionAuthorization) -> None:
        if self.current_world is None or self.current_context is None:
            self.reject(message, "AUTHORITATIVE_WORLD_OR_CONTEXT_MISSING")
            return
        context = self.current_context
        stale_reason = self._authoritative_state_error()
        if stale_reason is not None:
            self.reject(message, stale_reason)
            return
        if message.session_id != context.session_id or message.plan_id != context.allowed_plan_id or (message.predecessor_plan_id or "") != context.predecessor_plan_id:
            self.reject(message, "ONLINE_CONTEXT_MISMATCH")
            return
        try:
            command = self._command(message)
        except (ValueError, TypeError) as exc:
            self.reject(message, f"INVALID_EXECUTION_COMMAND:{exc}")
            return
        decision, reservation = self.gate.reserve(
            command, self.current_world, epoch=context.epoch,
            generation=int(context.planning_generation), now=self._now(), clock_domain="ros",
        )
        if reservation is None:
            self.publish_event(decision)
            return
        if not self.client.server_is_ready():
            self.gate.abort_reservation(command.command_id, now=self._now())
            self.reject(message, "CONTROLLER_UNAVAILABLE")
            return
        decision = self.gate.commit_send(command, reservation, now=self._now(), clock_domain="ros")
        self.publish_event(decision)
        if decision.kind is ExecutionEventKind.REJECTED:
            return
        self.command_messages[command.command_id] = message
        goal = FollowJointTrajectory.Goal(trajectory=message.trajectory)
        try:
            future = self.client.send_goal_async(goal, feedback_callback=lambda feedback: self.on_feedback(command, feedback))
        except Exception as exc:
            self.publish_event(self.gate.mark_controller_unknown(command, str(exc), event_time=self._now(), clock_domain="ros"))
            return
        future.add_done_callback(lambda result: self.on_goal_response(command, result))

    def _authoritative_state_error(self) -> str | None:
        now = self._now()
        if self.current_world_received_at is None or now - self.current_world_received_at > float(self.get_parameter("world_receive_max_age_seconds").value) or now < self.current_world_received_at:
            return "WORLD_PUBLISHER_STALE_OR_TIME_JUMP"
        if self.current_context_received_at is None or now - self.current_context_received_at > float(self.get_parameter("context_receive_max_age_seconds").value) or now < self.current_context_received_at:
            return "EXECUTION_CONTEXT_STALE_OR_TIME_JUMP"
        if self.current_robot_sample_time is None or self.current_robot_sample_time <= 0.0 or now - self.current_robot_sample_time > float(self.get_parameter("robot_state_max_age_seconds").value) or now < self.current_robot_sample_time:
            return "ROBOT_STATE_STALE_OR_TIME_JUMP"
        limit = float(self.get_parameter("mechanism_state_max_age_seconds").value)
        if self.current_mechanism_sample_time is None or self.current_mechanism_sample_time <= 0.0 or now - self.current_mechanism_sample_time > limit or now < self.current_mechanism_sample_time:
            return "MECHANISM_STATE_STALE_OR_TIME_JUMP"
        return None

    def on_feedback(self, command: ExecutionCommand, feedback) -> None:
        active = self.gate.active_command
        if active is None or active.command_id != command.command_id:
            self.get_logger().warning("ignoring feedback for stale command")
            return
        event = DomainEvent(command.command_id, command.plan_id, command.epoch, command.planning_generation, ExecutionEventKind.FEEDBACK, self._now(), "ros", tuple(feedback.feedback.actual.positions), tuple(feedback.feedback.actual.velocities), "simulated controller feedback")
        self.publish_event(event)

    def on_goal_response(self, command: ExecutionCommand, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            try:
                self.publish_event(self.gate.mark_controller_unknown(command, str(exc), event_time=self._now(), clock_domain="ros"))
            except StaleCallbackError:
                self.get_logger().warning("ignoring stale goal-response exception")
            return
        if not handle.accepted:
            try:
                self.gate.reject_active_goal(command.command_id, now=self._now())
            except StaleCallbackError:
                return
            self.command_messages.pop(command.command_id, None)
            self.reject(command, "CONTROLLER_REJECTED")
            return
        goal_id = bytes(handle.goal_id.uuid).hex()
        try:
            self.gate.bind_goal(command.command_id, controller_id=str(self.get_parameter("controller_id").value), controller_epoch=str(self.get_parameter("controller_epoch").value), goal_id=goal_id)
        except StaleCallbackError:
            self.get_logger().warning("ignoring goal response for stale command")
            return
        self.handles[command.command_id] = handle
        self.goal_to_command[goal_id] = command.command_id
        self.publish_event(DomainEvent(command.command_id, command.plan_id, command.epoch, command.planning_generation, ExecutionEventKind.STARTED, self._now(), "ros", message=f"controller accepted goal {goal_id}"))
        handle.get_result_async().add_done_callback(lambda result: self.on_result(command, result))
        if command.command_id in self.pending_cancel:
            self._request_controller_cancel(command, handle)

    def on_cancel(self, request: ExecutionCancel) -> None:
        command = self.gate.active_command
        if command is None or (request.command_id, request.plan_id, request.epoch, int(request.planning_generation)) != (command.command_id, command.plan_id, command.epoch, command.planning_generation):
            self.reject(request, "CANCEL_IDENTITY_MISMATCH")
            return
        handle = self.handles.get(command.command_id)
        if handle is None:
            self.pending_cancel.add(command.command_id)
            return
        self._request_controller_cancel(command, handle)

    def _request_controller_cancel(self, command: ExecutionCommand, handle) -> None:
        self.pending_cancel.discard(command.command_id)
        if command.command_id in self.cancel_requested:
            return
        self.cancel_requested.add(command.command_id)
        try:
            handle.cancel_goal_async().add_done_callback(lambda result: self.on_cancel_response(command, result))
        except Exception as exc:
            self.publish_event(self.gate.mark_controller_unknown(command, str(exc), event_time=self._now(), clock_domain="ros"))

    def on_cancel_response(self, command: ExecutionCommand, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            try:
                self.publish_event(self.gate.mark_controller_unknown(command, str(exc), event_time=self._now(), clock_domain="ros"))
            except StaleCallbackError:
                pass
            return
        if not response.goals_canceling:
            self.reject(command, "CONTROLLER_CANCEL_REJECTED")
            return
        buffered = self.buffered_stop_facts.get(self._goal_id_for(command.command_id), [])
        accepted_time = min((fact.cancel_accepted_time for fact in buffered), default=self._now())
        try:
            event = self.gate.accept_cancel(command.command_id, event_time=accepted_time, clock_domain="ros")
        except (StaleCallbackError, StopNotReadyError):
            self.get_logger().warning("ignoring stale cancel response")
            return
        self.publish_event(event)
        for fact in self.buffered_stop_facts.pop(self._goal_id_for(command.command_id), []):
            self._confirm_stop(fact)

    def _goal_id_for(self, command_id: str) -> str:
        return next((goal_id for goal_id, bound in self.goal_to_command.items() if bound == command_id), "")

    def on_result(self, command: ExecutionCommand, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            try:
                self.publish_event(self.gate.mark_controller_unknown(command, str(exc), event_time=self._now(), clock_domain="ros"))
            except StaleCallbackError:
                self.get_logger().warning("ignoring stale result exception")
            return
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            kind = ExecutionEventKind.CANCELED
        elif wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            kind = ExecutionEventKind.SUCCEEDED
        else:
            kind = ExecutionEventKind.FAILED
        try:
            event = self.gate.complete(command.command_id, kind, wrapped.result.error_string, event_time=self._now(), clock_domain="ros")
        except (StaleCallbackError, DuplicateCallbackError):
            self.get_logger().warning("ignoring stale or duplicate terminal result")
            return
        self.publish_event(event)
        if kind is not ExecutionEventKind.CANCELED:
            self._cleanup(command.command_id)

    def on_stop_fact(self, message: ControllerStopFact) -> None:
        try:
            fact = DomainStopFact(
                message.controller_id, message.controller_epoch, message.goal_id,
                int(message.sequence), time_to_float(message.cancel_accepted_time),
                time_to_float(message.stopped_time), message.clock_domain,
                tuple(message.joint_names), tuple(message.actual_positions),
                tuple(message.actual_velocities), message.evidence_reference,
            )
        except ValueError as exc:
            self.get_logger().error(f"rejecting malformed controller stop fact: {exc}")
            return
        try:
            self._confirm_stop(fact)
        except StopNotReadyError:
            if fact.goal_id not in self.goal_to_command:
                self.get_logger().error("rejecting stop fact for unknown goal")
                return
            buffered = self.buffered_stop_facts.setdefault(fact.goal_id, [])
            if len(buffered) < 8:
                buffered.append(fact)
            while len(self.buffered_stop_facts) > 128:
                self.buffered_stop_facts.pop(next(iter(self.buffered_stop_facts)))
        except ValueError as exc:
            self.get_logger().error(f"rejecting controller stop fact: {exc}")

    def _confirm_stop(self, fact: DomainStopFact) -> None:
        acknowledgement = self.gate.confirm_stop(fact, now=self._now(), max_age_seconds=float(self.get_parameter("stop_fact_max_age_seconds").value))
        self.stop_acknowledgements.publish(StopAcknowledgement(
            command_id=acknowledgement.command_id, plan_id=acknowledgement.plan_id,
            epoch=acknowledgement.epoch, planning_generation=acknowledgement.planning_generation,
            stopped_time=float_to_time(acknowledgement.stopped_time), clock_domain=acknowledgement.clock_domain,
            actual_positions=list(acknowledgement.actual_positions), actual_velocities=list(acknowledgement.actual_velocities),
            criterion=acknowledgement.criterion, evidence_reference=acknowledgement.evidence_reference,
            controller_id=acknowledgement.controller_id, controller_epoch=acknowledgement.controller_epoch,
            goal_id=acknowledgement.goal_id, stop_sequence=acknowledgement.stop_sequence,
        ))
        self._cleanup(acknowledgement.command_id)

    def _cleanup(self, command_id: str) -> None:
        self.handles.pop(command_id, None)
        self.command_messages.pop(command_id, None)
        self.pending_cancel.discard(command_id)
        self.cancel_requested.discard(command_id)
        for goal_id, bound in tuple(self.goal_to_command.items()):
            if bound == command_id:
                self.goal_to_command.pop(goal_id, None)
                self.buffered_stop_facts.pop(goal_id, None)


def main(args=None) -> None:
    require_humble_python310()
    rclpy.init(args=args)
    node = ExecutionBridgeNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
