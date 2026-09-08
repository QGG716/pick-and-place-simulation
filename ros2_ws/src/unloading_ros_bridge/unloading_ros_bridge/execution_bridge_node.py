from __future__ import annotations

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unloading_interfaces.msg import (
    ControllerStopFact,
    ExecutionAuthorization,
    ExecutionCancel,
    ExecutionEvent,
    StopAcknowledgement,
)

from .common import require_humble_python310


class ExecutionBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("unloading_execution_bridge")
        self.declare_parameter("enable_hardware", False)
        self.declare_parameter("controller_action", "/mock_controller/follow_joint_trajectory")
        self.declare_parameter("expected_joint_names", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        if bool(self.get_parameter("enable_hardware").value):
            raise RuntimeError("enable_hardware=true is refused: no verified FANUC hardware adapter is supplied")
        self.callback_group = ReentrantCallbackGroup()
        self.client = ActionClient(self, FollowJointTrajectory, str(self.get_parameter("controller_action").value), callback_group=self.callback_group)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.events = self.create_publisher(ExecutionEvent, "/unloading/execution_events", qos)
        self.stop_acknowledgements = self.create_publisher(StopAcknowledgement, "/unloading/stop_acknowledgements", qos)
        self.create_subscription(ExecutionAuthorization, "/unloading/execution_authorization", self.on_command, qos, callback_group=self.callback_group)
        self.create_subscription(ExecutionCancel, "/unloading/execution_cancel", self.on_cancel, qos, callback_group=self.callback_group)
        self.create_subscription(ControllerStopFact, "/unloading/controller_stop_facts", self.on_stop_fact, qos, callback_group=self.callback_group)
        self.seen = set()
        self.commands = {}
        self.handles = {}
        self.awaiting_stop = set()

    def reject(self, message, reason):
        event = ExecutionEvent(command_id=message.command_id, plan_id=message.plan_id, epoch=message.epoch, planning_generation=message.planning_generation, kind="REJECTED", event_time=self.get_clock().now().to_msg(), clock_domain="ros", message=reason)
        self.events.publish(event)

    def on_command(self, message: ExecutionAuthorization) -> None:
        if message.command_id in self.seen:
            self.reject(message, "DUPLICATE_COMMAND")
            return
        if self.commands:
            self.reject(message, "CONTROLLER_BUSY")
            return
        expected = list(self.get_parameter("expected_joint_names").value)
        if list(message.trajectory.joint_names) != expected:
            self.reject(message, "JOINT_ORDER_MISMATCH")
            return
        times = [point.time_from_start.sec + point.time_from_start.nanosec / 1e9 for point in message.trajectory.points]
        if not times or any(later <= earlier for earlier, later in zip(times, times[1:])):
            self.reject(message, "INVALID_TIMED_TRAJECTORY")
            return
        if not message.validation_reference or not message.world_fingerprint:
            self.reject(message, "EXECUTION_AUTHORIZATION_INCOMPLETE")
            return
        if not self.client.server_is_ready():
            self.reject(message, "MOCK_CONTROLLER_UNAVAILABLE")
            return
        self.seen.add(message.command_id)
        self.commands[message.command_id] = message
        goal = FollowJointTrajectory.Goal(trajectory=message.trajectory)
        future = self.client.send_goal_async(goal, feedback_callback=lambda feedback: self.on_feedback(message, feedback))
        future.add_done_callback(lambda result: self.on_goal_response(message, result))

    def on_feedback(self, command, feedback) -> None:
        event = ExecutionEvent(command_id=command.command_id, plan_id=command.plan_id, epoch=command.epoch, planning_generation=command.planning_generation, kind="FEEDBACK", event_time=self.get_clock().now().to_msg(), clock_domain="ros", actual_positions=list(feedback.feedback.actual.positions), actual_velocities=list(feedback.feedback.actual.velocities), message="mock controller feedback")
        self.events.publish(event)

    def on_goal_response(self, command, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.reject(command, "CONTROLLER_REJECTED")
            return
        event = ExecutionEvent(command_id=command.command_id, plan_id=command.plan_id, epoch=command.epoch, planning_generation=command.planning_generation, kind="ACCEPTED", event_time=self.get_clock().now().to_msg(), clock_domain="ros", message="mock controller accepted")
        self.events.publish(event)
        self.handles[command.command_id] = handle
        handle.get_result_async().add_done_callback(lambda result: self.on_result(command, result))

    def on_result(self, command, future) -> None:
        result = future.result()
        kind = "SUCCEEDED" if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL else "FAILED"
        event = ExecutionEvent(command_id=command.command_id, plan_id=command.plan_id, epoch=command.epoch, planning_generation=command.planning_generation, kind=kind, event_time=self.get_clock().now().to_msg(), clock_domain="ros", message=result.result.error_string)
        self.events.publish(event)
        if result.status != GoalStatus.STATUS_CANCELED:
            self.handles.pop(command.command_id, None)
            self.commands.pop(command.command_id, None)

    def on_cancel(self, request: ExecutionCancel) -> None:
        command = self.commands.get(request.command_id)
        handle = self.handles.get(request.command_id)
        if (
            command is None
            or handle is None
            or request.plan_id != command.plan_id
            or request.epoch != command.epoch
            or request.planning_generation != command.planning_generation
        ):
            self.reject(request, "CANCEL_IDENTITY_MISMATCH")
            return
        self.awaiting_stop.add(command.command_id)
        future = handle.cancel_goal_async()
        future.add_done_callback(lambda result: self.on_cancel_response(command, result))

    def on_cancel_response(self, command, future) -> None:
        if not future.result().goals_canceling:
            self.awaiting_stop.discard(command.command_id)
            self.reject(command, "CONTROLLER_CANCEL_REJECTED")
            return
        event = ExecutionEvent(command_id=command.command_id, plan_id=command.plan_id, epoch=command.epoch, planning_generation=command.planning_generation, kind="CANCEL_ACCEPTED", event_time=self.get_clock().now().to_msg(), clock_domain="ros", message="controller accepted cancellation; physical stop not yet confirmed")
        self.events.publish(event)

    def on_stop_fact(self, fact: ControllerStopFact) -> None:
        if len(self.awaiting_stop) != 1:
            self.get_logger().error("cannot bind controller stop fact: expected exactly one pending cancellation")
            return
        command_id = next(iter(self.awaiting_stop))
        command = self.commands[command_id]
        if not fact.actual_velocities or any(abs(value) > 1e-3 for value in fact.actual_velocities):
            self.reject(command, "STOP_FACT_NOT_STATIONARY")
            return
        acknowledgement = StopAcknowledgement(
            command_id=command.command_id,
            plan_id=command.plan_id,
            epoch=command.epoch,
            planning_generation=command.planning_generation,
            stopped_time=fact.stopped_time,
            clock_domain=fact.clock_domain,
            actual_positions=list(fact.actual_positions),
            actual_velocities=list(fact.actual_velocities),
            criterion=fact.criterion,
            evidence_reference=fact.evidence_reference,
        )
        self.stop_acknowledgements.publish(acknowledgement)
        self.awaiting_stop.remove(command_id)
        self.handles.pop(command_id, None)
        self.commands.pop(command_id, None)


def main(args=None) -> None:
    require_humble_python310()
    rclpy.init(args=args)
    node = ExecutionBridgeNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
