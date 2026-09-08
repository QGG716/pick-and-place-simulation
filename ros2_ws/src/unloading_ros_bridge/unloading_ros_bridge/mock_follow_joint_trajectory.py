from __future__ import annotations

import asyncio

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from unloading_interfaces.msg import ControllerStopFact

from .common import require_humble_python310


class MockTrajectoryServer(Node):
    def __init__(self) -> None:
        super().__init__("mock_follow_joint_trajectory")
        self.stop_publisher = self.create_publisher(ControllerStopFact, "/unloading/controller_stop_facts", 10)
        self.server = ActionServer(self, FollowJointTrajectory, "/mock_controller/follow_joint_trajectory", execute_callback=self.execute, goal_callback=self.goal, cancel_callback=self.cancel)

    def goal(self, request):
        points = request.trajectory.points
        times = [point.time_from_start.sec + point.time_from_start.nanosec / 1e9 for point in points]
        valid = bool(request.trajectory.joint_names and points and all(later > earlier for earlier, later in zip(times, times[1:])))
        return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    def cancel(self, _goal_handle):
        # Acceptance only means cancellation was requested. The execute loop
        # publishes the separate measured stop fact after it observes cancel.
        return CancelResponse.ACCEPT

    async def execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        for point in goal_handle.request.trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "cancel accepted and mock controller stopped"
                acknowledgement = ControllerStopFact(stopped_time=self.get_clock().now().to_msg(), clock_domain="ros", joint_names=list(goal_handle.request.trajectory.joint_names), actual_positions=list(point.positions), actual_velocities=[0.0] * len(point.positions), criterion="mock measured velocity is zero", evidence_reference="simulation-feedback")
                self.stop_publisher.publish(acknowledgement)
                return result
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(goal_handle.request.trajectory.joint_names)
            feedback.actual.positions = list(point.positions)
            feedback.actual.velocities = list(point.velocities) if point.velocities else [0.0] * len(point.positions)
            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(0.01)
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "mock trajectory complete"
        return result


def main(args=None) -> None:
    require_humble_python310()
    rclpy.init(args=args)
    node = MockTrajectoryServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
