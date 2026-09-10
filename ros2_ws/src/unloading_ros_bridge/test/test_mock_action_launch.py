import time
import unittest

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import launch_testing.markers
import pytest
import rclpy
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from unloading_interfaces.msg import ControllerStopFact


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        Node(package="unloading_ros_bridge", executable="mock_follow_joint_trajectory", output="screen", parameters=[{"controller_epoch": "launch-test-controller-epoch"}]),
        ReadyToTest(),
    ])


class TestMockFollowJointTrajectory(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("mock_action_launch_test")
        cls.client = ActionClient(cls.node, FollowJointTrajectory, "/mock_controller/follow_joint_trajectory")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_feedback_cancel_and_separate_stop_fact(self):
        assert self.client.wait_for_server(timeout_sec=5.0)
        stop_facts = []
        subscription = self.node.create_subscription(ControllerStopFact, "/unloading/controller_stop_facts", stop_facts.append, 10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_1", "joint_2"]
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[index / 100.0, index / 100.0], time_from_start=Duration(sec=0, nanosec=(index + 1) * 10_000_000))
            for index in range(20)
        ]
        feedback = []
        future = self.client.send_goal_async(goal, feedback_callback=feedback.append)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        handle = future.result()
        assert handle.accepted
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=5.0)
        assert cancel.result().goals_canceling
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=5.0)
        deadline = time.monotonic() + 2.0
        while not stop_facts and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(subscription)
        assert feedback
        assert stop_facts
        assert stop_facts[0].criterion == "mock measured velocity is zero"
        assert stop_facts[0].controller_epoch == "launch-test-controller-epoch"
        assert stop_facts[0].goal_id == bytes(handle.goal_id.uuid).hex()
        assert stop_facts[0].actual_positions != list(goal.trajectory.points[-1].positions)
