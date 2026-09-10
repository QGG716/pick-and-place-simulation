from __future__ import annotations

from dataclasses import replace
import time

from builtin_interfaces.msg import Duration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import launch_testing.markers
import pytest
import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from unloading_contracts import ExecutionCommand, PlanArtifactKind, TimedJointPoint, TimedJointTrajectory
from unloading_interfaces.msg import (
    ExecutionAuthorization, ExecutionCancel, ExecutionContext, ExecutionEvent,
    ExecutionGrant, PerceptionObservation, PlanningWorldSnapshot, StopAcknowledgement,
)
from unloading_perception.demo import _synthetic_observation
from unloading_ros_bridge.mapping import observation_to_msg, snapshot_from_msg


CONTROLLER_EPOCH = "integration-controller-epoch"


@pytest.mark.launch_test
def generate_test_description():
    common = {
        "expected_joint_names": [f"joint_{i}" for i in range(1, 7)],
        "snapshot_freshness_seconds": 5.0, "robot_state_freshness_seconds": 5.0,
    }
    return LaunchDescription([
        Node(package="unloading_ros_bridge", executable="mock_follow_joint_trajectory", output="screen", parameters=[{"controller_epoch": CONTROLLER_EPOCH}]),
        Node(package="unloading_ros_bridge", executable="mock_state_publisher", output="screen"),
        Node(package="unloading_ros_bridge", executable="world_bridge_node", output="screen", parameters=[common]),
        Node(package="unloading_ros_bridge", executable="execution_bridge_node", output="screen", parameters=[{"controller_epoch": CONTROLLER_EPOCH}]),
        ReadyToTest(),
    ])


class TestBridgeIntegration:
    @classmethod
    def setup_class(cls):
        rclpy.init()
        cls.node = rclpy.create_node("bridge_integration_test")

    @classmethod
    def teardown_class(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, values, predicate=lambda value: True, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            for value in values:
                if predicate(value):
                    return value
        raise AssertionError("timed out waiting for ROS integration output")

    def test_world_gate_action_cancel_and_correlated_stop(self):
        snapshots, events, stops = [], [], []
        self.node.create_subscription(PlanningWorldSnapshot, "/unloading/world_snapshot", snapshots.append, 10)
        self.node.create_subscription(ExecutionEvent, "/unloading/execution_events", events.append, 10)
        self.node.create_subscription(StopAcknowledgement, "/unloading/stop_acknowledgements", stops.append, 10)
        joints_pub = self.node.create_publisher(JointState, "/joint_states", 10)
        perception_pub = self.node.create_publisher(PerceptionObservation, "/unloading/perception", 10)
        context_pub = self.node.create_publisher(ExecutionContext, "/unloading/execution_context", 10)
        grant_pub = self.node.create_publisher(ExecutionGrant, "/unloading/execution_grant", 10)
        command_pub = self.node.create_publisher(ExecutionAuthorization, "/unloading/execution_authorization", 10)
        cancel_pub = self.node.create_publisher(ExecutionCancel, "/unloading/execution_cancel", 10)
        time.sleep(1.0)

        now = self.node.get_clock().now().to_msg()
        joints = JointState(name=[f"joint_{i}" for i in range(1, 7)], position=[0.0] * 6, velocity=[0.0] * 6)
        joints.header.stamp = now
        joints_pub.publish(joints)
        capture = time.time()
        observation = replace(_synthetic_observation(), capture_time=capture, processed_time=capture + 0.001, clock_domain="ros")
        perception_pub.publish(observation_to_msg(observation))
        world_message = self.wait_for(snapshots, lambda item: item.planning_admissible)
        world = snapshot_from_msg(world_message)

        points = tuple(TimedJointPoint((index / 100.0,) * 6, (index + 1) / 100.0, (0.0,) * 6) for index in range(30))
        trajectory = TimedJointTrajectory(tuple(f"joint_{i}" for i in range(1, 7)), points, PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY, "mock-validator@1", "synthetic-robot-v1", "synthetic_metric_v1")
        command = ExecutionCommand("cmd-integration", "plan-integration", "request-integration", "session-integration", "execution-epoch", 4, None, world.fingerprint, "synthetic-robot-v1", "synthetic_metric_v1", "mock-validator@1", 4, trajectory)

        context_pub.publish(ExecutionContext(schema_version="1.1.0", session_id=command.session_id, epoch=command.epoch, planning_generation=command.planning_generation, allowed_plan_id=command.plan_id, predecessor_plan_id="", observed_time=self.node.get_clock().now().to_msg(), clock_domain="ros"))
        expiry = self.node.get_clock().now().nanoseconds / 1e9 + 10.0
        expiry_msg = self.node.get_clock().now().to_msg()
        expiry_msg.sec, expiry_msg.nanosec = int(expiry), int((expiry - int(expiry)) * 1e9)
        grant_pub.publish(ExecutionGrant(schema_version="1.1.0", grant_id="grant-integration", command_id=command.command_id, plan_id=command.plan_id, request_id=command.request_id, session_id=command.session_id, epoch=command.epoch, planning_generation=command.planning_generation, predecessor_plan_id="", world_fingerprint=command.world_fingerprint, robot_model_fingerprint=command.robot_model_fingerprint, config_identity=command.config_identity, validation_reference=command.validation_reference, validation_generation=command.validation_generation, trajectory_fingerprint=command.trajectory_fingerprint, expires_at=expiry_msg, clock_domain="ros", mock_only=True))
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        ros_points = [JointTrajectoryPoint(positions=list(point.positions), velocities=list(point.velocities), time_from_start=Duration(sec=int(point.time_from_start), nanosec=int((point.time_from_start % 1.0) * 1e9))) for point in points]
        authorization = ExecutionAuthorization(schema_version="1.1.0", command_id=command.command_id, plan_id=command.plan_id, request_id=command.request_id, session_id=command.session_id, epoch=command.epoch, planning_generation=command.planning_generation, predecessor_plan_id="", world_fingerprint=command.world_fingerprint, robot_model_fingerprint=command.robot_model_fingerprint, config_identity=command.config_identity, validation_reference=command.validation_reference, validation_generation=command.validation_generation)
        authorization.trajectory.joint_names = list(trajectory.joint_names)
        authorization.trajectory.points = ros_points
        command_pub.publish(authorization)
        self.wait_for(events, lambda item: item.command_id == command.command_id and item.kind == "STARTED")

        cancel_pub.publish(ExecutionCancel(schema_version="1.1.0", command_id=command.command_id, plan_id=command.plan_id, epoch=command.epoch, planning_generation=command.planning_generation, reason="integration cancellation"))
        self.wait_for(events, lambda item: item.command_id == command.command_id and item.kind == "CANCEL_ACCEPTED")
        self.wait_for(events, lambda item: item.command_id == command.command_id and item.kind == "CANCELED")
        stopped = self.wait_for(stops, lambda item: item.command_id == command.command_id)
        assert stopped.plan_id == command.plan_id
        assert stopped.controller_epoch == CONTROLLER_EPOCH
        assert stopped.goal_id
        assert stopped.stop_sequence > 0
        assert all(abs(value) <= 1e-3 for value in stopped.actual_velocities)
        assert not any(item.command_id == command.command_id and item.kind == "SUCCEEDED" for item in events)
