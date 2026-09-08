from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    replay_path = LaunchConfiguration("replay_path")
    return LaunchDescription([
        DeclareLaunchArgument("replay_path"),
        Node(package="unloading_ros_bridge", executable="mock_follow_joint_trajectory", output="screen"),
        Node(package="unloading_ros_bridge", executable="perception_node", output="screen", parameters=[{"replay_path": replay_path, "use_sim_time": False}]),
        Node(package="unloading_ros_bridge", executable="world_bridge_node", output="screen", parameters=[{
            "robot_model_fingerprint": "fixture-robot", "world_model_fingerprint": "fixture-world",
            "tool_state_identity": "mock-tool-confirmed", "payload_state_identity": "mock-no-payload-confirmed",
            "base_state_identity": "mock-base-confirmed", "conveyor_state_identity": "mock-conveyor-stopped-confirmed",
            "config_identity": "synthetic-metric-demo-v1", "use_sim_time": False,
        }]),
        Node(package="unloading_ros_bridge", executable="execution_bridge_node", output="screen", parameters=[{"enable_hardware": False, "use_sim_time": False}]),
    ])
