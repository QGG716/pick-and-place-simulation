from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    replay_path = LaunchConfiguration("replay_path")
    return LaunchDescription([
        DeclareLaunchArgument("replay_path"),
        Node(package="unloading_ros_bridge", executable="mock_follow_joint_trajectory", output="screen", parameters=[{"controller_epoch": "mock-controller-epoch-v1", "use_sim_time": False}]),
        Node(package="unloading_ros_bridge", executable="mock_state_publisher", output="screen", parameters=[{"use_sim_time": False}]),
        Node(package="unloading_ros_bridge", executable="perception_node", output="screen", parameters=[{"replay_path": replay_path, "use_sim_time": False}]),
        Node(package="unloading_ros_bridge", executable="world_bridge_node", output="screen", parameters=[{"use_sim_time": False}]),
        Node(package="unloading_ros_bridge", executable="execution_bridge_node", output="screen", parameters=[{"enable_hardware": False, "controller_epoch": "mock-controller-epoch-v1", "use_sim_time": False}]),
    ])
