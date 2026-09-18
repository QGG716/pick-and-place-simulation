"""One pinned algorithm artifact, read-only world, explicitly mock display states."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('artifact'), DeclareLaunchArgument('artifact_sha256'),
        Node(package='unloading_ros_bridge', executable='mock_state_publisher', output='screen',
             parameters=[{'use_sim_time': False}]),
        Node(package='unloading_ros_bridge', executable='perception_node', output='screen',
             parameters=[{'backend': 'algorithm_replay', 'replay_path': LaunchConfiguration('artifact'),
                          'artifact_sha256': ParameterValue(LaunchConfiguration('artifact_sha256'), value_type=str),
                          'use_sim_time': False}]),
        Node(package='unloading_ros_bridge', executable='world_bridge_node', output='screen',
             parameters=[{'observation_mode': 'replay_display_only', 'use_sim_time': False}]),
    ])
