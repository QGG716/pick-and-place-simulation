"""One persistent display graph for a finite, serial algorithm completion list."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('progress'),DeclareLaunchArgument('receipts'),
        Node(package='unloading_ros_bridge',executable='mock_state_publisher',output='screen'),
        Node(package='unloading_ros_bridge',executable='finite_replay_node',output='screen',
             parameters=[{'progress':LaunchConfiguration('progress'),'receipts':LaunchConfiguration('receipts')}]),
        Node(package='unloading_ros_bridge',executable='world_bridge_node',output='screen',
             parameters=[{'observation_mode':'replay_display_only','use_sim_time':False}]),
    ])
