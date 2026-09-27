"""Standalone maintenance GUI. Stop normal bringup before using this tool."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('cat_bringup'), 'config', 'robot.yaml'])),
        Node(package='joy', executable='joy_node', name='joy_node'),
        Node(package='ddsm115_controller', executable='base_driver', name='base_driver',
             parameters=[config, {'enable_motor_tools': True, 'require_safety_heartbeat': False}]),
        Node(package='ddsm115_controller', executable='motor_test_gui', name='motor_test_gui',
             parameters=[config, {'config_file': config}]),
    ])
