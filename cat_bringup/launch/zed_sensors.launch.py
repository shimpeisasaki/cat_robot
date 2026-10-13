"""Shared ZED Mini configuration for sensor bringup and calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Select the ZED Mini permanently mounted on this robot.  A caller can
        # still override this argument when replacing the camera.
        DeclareLaunchArgument('serial_number', default_value='10028118'),
        DeclareLaunchArgument('zed_config', default_value=PathJoinSubstitution([
            FindPackageShare('cat_bringup'), 'config', 'zed_sensors.yaml'])),
        # The supervisor owns the entire child process group, including the
        # component container. Stop it before respawning another wrapper.
        Node(package='cat_bringup', executable='zed_supervisor',
            arguments=['--serial-number', LaunchConfiguration('serial_number'),
                       '--config', LaunchConfiguration('zed_config')],
            output='screen', respawn=True, respawn_delay=5.0),
    ])
