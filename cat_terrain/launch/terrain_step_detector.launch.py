"""Start only the terrain detector; a separate bringup owns the LiDAR and TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('cat_terrain'), 'config', 'terrain.yaml'])),
        Node(package='cat_terrain', executable='terrain_step_detector',
             name='terrain_step_detector', parameters=[config], output='screen'),
    ])
