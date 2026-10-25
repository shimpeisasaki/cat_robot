"""Start only the terrain detector; a separate bringup owns the LiDAR and TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    fixed_frame = LaunchConfiguration('fixed_frame')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('cat_terrain'), 'config', 'terrain.yaml'])),
        DeclareLaunchArgument('fixed_frame', default_value='odom'),
        DeclareLaunchArgument('show_rviz', default_value='false', choices=['true', 'false']),
        Node(package='cat_terrain', executable='terrain_step_detector',
             name='terrain_step_detector',
             parameters=[config, {'fixed_frame': fixed_frame}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='terrain_rviz', output='screen',
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('cat_terrain'), 'rviz', 'terrain.rviz'])],
             condition=IfCondition(LaunchConfiguration('show_rviz'))),
    ])
