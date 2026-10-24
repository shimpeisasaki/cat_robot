"""VLP-16 packet and point-cloud bringup, independent of the robot base."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    calibration = PathJoinSubstitution([
        FindPackageShare('velodyne_pointcloud'), 'params', 'VLP16db.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('cat_bringup'), 'config', 'vlp16.yaml'])),
        DeclareLaunchArgument('show_rviz', default_value='false', choices=['true', 'false']),
        Node(package='velodyne_driver', executable='velodyne_driver_node',
             name='velodyne_driver_node', output='screen', parameters=[config]),
        Node(package='velodyne_pointcloud', executable='velodyne_transform_node',
             name='velodyne_transform_node', output='screen',
             parameters=[config, {'calibration': calibration}]),
        Node(package='rviz2', executable='rviz2', name='vlp16_rviz', output='screen',
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('cat_bringup'), 'rviz', 'vlp16.rviz'])],
             condition=IfCondition(LaunchConfiguration('show_rviz'))),
    ])
