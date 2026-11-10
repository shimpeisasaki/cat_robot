"""Standalone Livox Horizon point cloud and IMU, independent of the base."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def setup(context):
    values = context.launch_configurations
    mapped = values['timestamp_mode'] == 'host_offset'
    if values['timestamp_mode'] not in ('sensor', 'host_offset'):
        raise ValueError('timestamp_mode must be sensor or host_offset')
    share = FindPackageShare('cat_bringup')
    nodes = [
        Node(
            package='livox_ros2_driver', executable='livox_ros2_driver_node',
            name='horizon_driver', output='screen',
            sigterm_timeout='8',
            remappings=[('livox/imu', '/livox/imu_sensor_raw' if mapped else '/livox/imu_raw'),
                        ('livox/lidar', '/livox/lidar_sensor_raw' if mapped else '/livox/lidar')],
            parameters=[LaunchConfiguration('config'), {
                'user_config_path': LaunchConfiguration('user_config'),
            }]),
        Node(
            package='cat_bringup', executable='horizon_imu',
            name='horizon_imu', output='screen',
            additional_env={'PYTHONNOUSERSITE': '1'},
            parameters=[LaunchConfiguration('imu_calibration')]),
        Node(
            package='rviz2', executable='rviz2', name='horizon_rviz', output='screen',
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'horizon.rviz'])],
            condition=IfCondition(LaunchConfiguration('show_rviz'))),
    ]
    if mapped:
        nodes.append(Node(package='cat_bringup', executable='horizon_time_bridge',
                          output='screen', additional_env={'PYTHONNOUSERSITE': '1'}))
    return nodes


def generate_launch_description():
    share = FindPackageShare('cat_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            share, 'config', 'horizon.yaml'])),
        DeclareLaunchArgument('user_config', default_value=PathJoinSubstitution([
            share, 'config', 'horizon.json'])),
        DeclareLaunchArgument('show_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('timestamp_mode', default_value='sensor',
                              choices=['sensor', 'host_offset']),
        DeclareLaunchArgument('imu_calibration', default_value=PathJoinSubstitution([
            share, 'config', 'horizon_imu.yaml'])),
        OpaqueFunction(function=setup),
    ])
