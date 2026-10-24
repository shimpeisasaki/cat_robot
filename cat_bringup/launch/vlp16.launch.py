"""VLP-16 packet and point-cloud bringup, independent of the robot base."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    calibration = PathJoinSubstitution([
        FindPackageShare('velodyne_pointcloud'), 'params', 'VLP16db.yaml'])
    power_manager = Node(package='cat_bringup', executable='vlp16_power_manager',
                         name='vlp16_power_manager', output='screen',
                         parameters=[config], additional_env={'PYTHONNOUSERSITE': '1'})
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('cat_bringup'), 'config', 'vlp16.yaml'])),
        DeclareLaunchArgument('show_rviz', default_value='false', choices=['true', 'false']),
        RegisterEventHandler(OnProcessExit(
            target_action=power_manager,
            on_exit=[EmitEvent(event=Shutdown())])),
        power_manager,
        Node(package='velodyne_driver', executable='velodyne_driver_node',
             name='velodyne_driver_node', output='screen', parameters=[config]),
        Node(package='velodyne_pointcloud', executable='velodyne_transform_node',
             name='velodyne_transform_node', output='screen',
             parameters=[config, {'calibration': calibration}],
             remappings=[('velodyne_points', 'velodyne_points_raw')]),
        Node(package='cat_bringup', executable='vlp16_sector_filter',
             name='vlp16_sector_filter', output='screen', parameters=[config],
             additional_env={'PYTHONNOUSERSITE': '1'}),
        Node(package='rviz2', executable='rviz2', name='vlp16_rviz', output='screen',
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('cat_bringup'), 'rviz', 'vlp16.rviz'])],
             condition=IfCondition(LaunchConfiguration('show_rviz'))),
    ])
