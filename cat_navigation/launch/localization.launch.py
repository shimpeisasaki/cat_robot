"""Saved-map localization only; requires scan and odom/base/RPLIDAR TF externally."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def gnss_transform(context):
    if LaunchConfiguration('use_gnss_reset').perform(context).lower() != 'true':
        return []
    calibration = Path(LaunchConfiguration('gnss_map_params_file').perform(context)).expanduser()
    if not calibration.is_file():
        raise RuntimeError('GNSS reset requires an explicit gnss_map_params_file for the selected map')
    return [Node(package='gnss2map', executable='gauss_kruger_node',
                 parameters=[str(calibration), {
                     'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
                 output='screen')]


def generate_launch_description():
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to saved map YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_gnss_reset', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gnss_map_params_file', default_value=''),
        OpaqueFunction(function=gnss_transform),
        DeclareLaunchArgument('localization_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('cat_navigation'), 'config', 'emcl2.yaml'])),
        Node(package='nav2_map_server', executable='map_server', name='localization_map_server',
             parameters=[clock, {'yaml_filename': LaunchConfiguration('map')}],
             remappings=[('map', '/localization/map')], output='screen'),
        Node(package='emcl2', executable='emcl2_node', name='emcl2',
             parameters=[LaunchConfiguration('localization_params_file'), clock, {
                 'use_gnss_reset': ParameterValue(LaunchConfiguration('use_gnss_reset'), value_type=bool),
                 'gnss_antenna_frame': 'gnss_antenna_link'}],
             remappings=[('map', '/localization/map')], output='screen'),
        # emcl2 is an ordinary node; only map_server needs lifecycle activation.
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization',
             parameters=[clock, {'autostart': True, 'node_names': ['localization_map_server']}],
             output='screen'),
    ])
