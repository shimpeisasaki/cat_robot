"""Saved-map localization only; requires scan and odom/base/RPLIDAR TF externally."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to saved map YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('cat_navigation'), 'config', 'emcl2.yaml'])),
        Node(package='nav2_map_server', executable='map_server', name='localization_map_server',
             parameters=[clock, {'yaml_filename': LaunchConfiguration('map')}],
             remappings=[('map', '/localization/map')], output='screen'),
        Node(package='emcl2', executable='emcl2_node', name='emcl2',
             parameters=[LaunchConfiguration('localization_params_file'), clock],
             remappings=[('map', '/localization/map')], output='screen'),
        # emcl2 is an ordinary node; only map_server needs lifecycle activation.
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization',
             parameters=[clock, {'autostart': True, 'node_names': ['localization_map_server']}],
             output='screen'),
    ])
