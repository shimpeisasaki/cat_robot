"""Offline map and waypoint editing. No robot, TF source or navigation servers."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def setup(context):
    map_file = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    output = Path(LaunchConfiguration('output').perform(context)).expanduser().resolve()
    source = LaunchConfiguration('input').perform(context)
    if not map_file.is_file():
        raise ValueError(f'Map YAML does not exist: {map_file}')
    if output == map_file:
        raise ValueError('output must be a route file, not the map YAML')
    if source:
        source = str(Path(source).expanduser().resolve())
        if not Path(source).is_file():
            raise ValueError(f'Input route YAML does not exist: {source}')
    return [
        Node(package='nav2_map_server', executable='map_server', name='waypoint_map_server',
             parameters=[{'use_sim_time': False, 'frame_id': 'map', 'yaml_filename': str(map_file)}],
             remappings=[('map', '/waypoint_editor/map')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='waypoint_map_lifecycle_manager',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['waypoint_map_server']}], output='screen'),
        Node(package='cat_navigation', executable='waypoint_editor', name='waypoint_editor',
             parameters=[{'input': source, 'output': str(output)}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='waypoint_editor_rviz',
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('cat_navigation'), 'rviz', 'waypoint_editor.rviz'])],
             output='screen'),
    ]


def generate_launch_description():
    share = FindPackageShare('cat_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('map',
                              default_value=PathJoinSubstitution([share, 'map', '19F_260928_Nav.yaml']),
                              description='Navigation map YAML to display'),
        DeclareLaunchArgument('output',
                              default_value=PathJoinSubstitution([share, 'map', '19F_waypoint.yaml']),
                              description='Route YAML written when ~/save is called'),
        DeclareLaunchArgument('input',
                              default_value=PathJoinSubstitution([share, 'map', '19F_waypoint.yaml']),
                              description='Existing route to edit'),
        OpaqueFunction(function=setup),
    ])
