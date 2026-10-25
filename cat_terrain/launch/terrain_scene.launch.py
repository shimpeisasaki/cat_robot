"""Run only the synthetic VLP-16, isolated TF, detector, and optional RViz."""

import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def simulation_robot_description():
    model = (Path(get_package_share_directory('cat_bringup')) / 'urdf' /
             'experiment_robot.urdf.xacro')
    document = xacro.process_file(str(model), mappings={'use_zed': 'false'})
    root = ET.fromstring(document.toxml())
    root.set('name', 'sim_experiment_robot')
    for link in root.findall('link'):
        link.set('name', 'sim_' + link.get('name'))
    for joint in root.findall('joint'):
        for tag in ('parent', 'child'):
            element = joint.find(tag)
            element.set('link', 'sim_' + element.get('link'))
    return ET.tostring(root, encoding='unicode')


def generate_launch_description():
    share = FindPackageShare('cat_terrain')
    scene = LaunchConfiguration('scene')
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value=PathJoinSubstitution([
            share, 'scenes', 'flat_boxes.yaml'])),
        DeclareLaunchArgument('show_rviz', default_value='false',
                              choices=['true', 'false']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='sim_robot_state_publisher',
             parameters=[{'robot_description': simulation_robot_description()}],
             remappings=[('robot_description', '/sim/robot_description')],
             output='screen'),
        Node(package='cat_terrain', executable='terrain_pointcloud_generator',
             name='terrain_pointcloud_generator',
             parameters=[{'scene_file': scene}], output='screen'),
        Node(package='cat_terrain', executable='terrain_step_detector',
             name='sim_terrain_step_detector',
             parameters=[PathJoinSubstitution([share, 'config', 'terrain.yaml']), {
                 'cloud_topic': '/sim/velodyne_points',
                 'hazards_topic': '/sim/terrain/hazards',
                 'base_frame': 'sim_base_link',
                 'fixed_frame': 'sim_odom',
             }],
             remappings=[
                 ('/terrain/up_steps', '/sim/terrain/up_steps'),
                 ('/terrain/drop_offs', '/sim/terrain/drop_offs'),
                 ('/terrain/up_cells', '/sim/terrain/up_cells'),
                 ('/terrain/drop_cells', '/sim/terrain/drop_cells'),
                 ('/terrain/valid', '/sim/terrain/valid'),
                 ('/terrain/coverage_by_range', '/sim/terrain/coverage_by_range'),
             ], output='screen'),
        Node(package='rviz2', executable='rviz2', name='terrain_sim_rviz',
             arguments=['-d', PathJoinSubstitution([
                 share, 'rviz', 'terrain_sim.rviz'])],
             condition=IfCondition(LaunchConfiguration('show_rviz'))),
    ])
