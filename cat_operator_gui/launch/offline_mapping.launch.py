"""SLAM, filtered TF and RViz; the GUI manages bag playback as a separate job."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('cat_operator_gui')
    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='cat_operator_gui', executable='offline_tf_filter',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
             parameters=[LaunchConfiguration('slam_params_file'), {
                 'use_sim_time': True, 'base_frame': 'base_link', 'map_frame': 'map',
                 'odom_frame': 'odom', 'scan_topic': '/scan', 'tf_buffer_duration': 120.0}],
             output='screen'),
        Node(package='rviz2', executable='rviz2', name='offline_mapping_rviz',
             arguments=['-d', PathJoinSubstitution([share, 'config', 'offline_mapping.rviz'])],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
