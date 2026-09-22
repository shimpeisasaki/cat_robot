"""DDSM115 + ZED Mini sensor inspection, without Nav2 or GNSS."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('cat_bringup')
    robot_config = LaunchConfiguration('robot_config')
    manual_config = LaunchConfiguration('manual_config')
    model = PathJoinSubstitution([share, 'urdf', 'experiment_robot.urdf.xacro'])
    return LaunchDescription([
        DeclareLaunchArgument('use_base', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_zed', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('odom_source', default_value='vio', choices=['vio', 'wheel']),
        DeclareLaunchArgument('use_lidar', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_joystick', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('serial_number', default_value='10028118'),
        DeclareLaunchArgument('robot_config', default_value=PathJoinSubstitution([
            share, 'config', 'robot_nav2.yaml'])),
        DeclareLaunchArgument('manual_config', default_value=PathJoinSubstitution([
            share, 'config', 'manual_control.yaml'])),
        DeclareLaunchArgument('zed_config', default_value=PathJoinSubstitution([
            share, 'config', PythonExpression(["'zed_vio_test.yaml' if '",
                LaunchConfiguration('odom_source'), "' == 'vio' else 'zed_sensors.yaml'"])])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', model]), value_type=str)}], output='screen'),
        Node(package='ddsm115_controller', executable='velocity_control',
             name='velocity_control_node', parameters=[robot_config,
                 {'require_safety_heartbeat': True, 'safety_heartbeat_timeout': 0.3}], output='screen',
             respawn=True, respawn_delay=2.0,
             condition=IfCondition(LaunchConfiguration('use_base'))),
        Node(package='ddsm115_controller', executable='two_wheels_robot',
             name='two_wheels_robot_node',
             parameters=[robot_config, {
                 'enable_joystick': False,
                 'pub_tf': False}],
             remappings=[('/odom', '/wheel/odom'), ('/cmd_vel', '/cmd_vel_safe')],
             output='screen', condition=IfCondition(LaunchConfiguration('use_base'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'odometry.launch.py'])),
            launch_arguments={'odom_source': LaunchConfiguration('odom_source')}.items(),
            condition=IfCondition(LaunchConfiguration('use_base'))),
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             parameters=[{'autorepeat_rate': 20.0}],
             condition=IfCondition(LaunchConfiguration('enable_joystick'))),
        Node(package='ddsm115_controller', executable='curvature_teleop',
             name='curvature_teleop', parameters=[manual_config, {'safety_managed': True}], output='screen',
             condition=IfCondition(LaunchConfiguration('enable_joystick'))),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[manual_config], output='screen',
             remappings=[('cmd_vel', '/cmd_vel_selected'), ('cmd_vel_smoothed', '/cmd_vel_smoothed')],
             condition=IfCondition(LaunchConfiguration('use_base'))),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='manual_velocity_smoother_lifecycle_manager', output='screen',
             parameters=[{'autostart': True, 'node_names': ['velocity_smoother']}],
             condition=IfCondition(LaunchConfiguration('use_base'))),
        Node(package='cat_bringup', executable='command_mux', name='command_mux',
             parameters=[{'enable_joystick': ParameterValue(
                 LaunchConfiguration('enable_joystick'), value_type=bool)}],
             output='screen', condition=IfCondition(LaunchConfiguration('use_base'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'zed_sensors.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_zed')),
            launch_arguments={
                'serial_number': LaunchConfiguration('serial_number'),
                'zed_config': LaunchConfiguration('zed_config'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'rplidar_s1.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_lidar')),
            launch_arguments={'lidar_rviz': 'false'}.items()),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([
                 share, 'rviz', 'experiment_robot.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
