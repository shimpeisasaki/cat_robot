"""Robot hardware, sensors and command gate; independent of navigation."""

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
        DeclareLaunchArgument('use_gnss', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gnss_serial', default_value=''),
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('require_navigation', default_value='false'),
        DeclareLaunchArgument('enable_motor_tools', default_value='false'),
        DeclareLaunchArgument('use_lidar', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_joystick', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('serial_number', default_value='10028118'),
        DeclareLaunchArgument('robot_config', default_value=PathJoinSubstitution([
            share, 'config', 'robot.yaml'])),
        DeclareLaunchArgument('manual_config', default_value=PathJoinSubstitution([
            share, 'config', 'manual_control.yaml'])),
        DeclareLaunchArgument('zed_config', default_value=PathJoinSubstitution([
            share, 'config', PythonExpression(["'zed_vio.yaml' if '",
                LaunchConfiguration('odom_source'), "' == 'vio' else 'zed_sensors.yaml'"])])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', model]), value_type=str)}], output='screen'),
        Node(package='ddsm115_controller', executable='base_driver',
             name='base_driver', parameters=[robot_config, {
                 'require_safety_heartbeat': True,
                 'enable_motor_tools': ParameterValue(LaunchConfiguration('enable_motor_tools'), value_type=bool)}],
             output='screen', respawn=True, respawn_delay=2.0,
             condition=IfCondition(LaunchConfiguration('use_base'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'odometry.launch.py'])),
            launch_arguments={'odom_source': LaunchConfiguration('odom_source')}.items(),
            condition=IfCondition(LaunchConfiguration('use_base'))),
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             parameters=[{'autorepeat_rate': 20.0}],
             condition=IfCondition(LaunchConfiguration('enable_joystick'))),
        Node(package='ddsm115_controller', executable='curvature_teleop',
             name='curvature_teleop', parameters=[manual_config], output='screen',
             remappings=[('/cmd_vel_teleop', '/cmd_vel_teleop_raw')],
             condition=IfCondition(LaunchConfiguration('enable_joystick'))),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother_manual', parameters=[manual_config], output='screen',
             remappings=[('cmd_vel', '/cmd_vel_teleop_raw'), ('cmd_vel_smoothed', '/cmd_vel_teleop')],
             condition=IfCondition(LaunchConfiguration('use_base'))),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='manual_velocity_smoother_lifecycle_manager', output='screen',
             parameters=[{'autostart': True, 'node_names': ['velocity_smoother_manual']}],
             condition=IfCondition(LaunchConfiguration('use_base'))),
        Node(package='cat_bringup', executable='base_safety', name='base_safety',
             parameters=[{
                 'enable_joystick': ParameterValue(LaunchConfiguration('enable_joystick'), value_type=bool),
                 'require_navigation': ParameterValue(LaunchConfiguration('require_navigation'), value_type=bool)}],
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
            launch_arguments={'serial_port': LaunchConfiguration('serial_port')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ublox_dgnss'), 'launch', 'gnss_launch_compatible.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_gnss')),
            launch_arguments={'frame_id': 'gnss_antenna_link',
                              'device_serial_string': LaunchConfiguration('gnss_serial')}.items()),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([
                 share, 'rviz', 'experiment_robot.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
