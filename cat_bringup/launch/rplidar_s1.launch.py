"""RPLIDAR S1 with in-driver vehicle-sector filtering and direct /scan output."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('front_blind_rear_angle_deg', default_value='90.0'),
        DeclareLaunchArgument('frame_id', default_value='RPLIDAR_F',
                              choices=['RPLIDAR_F', 'RPLIDAR_custom']),
        DeclareLaunchArgument('mounting_roll', default_value='0'),
        DeclareLaunchArgument('mounting_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('filter_config', default_value=PathJoinSubstitution([
            FindPackageShare('cat_bringup'), 'config', 'scan_filter.yaml'])),
        Node(
            package='sllidar_ros2', executable='sllidar_node',
            name='sllidar_node', output='screen',
            parameters=[LaunchConfiguration('filter_config'),
                {'channel_type': 'serial',
                 'serial_port': LaunchConfiguration('serial_port'),
                 'serial_baudrate': 256000,
                 'frame_id': LaunchConfiguration('frame_id'),
                 'inverted': False,
                 'angle_compensate': True,
                 'mounting_roll': ParameterValue(LaunchConfiguration('mounting_roll'),
                                                 value_type=float),
                 'mounting_yaw': ParameterValue(LaunchConfiguration('mounting_yaw'),
                                                value_type=float),
                 'blind_rear_angle_deg': ParameterValue(PythonExpression([
                     LaunchConfiguration('front_blind_rear_angle_deg'),
                     " if '", LaunchConfiguration('frame_id'),
                     "' == 'RPLIDAR_F' else 0.0"]), value_type=float),
                 'rear_min_range': ParameterValue(PythonExpression([
                     "0.6 if '", LaunchConfiguration('frame_id'),
                     "' == 'RPLIDAR_F' else 0.2"]), value_type=float)}]),
    ])
