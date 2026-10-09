"""Internal RPLIDAR S1 driver and vehicle-sector filter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        GroupAction(actions=[
          SetRemap(src='/scan', dst='/scan_raw'),
          Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
               parameters=[{'channel_type': 'serial',
                            'serial_port': LaunchConfiguration('serial_port'),
                            'serial_baudrate': 256000,
                            'frame_id': LaunchConfiguration('frame_id'),
                            'inverted': False, 'angle_compensate': True}],
               arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
               output='screen'),
        ]),
        Node(
            package='cat_bringup', executable='sector_scan_filter',
            name='sector_scan_filter', output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[PathJoinSubstitution([
                FindPackageShare('cat_bringup'), 'config', 'scan_filter.yaml']),
                {'scan_frame': LaunchConfiguration('frame_id')}]),
    ])
