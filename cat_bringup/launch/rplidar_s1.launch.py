"""Internal RPLIDAR S1 driver and vehicle-sector filter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sllidar_share = FindPackageShare('sllidar_ros2')
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        GroupAction(actions=[
          SetRemap(src='/scan', dst='/scan_raw'),
          IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                sllidar_share, 'launch', 'sllidar_s1_launch.py'])),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': '256000',
                'frame_id': LaunchConfiguration('frame_id'),
                'inverted': 'false',
                'angle_compensate': 'true',
            }.items()),
        ]),
        Node(
            package='cat_bringup', executable='sector_scan_filter',
            name='sector_scan_filter', output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('cat_bringup'), 'config', 'scan_filter.yaml']),
                {'scan_frame': LaunchConfiguration('frame_id')}]),
    ])
