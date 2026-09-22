"""Start the robot's RPLIDAR S1 standalone, optionally with RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
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
        # Do not call this simply "rviz": parent launches may start their own
        # RViz and pass a different value to this included launch.
        DeclareLaunchArgument('lidar_rviz', default_value='true', choices=['true', 'false']),
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
        Node(
            package='rviz2', executable='rviz2', name='rplidar_rviz',
            arguments=['-d', PathJoinSubstitution([
                sllidar_share, 'rviz', 'sllidar_ros2.rviz'])],
            output='screen', condition=IfCondition(LaunchConfiguration('lidar_rviz'))),
    ])
