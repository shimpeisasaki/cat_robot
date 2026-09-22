"""Independent wheel/IMU estimate plus a single selected navigation TF source."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('cat_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('odom_source', default_value='vio', choices=['vio', 'wheel']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'ekf.launch.py'])),
            launch_arguments={'ekf_output': '/wheel_odom', 'ekf_frame': 'wheel_odom',
                              'ekf_publish_tf': 'false'}.items()),
        Node(package='cat_bringup', executable='odom_source', name='odom_source',
             parameters=[{'source': LaunchConfiguration('odom_source')}], output='screen',
             additional_env={'PYTHONNOUSERSITE': '1'}),
    ])
