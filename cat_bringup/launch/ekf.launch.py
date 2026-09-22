"""Continuous wheel/gyro EKF, including IMU freshness monitoring."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ekf_output', default_value='/odom'),
        DeclareLaunchArgument('ekf_frame', default_value='odom'),
        DeclareLaunchArgument('ekf_publish_tf', default_value='true'),
        Node(package='cat_bringup', executable='imu_guard', output='screen',
             parameters=[{'max_gyro_magnitude': 3.0}]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             parameters=[PathJoinSubstitution([
                 FindPackageShare('cat_bringup'), 'config', 'ekf.yaml']),
                 {'publish_tf': ParameterValue(LaunchConfiguration('ekf_publish_tf'), value_type=bool),
                  'odom_frame': LaunchConfiguration('ekf_frame'),
                  'world_frame': LaunchConfiguration('ekf_frame')}],
             remappings=[('odometry/filtered', LaunchConfiguration('ekf_output'))]),
    ])
