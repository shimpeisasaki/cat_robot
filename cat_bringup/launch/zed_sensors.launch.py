"""Shared ZED Mini configuration for sensor bringup and calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Select the ZED Mini permanently mounted on this robot.  A caller can
        # still override this argument when replacing the camera.
        DeclareLaunchArgument('serial_number', default_value='10028118'),
        DeclareLaunchArgument('zed_config', default_value=PathJoinSubstitution([
            FindPackageShare('cat_bringup'), 'config', 'zed_sensors.yaml'])),
        # Run the wrapper as a child launch process. A camera disconnect makes
        # its component container exit; respawn then starts a new container
        # that can discover the re-enumerated USB device.
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py',
                'camera_model:=zedm', 'camera_name:=zed',
                'node_name:=zed_node',
                PythonExpression([
                    "'serial_number:=' + '", LaunchConfiguration('serial_number'), "'"]),
                'publish_urdf:=false', 'publish_tf:=false',
                'publish_map_tf:=false', 'publish_imu_tf:=true',
                'enable_gnss:=false', 'enable_ipc:=false', 'use_sim_time:=false',
                PythonExpression([
                    "'ros_params_override_path:=' + '", LaunchConfiguration('zed_config'), "'"]),
            ],
            output='screen', respawn=True, respawn_delay=5.0),
    ])
