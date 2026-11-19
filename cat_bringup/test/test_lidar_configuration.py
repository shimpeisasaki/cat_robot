"""Pure launch expansion; never starts a node or opens serial devices."""
import math
import os
from pathlib import Path
import runpy
import tempfile
import unittest
from unittest.mock import patch
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


class LidarConfigurationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # launch caches its file logger; retain its directory for the whole suite.
        cls.log_dir = tempfile.TemporaryDirectory()
        cls.environment = patch.dict(os.environ, {'ROS_LOG_DIR': cls.log_dir.name})
        cls.environment.start()

    @classmethod
    def tearDownClass(cls):
        cls.environment.stop()
        cls.log_dir.cleanup()

    def test_launch_arguments_reject_retired_sensor_choices(self) -> None:
        root = Path(__file__).resolve().parents[2]
        cases = [
            ('cat_bringup/launch/bringup.launch.py', 'lidar_source', 'vlp16'),
            ('cat_bringup/launch/bringup.launch.py', 'lidar_source', 'both'),
            ('cat_bringup/launch/bringup.launch.py', 'rplidar_mount', 'back'),
            ('cat_bringup/launch/rplidar_s1.launch.py', 'frame_id', 'RPLIDAR_B'),
            ('cat_navigation/launch/mapping.launch.py', 'lidar_source', 'both'),
            ('cat_navigation/launch/mapping.launch.py', 'rplidar_mount', 'back'),
            ('cat_navigation/launch/navigation.launch.py', 'lidar_source', 'both'),
            ('cat_navigation/launch/navigation.launch.py', 'rplidar_mount', 'back'),
        ]
        for path, name, value in cases:
            with self.subTest(path=path, argument=name, value=value):
                launch = runpy.run_path(str(root / path))['generate_launch_description']()
                declaration = next(entity for entity in launch.entities
                                   if isinstance(entity, DeclareLaunchArgument) and entity.name == name)
                context = LaunchContext()
                context.launch_configurations[name] = value
                with self.assertRaises(RuntimeError):
                    declaration.execute(context)


    def test_direct_driver_and_mount_overrides(self):
        path = Path(__file__).resolve().parents[1] / 'launch/rplidar_s1.launch.py'
        for frame, roll, yaw, rear, blind in [
                ('RPLIDAR_F', '0', str(math.pi), .6, 90.),
                ('RPLIDAR_custom', str(math.pi), str(math.pi/2), .2, 0.)]:
            launch = runpy.run_path(str(path))['generate_launch_description']()
            context = LaunchContext()
            context.launch_configurations.update(frame_id=frame, mounting_roll=roll,
                mounting_yaw=yaw, front_blind_rear_angle_deg='90.0')
            node = launch.entities[-1]
            self.assertEqual(node.node_package, 'sllidar_ros2')
            self.assertEqual(node.node_executable, 'sllidar_node')
            params = {''.join(context.perform_substitution(part) for part in name): value
                      for name, value in node._Node__parameters[-1].items()}
            self.assertEqual(params['mounting_roll'].evaluate(context), float(roll))
            self.assertEqual(params['mounting_yaw'].evaluate(context), float(yaw))
            self.assertEqual(params['rear_min_range'].evaluate(context), rear)
            self.assertEqual(params['blind_rear_angle_deg'].evaluate(context), blind)
            self.assertNotIn('scan_frame', params)
            self.assertNotIn('scan_raw', path.read_text())
            self.assertNotIn('sector_scan_filter', path.read_text())

    def test_config_and_old_executable_retired(self):
        root = Path(__file__).resolve().parents[1]
        params = yaml.safe_load((root / 'config/scan_filter.yaml').read_text())['sllidar_node']['ros__parameters']
        self.assertTrue(params['sector_filter_enabled'])
        self.assertEqual(params['mounting_yaw'], math.pi)
        self.assertEqual(params['front_min_range'], .2)
        self.assertEqual(params['rear_min_range'], .6)
        self.assertEqual(params['blind_rear_angle_deg'], 90.)
        self.assertFalse((root / 'scripts/sector_scan_filter').exists())
        self.assertNotIn('scripts/sector_scan_filter', (root / 'CMakeLists.txt').read_text())
        bringup = (root / 'launch/bringup.launch.py').read_text()
        self.assertIn("'filter_config': LaunchConfiguration('rplidar_filter_config')", bringup)
