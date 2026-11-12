"""Geometry and unknown-return regression checks; no hardware required."""
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

filter_ranges = runpy.run_path(str(
    Path(__file__).resolve().parents[1] / 'scripts/sector_scan_filter'))['filter_ranges']


class SectorFilterTest(unittest.TestCase):
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
        with tempfile.TemporaryDirectory() as log_dir, patch.dict(
                os.environ, {'ROS_LOG_DIR': log_dir}):
            for path, name, value in cases:
                with self.subTest(path=path, argument=name, value=value):
                    launch = runpy.run_path(str(root / path))['generate_launch_description']()
                    declaration = next(entity for entity in launch.entities
                                       if isinstance(entity, DeclareLaunchArgument) and entity.name == name)
                    context = LaunchContext()
                    context.launch_configurations[name] = value
                    with self.assertRaises(RuntimeError):
                        declaration.execute(context)

    def test_blind_rear_sector_is_unknown_even_for_infinite_or_far_returns(self):
        for distance in (.3, 2., float('inf')):
            self.assertTrue(math.isnan(filter_ranges(
                [distance], 0., 0., math.pi, math.radians(270), .2, .6,
                blind_rear_angle=math.pi/2)[0]))
        self.assertEqual(filter_ranges(
            [2.], math.pi, 0., math.pi, math.radians(270), .2, .6,
            blind_rear_angle=math.pi/2)[0], 2.)

    def test_default_config_matches_front_mount_mask(self):
        config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                 'config/scan_filter.yaml').read_text())
        params = config['sector_scan_filter']['ros__parameters']
        self.assertEqual(params['front_min_range'], 0.2)
        self.assertEqual(params['rear_min_range'], 0.6)
        self.assertEqual(params['scan_frame'], 'RPLIDAR_F')
        self.assertEqual(params['mounting_roll'], 0.0)
        self.assertEqual(params['mounting_yaw'], math.pi)
        for bearing, expected in [(math.pi, .3), (0., math.nan)]:
            result = filter_ranges(
                [.3], bearing, 0, params['mounting_yaw'],
                math.radians(params['front_angle_deg']),
                params['front_min_range'], params['rear_min_range'],
                blind_rear_angle=math.radians(params['blind_rear_angle_deg']))[0]
            if math.isnan(expected):
                self.assertTrue(math.isnan(result))
            else:
                self.assertEqual(result, expected)

    def test_front_mount_uses_rear_90_degree_mask(self):
        for bearing in (-135, 0, 135):
            self.assertEqual(filter_ranges(
                [0.3], math.radians(bearing - 180), 0, math.pi,
                math.radians(270), 0.2, 0.6)[0], 0.3)
        for bearing in (-136, 180, 136):
            self.assertTrue(math.isnan(filter_ranges(
                [0.3], math.radians(bearing - 180), 0, math.pi,
                math.radians(270), 0.2, 0.6)[0]))
            self.assertTrue(math.isnan(filter_ranges(
                [0.599], math.radians(bearing - 180), 0, math.pi,
                math.radians(270), 0.2, 0.6)[0]))
            self.assertEqual(filter_ranges(
                [0.6], math.radians(bearing - 180), 0, math.pi,
                math.radians(270), 0.2, 0.6)[0], 0.6)

    def test_launch_selects_mask_from_scan_frame(self):
        launch_path = (Path(__file__).resolve().parents[1] /
                       'launch/rplidar_s1.launch.py')
        for frame, expected in [('RPLIDAR_F', 0.6), ('RPLIDAR_custom', 0.2)]:
            with tempfile.TemporaryDirectory() as log_dir, patch.dict(
                    os.environ, {'ROS_LOG_DIR': log_dir}):
                launch = runpy.run_path(str(launch_path))['generate_launch_description']()
            filter_node = launch.entities[-1]
            overrides = filter_node._Node__parameters[-1]
            context = LaunchContext()
            context.launch_configurations['frame_id'] = frame
            parameters = {
                ''.join(context.perform_substitution(part) for part in name): value
                for name, value in overrides.items()
            }
            self.assertEqual(parameters['rear_min_range'].evaluate(context), expected)

    def filter(self, vehicle_degrees, distance):
        return filter_ranges([distance], math.radians(vehicle_degrees)-math.pi,
                             0.01, math.pi, math.radians(200), 0.1, 0.4)[0]

    def test_front_and_rear(self):
        for angle in [0, 90, -90, 100, -100, 360]:
            self.assertEqual(self.filter(angle, 0.2), 0.2)
            self.assertTrue(math.isnan(self.filter(angle, 0.09)))
        for angle in [101, -101, 180, -180]:
            self.assertTrue(math.isnan(self.filter(angle, 0.39)))
            self.assertEqual(self.filter(angle, 0.4), 0.4)
        self.assertEqual(self.filter(0, 0.1), 0.1)

    def test_nonfinite_and_far_returns(self):
        self.assertTrue(math.isnan(self.filter(0, float('nan'))))
        self.assertEqual(self.filter(180, float('inf')), float('inf'))
        self.assertEqual(self.filter(180, 2.0), 2.0)

    def test_scan_index_and_mounting_rotation(self):
        out = filter_ranges([0.2]*5, -math.pi, math.pi/2, math.pi,
                            math.radians(200), 0.1, 0.4)
        self.assertEqual(len(out), 5)
        self.assertTrue(math.isnan(out[2]))  # sensor 0 degrees = vehicle rear
        for i in [0, 1, 3, 4]:
            self.assertEqual(out[i], 0.2)

    def test_upside_down_right_facing_mount(self):
        # An inverted custom mount reverses each bearing before yaw +90 deg.
        out = filter_ranges([0.2, 0.2], -math.pi / 2, math.pi,
                            math.pi / 2, math.radians(200), 0.1, 0.4, -1.0)
        self.assertTrue(math.isnan(out[0]))  # sensor -90 deg points vehicle rear
        self.assertEqual(out[1], 0.2)  # sensor +90 deg points vehicle front


if __name__ == '__main__':
    unittest.main()
