"""Geometry and unknown-return regression checks; no hardware required."""
import math
from pathlib import Path
import runpy
import unittest
import yaml

filter_ranges = runpy.run_path(str(
    Path(__file__).resolve().parents[1] / 'scripts/sector_scan_filter'))['filter_ranges']


class SectorFilterTest(unittest.TestCase):
    def test_configured_rplidar_mask_is_uniform(self):
        config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                 'config/scan_filter.yaml').read_text())
        params = config['sector_scan_filter']['ros__parameters']
        self.assertEqual(params['front_min_range'], 0.2)
        self.assertEqual(params['rear_min_range'], 0.2)
        for bearing in (-math.pi, -math.pi / 2, 0, math.pi / 2, math.pi):
            self.assertTrue(math.isnan(filter_ranges(
                [0.19], bearing, 0, params['mounting_yaw'],
                math.radians(params['front_angle_deg']), 0.2, 0.2,
                -1.0)[0]))

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
        # RPLIDAR_B: roll 180 deg reverses each scan bearing before yaw +90 deg.
        out = filter_ranges([0.2, 0.2], -math.pi / 2, math.pi,
                            math.pi / 2, math.radians(200), 0.1, 0.4, -1.0)
        self.assertTrue(math.isnan(out[0]))  # sensor -90 deg points vehicle rear
        self.assertEqual(out[1], 0.2)  # sensor +90 deg points vehicle front


if __name__ == '__main__':
    unittest.main()
