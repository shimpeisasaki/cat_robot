"""Calibration math must preserve gravity, sensor time and input data."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path

import pytest
from sensor_msgs.msg import Imu


loader = SourceFileLoader('horizon_imu_impl', str(
    Path(__file__).parents[1] / 'scripts/horizon_imu'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)


def test_upside_down_imu_correction_preserves_gravity_and_stamp():
    raw = Imu()
    raw.header.frame_id = 'livox_frame'
    raw.header.stamp.sec = 123
    raw.angular_velocity.x = 0.012
    raw.angular_velocity.y = -0.008
    raw.angular_velocity.z = 0.003
    raw.linear_acceleration.z = -1.0
    out = impl.correct_imu(raw, [0.01, -0.01, 0.002], [1e-5] * 3, [0.01] * 3)
    assert out.angular_velocity.x == pytest.approx(0.002)
    assert out.angular_velocity.y == pytest.approx(0.002)
    assert out.angular_velocity.z == pytest.approx(0.001)
    assert out.linear_acceleration.z == pytest.approx(-9.80665)
    assert out.header == raw.header
    assert raw.linear_acceleration.z == -1.0
    assert raw.angular_velocity.x == 0.012
    assert out.orientation_covariance[0] == -1.0
    assert list(out.angular_velocity_covariance) == [1e-5, 0., 0., 0., 1e-5, 0., 0., 0., 1e-5]
