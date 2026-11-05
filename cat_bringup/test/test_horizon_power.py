"""Opt-in real Horizon wake/idle regression; never start hardware by default."""

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get('CAT_TEST_HORIZON') != '1',
    reason='Real hardware: explicitly set CAT_TEST_HORIZON=1 to enable')


def test_horizon_wake_receive_and_idle_twice(tmp_path):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node('horizon_power_test')
    received = []
    subscription = node.create_subscription(
        PointCloud2, '/livox/lidar',
        lambda cloud: received.append(cloud.width * cloud.height),
        qos_profile_sensor_data)
    try:
        # Discover existing publishers before acquiring the device.
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert not node.get_publishers_info_by_topic('/livox/lidar'), (
            'Stop any existing Livox driver before running this test')
        for cycle in range(2):
            received.clear()
            log_path = Path(tmp_path) / f'horizon-cycle-{cycle}.log'
            with log_path.open('w') as output:
                process = subprocess.Popen(
                    ['ros2', 'launch', 'cat_bringup', 'horizon.launch.py'],
                    stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    deadline = time.monotonic() + 20
                    while time.monotonic() < deadline and len(received) < 25:
                        if process.poll() is not None:
                            break
                        rclpy.spin_once(node, timeout_sec=0.2)
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                    try:
                        result = process.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                        pytest.fail(f'Shutdown timed out; sensor stop unconfirmed: {log_path}')
            log = log_path.read_text()
            assert len(received) >= 25, log
            assert max(received) > 0, log
            assert result == 0, log
            assert 'Horizon Standby confirmed:' in log, log
            assert 'NOT confirmed' not in log, log
            assert 'process has finished cleanly' in log, log
            if cycle == 1:
                assert 'Horizon requesting Normal:' in log, log
            time.sleep(3)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
