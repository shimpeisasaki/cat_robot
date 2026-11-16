"""Synthetic raw sensor topics only: no hardware driver or motion publisher."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
import os
import time

import rclpy
from sensor_msgs.msg import Imu, PointCloud2

loader = SourceFileLoader('horizon_time_ros_impl', str(
    Path(__file__).parents[1] / 'scripts/horizon_time_bridge'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)


def test_raw_payload_mapped_stamp_and_reset_latch():
    assert os.environ['ROS_DOMAIN_ID'] == '211'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init(args=['--ros-args', '-p', 'warmup_seconds:=0.1'])
    bridge = impl.HorizonTimeBridge()
    peer = rclpy.create_node('horizon_synthetic_source')
    cloud_pub = peer.create_publisher(PointCloud2, '/livox/lidar_sensor_raw', 1)
    imu_pub = peer.create_publisher(Imu, '/livox/imu_sensor_raw', 10)
    clouds, imus = [], []
    subscriptions = [
        peer.create_subscription(PointCloud2, '/livox/lidar', clouds.append, 10),
        peer.create_subscription(Imu, '/livox/imu_raw', imus.append, 10)]
    start = time.monotonic()
    reset = False

    def spin_for(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            stamp = int(((.01 if reset else 1000.) + time.monotonic()-start) * 1e9)
            cloud = PointCloud2()
            cloud.header.frame_id = 'livox_frame'
            cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(stamp, 10**9)
            cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 1, 12, 12
            cloud.data = bytes(range(12))
            imu = Imu()
            imu.header = cloud.header
            imu.angular_velocity.z = .123
            cloud_pub.publish(cloud)
            imu_pub.publish(imu)
            rclpy.spin_once(bridge, timeout_sec=.005)
            rclpy.spin_once(bridge, timeout_sec=.005)
            rclpy.spin_once(peer, timeout_sec=.005)
            assert cloud.header.stamp.sec < 2000  # Raw input never mutated.
            time.sleep(.01)

    try:
        spin_for(2.)
        assert clouds and imus
        assert bytes(clouds[-1].data) == bytes(range(12))
        assert clouds[-1].header.frame_id == 'livox_frame'
        mapped = clouds[-1].header.stamp
        age = peer.get_clock().now().nanoseconds/1e9 - mapped.sec - mapped.nanosec/1e9
        assert -.05 <= age < .5
        assert imus[-1].angular_velocity.z == .123
        reset = True
        spin_for(.3)
        assert 'sensor_clock_reset' in bridge.mapper.fault
        counts = dict(bridge.counts)
        spin_for(.3)
        assert bridge.counts == counts
    finally:
        for subscription in subscriptions:
            peer.destroy_subscription(subscription)
        peer.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()
