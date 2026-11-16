"""Synthetic ZED poses and mounting TF only; no camera or command publisher."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
import os
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

loader = SourceFileLoader('spatial_odom_ros_impl', str(
    Path(__file__).parents[1] / 'scripts/odom_source'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)


@pytest.mark.parametrize('initial_pitch_deg', [-5., 0., 5.])
def test_spatial_vio_publishes_height_pitch_and_latches_after_tracking_gap(initial_pitch_deg):
    assert os.environ['ROS_DOMAIN_ID'] == '212'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init(args=['--ros-args', '-p', 'source:=vio', '-p', 'pose_mode:=spatial'])
    odometry = impl.OdomSource()
    peer = rclpy.create_node('synthetic_spatial_zed')
    publisher = peer.create_publisher(PoseWithCovarianceStamped, '/zed/zed_node/pose_with_covariance', 10)
    outputs = []
    subscription = peer.create_subscription(Odometry, '/odom', outputs.append, 10)
    buffer = Buffer()
    listener = TransformListener(buffer, peer)
    broadcaster = StaticTransformBroadcaster(peer)
    mount = np.eye(4)
    mount[:3, 3] = [-.285, 0., .550]
    mount[:3, :3] = Rotation.from_euler('y', np.deg2rad(12.)).as_matrix()
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'zed_camera_link'
    transform.header.stamp = peer.get_clock().now().to_msg()
    for axis, value in zip(('x', 'y', 'z'), mount[:3, 3]):
        setattr(transform.transform.translation, axis, float(value))
    for axis, value in zip(('x', 'y', 'z', 'w'), Rotation.from_matrix(mount[:3, :3]).as_quat()):
        setattr(transform.transform.rotation, axis, float(value))
    broadcaster.sendTransform(transform)

    def send(progress):
        base = np.eye(4)
        base[:3, 3] = [2.+.2*progress, 3., .3+.03*progress]
        base[:3, :3] = Rotation.from_euler('y', np.deg2rad(initial_pitch_deg-5.*progress)).as_matrix()
        camera = base @ mount
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'zed_map'
        message.header.stamp = peer.get_clock().now().to_msg()
        for axis, value in zip(('x', 'y', 'z'), camera[:3, 3]):
            setattr(message.pose.pose.position, axis, float(value))
        for axis, value in zip(('x', 'y', 'z', 'w'), Rotation.from_matrix(camera[:3, :3]).as_quat()):
            setattr(message.pose.pose.orientation, axis, float(value))
        publisher.publish(message)

    def poll():
        # Drain odometry and TF callbacks; one callback per published sample
        # can otherwise starve the output subscription behind TF delivery.
        for _ in range(4):
            rclpy.spin_once(odometry, timeout_sec=.001)
            rclpy.spin_once(peer, timeout_sec=.001)
        time.sleep(.01)

    try:
        until = time.monotonic()+4.
        while not outputs and time.monotonic() < until:
            send(0.)
            poll()
        assert outputs, (odometry.source, odometry.pose_mode, odometry.last_stamp,
                         odometry.count_subscribers('/zed/zed_node/pose_with_covariance'),
                         odometry.buffer.all_frames_as_yaml())
        start = time.monotonic()
        while time.monotonic()-start < 1.5:
            send(min(1., time.monotonic()-start))
            poll()
        assert not odometry.failed
        out = outputs[-1]
        p, q = out.pose.pose.position, out.pose.pose.orientation
        assert np.allclose([p.x, p.y, p.z], [.2, 0., .03], atol=.001)
        assert np.allclose(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz'),
                           [0., np.deg2rad(initial_pitch_deg-5.), 0.], atol=.001)
        tf = buffer.lookup_transform('odom', 'base_link', Time()).transform
        assert np.allclose([tf.translation.x, tf.translation.y, tf.translation.z],
                           [.2, 0., .03], atol=.001)
        assert np.allclose([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w],
                           [q.x, q.y, q.z, q.w], atol=.001)
        until = time.monotonic()+1.2
        while time.monotonic() < until:
            poll()
        count = odometry.last_stamp
        for _ in range(10):
            send(1.)
            poll()
        assert odometry.failed and odometry.last_stamp == count
    finally:
        peer.destroy_subscription(subscription)
        # Retain TF listener lifetime until after the polling checks.
        del listener
        peer.destroy_node()
        odometry.destroy_node()
        rclpy.shutdown()
