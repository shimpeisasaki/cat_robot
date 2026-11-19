from collections import deque
import math
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

from cat_capture.capture_node import PanoramaCapture


@pytest.mark.parametrize('topic', [
    '/zed/zed_node/rgb/color/raw/image/compressed', '/synthetic/explicit_jpeg',
])
def test_capture_subscribes_reliable_jpeg_without_transport_selector(topic):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import ReliabilityPolicy
    rclpy.init()
    node = PanoramaCapture(parameter_overrides=[Parameter('rgb_topic', value=topic)])
    try:
        subscriptions = node.get_subscriptions_info_by_topic(topic)
        assert len(subscriptions) == 1
        assert subscriptions[0].topic_type == 'sensor_msgs/msg/CompressedImage'
        assert subscriptions[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert not node.has_parameter('rgb_transport')
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


def test_capture_interpolates_successive_odom_positions():
    node = SimpleNamespace(odom_frames=deque(maxlen=150))
    first = Odometry()
    first.header.stamp.sec = 1
    first.pose.pose.orientation.w = 1.
    first.pose.pose.position.x = 1.
    second = Odometry()
    second.header.stamp.sec = 1
    second.header.stamp.nanosec = 100_000_000
    second.pose.pose.orientation.z = math.sin(.2)
    second.pose.pose.orientation.w = math.cos(.2)
    second.pose.pose.position.x = 3.
    PanoramaCapture.receive_odom(node, first)
    PanoramaCapture.receive_odom(node, second)
    assert len(node.odom_frames) == 2
    assert node.odom_frames[1][3:] == (3., 0.)
    node.image_pose = lambda stamp: PanoramaCapture.image_pose(node, stamp)
    assert node.image_pose(1_050_000_000) == pytest.approx((.2, 2., 0.))


def test_capture_node_keeps_compressed_frame_and_metadata(tmp_path):
    import cv2
    import numpy as np
    from sensor_msgs.msg import CompressedImage
    from cat_capture.session import CaptureSession
    ok, encoded = cv2.imencode('.jpg', np.zeros((4, 6, 3), dtype=np.uint8))
    assert ok
    msg = CompressedImage()
    msg.format = 'bgr8; jpeg compressed bgr8'
    msg.data = encoded.tobytes()
    node = SimpleNamespace(session=CaptureSession(tmp_path, 360., 24))
    assert PanoramaCapture.save_frame(node, msg, 0., {'stamp': 2.}) == 1
    assert node.session.frames[0].read_bytes() == encoded.tobytes()
    assert node.session.metadata[0]['image_format'] == msg.format


@pytest.mark.parametrize('format,data', [
    ('png', b'\xff\xd8fixture\xff\xd9'), ('jpeg', b'bad'),
    ('jpeg', b'\xff\xd8missing_end'), ('', b'\xff\xd8fixture\xff\xd9'),
])
def test_capture_rejects_malformed_jpeg_before_it_becomes_ready(format, data):
    from sensor_msgs.msg import CompressedImage
    msg = CompressedImage()
    msg.header.stamp.sec = 1
    msg.format, msg.data = format, data
    node = SimpleNamespace(rgb_frames={}, rgb_at=0.)
    node.remember = lambda frames, frame: PanoramaCapture.remember(frames, frame)
    PanoramaCapture.receive_image(node, msg)
    assert node.rgb_frames == {} and node.rgb_at == 0.


def test_cancel_waits_for_running_write_and_blocks_new_session():
    from concurrent.futures import Future
    from unittest.mock import Mock
    pending = Future()
    pending.set_running_or_notify_cancel()
    node = SimpleNamespace(
        state='recording', session=SimpleNamespace(path='/synthetic', total_frames=24,
                                                  capture_angle_deg=360.),
        jobs=deque([('capture', pending)]), pending_capture=(1, 2, None),
        cancel_reason=None, error='', frames=0, ready=lambda: True,
        status=Mock(), get_logger=Mock())
    PanoramaCapture.cancel(node, None, SimpleNamespace())
    response = PanoramaCapture.start(node, SimpleNamespace(), SimpleNamespace(success=False))
    assert not response.success and node.state == 'cancelling'
    PanoramaCapture.tick(node)
    assert node.state == 'cancelling' and len(node.jobs) == 1
    pending.set_result(1)
    PanoramaCapture.tick(node)
    assert node.state == 'cancelled' and not node.jobs and node.pending_capture is None
