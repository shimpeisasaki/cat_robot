from collections import deque
import math
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

from cat_capture.capture_node import PanoramaCapture


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
    from unittest.mock import Mock
    from sensor_msgs.msg import CompressedImage
    from cat_capture.session import CaptureSession
    ok, encoded = cv2.imencode('.jpg', np.zeros((4, 6, 3), dtype=np.uint8))
    assert ok
    msg = CompressedImage()
    msg.format = 'bgr8; jpeg compressed bgr8'
    msg.data = encoded.tobytes()
    node = SimpleNamespace(session=CaptureSession(tmp_path, 360., 24), bridge=Mock())
    assert PanoramaCapture.save_frame(node, msg, 0., {'stamp': 2.}) == 1
    node.bridge.imgmsg_to_cv2.assert_not_called()
    assert node.session.frames[0].read_bytes() == encoded.tobytes()
    assert node.session.metadata[0]['image_format'] == msg.format
