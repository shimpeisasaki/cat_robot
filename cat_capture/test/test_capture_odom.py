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
