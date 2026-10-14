"""The online image matches the successfully saved frame and its index."""

from collections import deque
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from sensor_msgs.msg import Image

from cat_capture.capture_node import PanoramaCapture


def test_successful_capture_publishes_indexed_ros_image():
    image = Image()
    future = Future()
    future.set_result((1, image))
    node = SimpleNamespace(
        jobs=deque([('capture', future)]), cancel_reason=None,
        state='recording', frames=0, error='', pending_capture=None,
        session=SimpleNamespace(path=Path('/tmp/capture'), total_frames=24,
                                capture_angle_deg=360.0),
        captured_frame=Mock(), status=Mock(), ready=lambda: True,
    )
    PanoramaCapture.tick(node)
    message = node.captured_frame.publish.call_args.args[0]
    assert message.session == '/tmp/capture'
    assert message.index == 0
    assert message.total_frames == 24
    assert message.capture_angle_deg == 360.0
    assert message.image is image
