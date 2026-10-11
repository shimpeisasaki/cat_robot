import cv2
import struct
from types import SimpleNamespace

import numpy as np

from cat_capture.session import CaptureSession


def test_saves_ordered_rgb_cloud_pairs(tmp_path):
    session = CaptureSession(tmp_path, 120.0, 10)
    rgb = np.zeros((3, 4, 3), dtype=np.uint8)
    points = struct.pack('<8f', 1, 2, 3, 4, 5, 6, 7, 8)
    cloud = SimpleNamespace(
        fields=[SimpleNamespace(name=name, offset=offset, datatype=7, count=1)
                for name, offset in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]],
        is_bigendian=False, point_step=16, row_step=32, width=2, height=1,
        data=points, header=SimpleNamespace(
            frame_id='zed_left_camera_optical_frame',
            stamp=SimpleNamespace(sec=123, nanosec=456)))
    assert session.capture(rgb, cloud) == 1
    assert session.path.name.endswith('_120_10')
    assert cv2.imread(str(session.path / 'f00_rgb.png')).shape == rgb.shape
    saved = (session.path / 'f00_cloud.pcd').read_bytes()
    assert b'FIELDS x y z rgb\n' in saved
    assert b'# ROS_STAMP_NSEC 123000000456\n' in saved
    assert saved.endswith(points)
    assert not list(session.path.glob('*.npy'))
