import cv2
import numpy as np

from cat_capture.session import CaptureSession


def test_saves_ordered_rgb_depth_pairs(tmp_path):
    session = CaptureSession(tmp_path, 120.0, 10)
    rgb = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2.5, dtype=np.float32)
    assert session.capture(rgb, depth) == 1
    assert session.path.name.endswith('_120_10')
    assert cv2.imread(str(session.path / 'f00_rgb.png')).shape == rgb.shape
    np.testing.assert_array_equal(np.load(session.path / 'f00_depth.npy'), depth)
