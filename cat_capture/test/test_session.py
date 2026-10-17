import cv2

import numpy as np

from cat_capture.session import CaptureSession


def test_saves_ordered_rgb_images(tmp_path):
    session = CaptureSession(tmp_path, 120.0, 10)
    rgb = np.zeros((3, 4, 3), dtype=np.uint8)
    assert session.capture(rgb, 0.0) == 1
    assert session.path.name.endswith('_120_10')
    assert cv2.imread(str(session.path / 'f00_000.0deg_rgb.jpg')).shape == rgb.shape
    assert (session.path / 'f00_000.0deg_rgb.jpg').read_bytes().startswith(b'\xff\xd8')
    assert not list(session.path.glob('*.pcd'))
    assert not list(session.path.glob('*.png'))
