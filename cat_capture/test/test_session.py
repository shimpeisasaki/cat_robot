"""Verify the migrated capture saves frames without calling the old stitcher."""
import numpy as np

from cat_capture.session import CaptureSession


def test_capture_saves_only_rgb_frames(tmp_path):
    session = CaptureSession(tmp_path)
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    assert session.capture(image, -30) == 1
    assert session.finish() == str(session.path)
    assert [path.name for path in session.path.iterdir()] == ['f00_330.png']
