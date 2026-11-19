import cv2

import numpy as np

from cat_capture.session import CaptureSession


def test_saves_ordered_rgb_images(tmp_path):
    session = CaptureSession(tmp_path, 120.0, 10)
    rgb = np.zeros((3, 4, 3), dtype=np.uint8)
    ok, jpeg = cv2.imencode('.jpg', rgb)
    assert ok
    assert session.capture_jpeg(jpeg.tobytes(), 0.0) == 1
    assert session.path.name.endswith('_120_10')
    assert cv2.imread(str(session.path / 'f00_000.0deg_rgb.jpg')).shape == rgb.shape
    assert (session.path / 'f00_000.0deg_rgb.jpg').read_bytes().startswith(b'\xff\xd8')
    assert not list(session.path.glob('*.pcd'))
    assert not list(session.path.glob('*.png'))


def test_source_jpeg_is_saved_without_transcoding(tmp_path, monkeypatch):
    import json
    session = CaptureSession(tmp_path, 360., 24)
    ok, encoded = cv2.imencode('.jpg', np.zeros((4, 6, 3), dtype=np.uint8))
    assert ok
    blob = encoded.tobytes()
    comment = b'source-camera-metadata'
    blob = blob[:2] + b'\xff\xfe' + (len(comment) + 2).to_bytes(2, 'big') + comment + blob[2:]
    def forbidden(*args, **kwargs):
        raise AssertionError('Source JPEG must not be transcoded')
    monkeypatch.setattr(cv2, 'imwrite', forbidden)
    monkeypatch.setattr(cv2, 'imdecode', forbidden)
    monkeypatch.setattr(cv2, 'imencode', forbidden)
    assert session.capture_jpeg(blob, 15., {'stamp': 123., 'frame': 'camera'}) == 1
    session.finish()
    assert session.frames[0].read_bytes() == blob
    row = json.loads((session.path/'frames.json').read_text())[0]
    assert row['image_storage'] == 'source_jpeg' and row['stamp'] == 123.


def test_bad_source_jpeg_does_not_create_a_frame(tmp_path):
    import pytest
    session = CaptureSession(tmp_path, 360., 24)
    with pytest.raises(ValueError, match='JPEG'):
        session.capture_jpeg(b'bad', 0.)
    assert session.frames == [] and not list(session.path.glob('*.jpg'))
