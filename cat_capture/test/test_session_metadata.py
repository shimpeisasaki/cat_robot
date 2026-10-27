import json
import numpy as np

from cat_capture.session import CaptureSession


def test_capture_session_writes_source_metadata(tmp_path):
    session = CaptureSession(tmp_path, 360, 24)
    session.capture(np.zeros((4, 4, 3), dtype=np.uint8), 15.,
                    dict(stamp=123.5, frame='camera', odom_pose=[1., 2., .3]))
    session.finish()
    rows = json.loads((session.path/'frames.json').read_text())
    assert len(rows) == 1
    assert rows[0]['stamp'] == 123.5
    assert rows[0]['odom_pose'] == [1., 2., .3]
    assert (session.path/rows[0]['file']).is_file()
