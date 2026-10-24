"""VLP-16 masking must preserve cloud metadata and organized point layout."""
import math
from pathlib import Path
import runpy
import struct

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


module = runpy.run_path(str(
    Path(__file__).resolve().parents[1] / 'scripts/vlp16_sector_filter'))
filter_cloud = module['filter_cloud']
mounting_rotation = module['mounting_rotation']


def sample_cloud():
    cloud = PointCloud2()
    cloud.header.frame_id = 'vlp16'
    cloud.height = 1
    cloud.width = 5
    cloud.is_dense = True
    cloud.point_step = 24
    cloud.row_step = 24 * cloud.width
    cloud.fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
                    for name, offset, datatype in (
                        ('x', 0, PointField.FLOAT32),
                        ('y', 4, PointField.FLOAT32),
                        ('z', 8, PointField.FLOAT32),
                        ('intensity', 12, PointField.FLOAT32),
                        ('ring', 16, PointField.UINT16),
                        ('time', 20, PointField.FLOAT32))]
    points = [(0.05, 0., 0.), (0.2, 0., 0.),
              (-0.3, 0., 0.), (-0.7, 0., 0.), (0., 0.15, 0.)]
    cloud.data = b''.join(struct.pack('<ffffHxxf', x, y, z, 5., i, i * 0.01)
                          for i, (x, y, z) in enumerate(points))
    return cloud


def test_front_and_rear_mask_keep_organized_layout():
    cloud = sample_cloud()
    rotation = mounting_rotation(0, math.radians(7), 0)
    output, count = filter_cloud(cloud, rotation, math.radians(270), 0.1, 0.6)
    assert count == 2
    assert (output.height, output.width, output.point_step, output.row_step) == (1, 5, 24, 120)
    assert output.header.frame_id == 'vlp16'
    assert not output.is_dense
    values = [struct.unpack_from('<ffffHxxf', output.data, i * 24) for i in range(5)]
    assert all(math.isnan(values[i][axis]) for i in (0, 2) for axis in range(4))
    assert np.allclose([values[i][0] for i in (1, 3, 4)], [0.2, -0.7, 0.0])
    assert [row[4] for row in values] == list(range(5))
    assert np.allclose([row[5] for row in values], [i * 0.01 for i in range(5)])


def test_invalid_layout_fails_closed():
    cloud = sample_cloud()
    cloud.fields = [field for field in cloud.fields if field.name != 'z']
    rotation = np.eye(3)
    try:
        filter_cloud(cloud, rotation, math.radians(270), 0.1, 0.6)
    except ValueError as exc:
        assert 'FLOAT32 z' in str(exc)
    else:
        raise AssertionError('Missing z field was accepted')
