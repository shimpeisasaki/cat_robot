"""Save ordered RGB/point-cloud pairs without generating a stitched panorama."""
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import cv2


class CaptureSession:
    def __init__(self, output_dir, angle_deg, frames):
        root = Path(output_dir).expanduser().resolve()
        angle = f'{angle_deg:g}'
        name = datetime.now(ZoneInfo('Asia/Tokyo')).strftime('%m%d_%H:%M')
        name = f'{name}_{angle}_{frames}'
        root.mkdir(parents=True, exist_ok=True)
        for suffix in range(100):
            self.path = root / (name if suffix == 0 else f'{name}_{suffix:02d}')
            try:
                self.path.mkdir(exist_ok=False)
                break
            except FileExistsError:
                continue
        else:
            raise FileExistsError(f'Too many capture sessions at {name}')
        self.frames = []

    def capture(self, bgr, cloud):
        stem = f'f{len(self.frames):02d}'
        rgb_path = self.path / f'{stem}_rgb.png'
        cloud_path = self.path / f'{stem}_cloud.pcd'
        try:
            self.write_cloud(cloud_path, cloud)
            if not cv2.imwrite(str(rgb_path), bgr):
                raise OSError(f'Could not save {rgb_path}')
        except Exception:
            cloud_path.unlink(missing_ok=True)
            rgb_path.unlink(missing_ok=True)
            raise
        self.frames.append((rgb_path, cloud_path))
        return len(self.frames)

    @staticmethod
    def write_cloud(path, cloud):
        # ZED publishes organized XYZRGB with a packed float32 rgb field.
        expected = [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]
        fields = [(field.name, field.offset) for field in cloud.fields]
        if (fields != expected or any(field.datatype != 7 or field.count != 1
                                      for field in cloud.fields) or
                cloud.is_bigendian or cloud.point_step != 16 or
                cloud.row_step != cloud.width * 16 or
                len(cloud.data) != cloud.row_step * cloud.height or
                not cloud.header.frame_id):
            raise ValueError('Expected little-endian organized ZED XYZRGB point cloud')
        stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        header = (
            '# .PCD v0.7 - Point Cloud Data file format\n'
            f'# ROS_FRAME_ID {cloud.header.frame_id}\n'
            f'# ROS_STAMP_NSEC {stamp}\n'
            'VERSION 0.7\n'
            'FIELDS x y z rgb\n'
            'SIZE 4 4 4 4\n'
            'TYPE F F F F\n'
            'COUNT 1 1 1 1\n'
            f'WIDTH {cloud.width}\n'
            f'HEIGHT {cloud.height}\n'
            'VIEWPOINT 0 0 0 1 0 0 0\n'
            f'POINTS {cloud.width * cloud.height}\n'
            'DATA binary\n'
        )
        with path.open('wb') as output:
            output.write(header.encode('ascii'))
            output.write(cloud.data)

    def finish(self):
        return str(self.path)
