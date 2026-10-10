"""Save ordered RGB frames without generating a stitched panorama."""
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import cv2


class CaptureSession:
    def __init__(self, output_dir):
        root = Path(output_dir).expanduser().resolve()
        name = datetime.now(ZoneInfo('Asia/Tokyo')).strftime('panorama_%m%d_%H%M_%S')
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

    def capture(self, bgr, angle_degrees):
        angle = int(angle_degrees)
        if not -180 <= angle < 180:
            raise ValueError('Frame angle must be in [-180, 180) degrees')
        path = self.path / f'f{len(self.frames):02d}_{angle % 360:03d}.png'
        if not cv2.imwrite(str(path), bgr):
            raise OSError(f'Could not save {path}')
        self.frames.append(path)
        return len(self.frames)

    def finish(self):
        return str(self.path)
