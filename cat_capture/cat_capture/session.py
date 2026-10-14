"""Save ordered RGB frames without generating a stitched panorama."""
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import cv2


class CaptureSession:
    def __init__(self, output_dir, angle_deg, frames):
        self.capture_angle_deg = angle_deg
        self.total_frames = frames
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

    def capture(self, bgr):
        stem = f'f{len(self.frames):02d}'
        rgb_path = self.path / f'{stem}_rgb.png'
        if not cv2.imwrite(str(rgb_path), bgr):
            rgb_path.unlink(missing_ok=True)
            raise OSError(f'Could not save {rgb_path}')
        self.frames.append(rgb_path)
        return len(self.frames)

    def finish(self):
        return str(self.path)
