"""Save ordered RGB frames without generating a stitched panorama."""
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo

class CaptureSession:
    def __init__(self, output_dir, angle_deg, frames):
        self.capture_angle_deg = angle_deg
        self.total_frames = frames
        root = Path(output_dir).expanduser().resolve()
        angle = f'{angle_deg:g}'
        name = datetime.now(ZoneInfo('Asia/Tokyo')).strftime('%Y%m%d_%H%M%S')
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
        self.metadata = []

    def capture_jpeg(self, jpeg, angle_deg, metadata=None):
        """Keep the camera's compressed bytes, without another lossy encode."""
        blob = bytes(jpeg)
        if not blob.startswith(b'\xff\xd8') or not blob.endswith(b'\xff\xd9'):
            raise ValueError('Expected JPEG camera bytes')
        stem = f'f{len(self.frames):02d}'
        rounded_angle = round(angle_deg, 1) % 360
        rgb_path = self.path / f'{stem}_{rounded_angle:05.1f}deg_rgb.jpg'
        try:
            rgb_path.write_bytes(blob)
        except Exception:
            rgb_path.unlink(missing_ok=True)
            raise
        self.frames.append(rgb_path)
        self.metadata.append(dict(file=rgb_path.name, **(metadata or {}),
                                  image_storage='source_jpeg'))
        return len(self.frames)

    def finish(self):
        if self.metadata:
            (self.path / 'frames.json').write_text(json.dumps(self.metadata, indent=2))
        return str(self.path)
