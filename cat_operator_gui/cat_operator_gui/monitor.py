"""PC measurements outside the UI thread; NVIDIA monitoring is optional."""

import shutil
import subprocess
from threading import Event, Thread

from PyQt5.QtCore import QObject, pyqtSignal
import psutil


class PcMonitor(QObject):
    updated = pyqtSignal(dict)

    def __init__(self, directory):
        super().__init__()
        self.directory = directory
        self.exit_event = Event()
        self.thread = Thread(target=self.run, name='cat-gui-pc', daemon=False)

    def start(self):
        self.thread.start()

    def run(self):
        while not self.exit_event.is_set():
            result = {'cpu': psutil.cpu_percent(), 'memory': psutil.virtual_memory().percent}
            try:
                result['disk_free_gb'] = round(shutil.disk_usage(self.directory).free / 1024**3, 1)
            except OSError:
                result['disk_free_gb'] = None
            try:
                process = subprocess.run([
                    'nvidia-smi',
                    '--query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu',
                    '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=2,
                    check=False)
                if process.returncode:
                    raise OSError(process.stderr.strip() or 'NVIDIA GPUを取得できません')
                result['gpu'] = process.stdout.strip()
            except (OSError, subprocess.TimeoutExpired) as exc:
                result['gpu'] = f'取得不可: {exc}'
            self.updated.emit(result)
            self.exit_event.wait(1.0)

    def close(self):
        self.exit_event.set()
        self.thread.join(timeout=3.0)
