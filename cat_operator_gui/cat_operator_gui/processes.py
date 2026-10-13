"""Own process groups, logs and bounded graceful shutdown without blocking the GUI."""

from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import psutil


@dataclass
class RunningJob:
    process: object
    log_path: Path
    output: object
    offset: int = 0
    stopping_at: float = 0.0
    stage: int = 0
    launch_parent: bool = False


class ProcessManager:
    def __init__(self, log_dir, log_callback=lambda *_: None, event_callback=lambda *_: None):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_callback = log_callback
        self.event_callback = event_callback
        self.jobs = {}
        self.serial = 0

    def start(self, plan):
        if any(job.name in self.jobs for job in plan.jobs):
            raise ValueError('同じ処理が既に起動しています')
        self.serial += 1
        for index, (label, value) in enumerate(list(plan.sources.items())):
            source = Path(value)
            if source.is_file() and source.suffix in ('.yaml', '.yml'):
                copies = self.log_dir / f'plan_{self.serial:03d}_inputs'
                copies.mkdir(exist_ok=True)
                copy = copies / f'{index:02d}_{source.name}'
                copy.write_bytes(source.read_bytes())
                plan.sources[f'起動時コピー ({label})'] = str(copy)
        (self.log_dir / f'plan_{self.serial:03d}.json').write_text(
            json.dumps(asdict(plan), ensure_ascii=False, indent=2), encoding='utf-8')
        started = []
        try:
            for spec in plan.jobs:
                path = self.log_dir / f'{self.serial:03d}_{spec.name}.log'
                output = path.open('wb')
                environment = os.environ.copy()
                environment.update(spec.env)
                environment['ROS_LOG_DIR'] = str(self.log_dir / 'ros')
                environment['PYTHONNOUSERSITE'] = '1'
                try:
                    process = subprocess.Popen(spec.command, env=environment,
                                               stdin=subprocess.DEVNULL, stdout=output,
                                               stderr=subprocess.STDOUT, start_new_session=True)
                except BaseException:
                    output.close()
                    raise
                self.jobs[spec.name] = RunningJob(
                    process, path, output, launch_parent=spec.command[:2] == ['ros2', 'launch'])
                started.append(spec.name)
                self.event_callback(spec.name, 'started', process.pid)
        except BaseException:
            for name in started:
                self.stop(name)
            raise

    @staticmethod
    def group_alive(pgid):
        for process in psutil.process_iter(['pid', 'status']):
            try:
                if (process.info['status'] != psutil.STATUS_ZOMBIE
                        and os.getpgid(process.pid) == pgid):
                    return True
            except (ProcessLookupError, PermissionError, psutil.NoSuchProcess):
                continue
        return False

    @staticmethod
    def send(job, sig):
        try:
            os.killpg(job.process.pid, sig)
        except ProcessLookupError:
            pass

    def stop(self, name):
        job = self.jobs.get(name)
        if job is None or job.stopping_at:
            return
        job.stopping_at = time.monotonic()
        if job.launch_parent and job.process.poll() is None:
            try:
                os.kill(job.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        else:
            self.send(job, signal.SIGINT)
        self.event_callback(name, 'stopping', None)

    def stop_all(self):
        for name in list(self.jobs):
            self.stop(name)

    def tick(self):
        for name, job in list(self.jobs.items()):
            with job.log_path.open('rb') as stream:
                stream.seek(job.offset)
                data = stream.read(32 * 1024)
                job.offset = stream.tell()
            if data:
                self.log_callback(name, data.decode('utf-8', errors='replace'))
            code = job.process.poll()
            if code is not None and not job.stopping_at:
                if self.group_alive(job.process.pid):
                    self.stop(name)
                else:
                    self.finish(name, code)
                    continue
            if job.stopping_at:
                if code is not None and not self.group_alive(job.process.pid):
                    self.finish(name, code)
                    continue
                elapsed = time.monotonic() - job.stopping_at
                if elapsed > 20 and job.stage == 0:
                    self.send(job, signal.SIGTERM)
                    job.stage = 1
                    self.event_callback(name, 'terminate', None)
                elif elapsed > 25 and job.stage == 1:
                    self.send(job, signal.SIGKILL)
                    job.stage = 2
                    self.event_callback(name, 'kill', None)

    def finish(self, name, code):
        job = self.jobs.pop(name)
        job.output.close()
        self.event_callback(name, 'finished', code)
