"""Exercise shutdown against a real unresponsive grandchild without hardware."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

loader = SourceFileLoader('zed_supervisor_impl', str(
    Path(__file__).parents[1] / 'scripts/zed_supervisor'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)


@pytest.mark.parametrize('parent_exits', [False, True])
def test_stop_cleans_unresponsive_descendant_even_after_parent_exit(tmp_path, parent_exits):
    pid_file = tmp_path / 'child.pid'
    child = ('import os, signal, time; from pathlib import Path; '
             'signal.signal(signal.SIGINT, signal.SIG_IGN); '
             'signal.signal(signal.SIGTERM, signal.SIG_IGN); '
             f'Path({str(pid_file)!r}).write_text(str(os.getpid())); time.sleep(30)')
    parent = ('import subprocess, sys, time; '
              f'subprocess.Popen([sys.executable, "-c", {child!r}]); '
              + ('time.sleep(.2)' if parent_exits else 'time.sleep(30)'))
    process = subprocess.Popen([sys.executable, '-c', parent], start_new_session=True)
    try:
        deadline = time.monotonic() + 5
        while not pid_file.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert pid_file.exists()
        child_pid = int(pid_file.read_text())
        if parent_exits:
            process.wait(timeout=5)
        start = time.monotonic()
        impl.stop_group(process)
        assert time.monotonic() - start < 4.5
        assert process.poll() is not None
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            stat = Path(f'/proc/{child_pid}/stat')
            if not stat.exists() or stat.read_text().split()[2] == 'Z':
                break
            time.sleep(.02)
        else:
            pytest.fail('Unresponsive descendant survived group cleanup')
    finally:
        try:
            import os
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)
