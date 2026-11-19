"""Exercise shutdown against a real unresponsive grandchild without hardware."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
import signal
import subprocess
import sys
import time
from contextlib import nullcontext

import pytest
import yaml

loader = SourceFileLoader('zed_supervisor_impl', str(
    Path(__file__).parents[1] / 'scripts/zed_supervisor'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)


@pytest.mark.parametrize('profile', ['zed_vio.yaml', 'zed_sensors.yaml'])
def test_profile_effective_values_and_types_match_before_refactor(profile):
    root = Path(__file__).parents[1]
    expected = yaml.safe_load((root / 'test/fixtures' / profile).read_text())
    resolve = getattr(impl, 'resolved_config', nullcontext)
    with resolve(str(root / 'config' / profile)) as effective:
        actual = yaml.safe_load(Path(effective).read_text())
    # YAML serialization distinguishes float ROS parameters from integer ones.
    assert yaml.safe_dump(actual) == yaml.safe_dump(expected)


def test_standalone_custom_config_is_not_overlaid_or_rewritten(tmp_path):
    custom = tmp_path / 'custom.yaml'
    original = '/**:\n  ros__parameters:\n    pos_tracking:\n      pos_tracking_enabled: false\n'
    custom.write_text(original)
    resolve = getattr(impl, 'resolved_config', nullcontext)
    with resolve(str(custom)) as effective:
        assert effective == str(custom)
        assert Path(effective).read_text() == original
    assert custom.read_text() == original


def test_common_profile_overrides_from_its_directory_and_cleans_up(tmp_path, monkeypatch):
    common = tmp_path / 'common.yaml'
    common.write_text('/**:\n  ros__parameters:\n    video:\n      jpeg_quality: 90\n'
                      '      publish_raw: true\n    pos_tracking:\n      pos_tracking_enabled: false\n')
    profile = tmp_path / 'profile.yaml'
    profile.write_text('cat_zed_common: common.yaml\n/**:\n  ros__parameters:\n'
                       '    video:\n      jpeg_quality: 81\n'
                       '    pos_tracking:\n      pos_tracking_enabled: true\n')
    monkeypatch.chdir(tmp_path.parent)
    with impl.resolved_config(str(profile)) as effective:
        path = Path(effective)
        params = yaml.safe_load(path.read_text())['/**']['ros__parameters']
        assert params['video'] == {'jpeg_quality': 81, 'publish_raw': True}
        assert params['pos_tracking']['pos_tracking_enabled'] is True
        assert 'cat_zed_common' not in yaml.safe_load(path.read_text())
        assert path != profile
    assert not path.exists()
    assert profile.is_file() and common.is_file()


@pytest.mark.parametrize('exit_code', [0, 2])
def test_main_keeps_exact_override_alive_until_supervision_returns(monkeypatch, exit_code):
    profile = Path(__file__).parents[1] / 'config/zed_vio.yaml'
    paths = []
    def supervise(serial, path, ros_args):
        paths.append(Path(path))
        assert serial == 'synthetic_serial' and ros_args == []
        config = yaml.safe_load(Path(path).read_text())
        assert config['/**']['ros__parameters']['pos_tracking']['pos_tracking_enabled'] is True
        assert config['/**']['ros__parameters']['video']['jpeg_quality'] == 90
        return exit_code
    monkeypatch.setattr(impl, 'supervise', supervise)
    monkeypatch.setattr(sys, 'argv', ['zed_supervisor', '--serial-number', 'synthetic_serial',
                                     '--config', str(profile)])
    assert impl.main() == exit_code
    assert len(paths) == 1 and not paths[0].exists()


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
