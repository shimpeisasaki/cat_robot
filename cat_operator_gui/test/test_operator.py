"""Validate command generation, TF isolation, process ownership and the Qt console."""

from dataclasses import asdict
from pathlib import Path
import runpy
import sys
import time
from types import SimpleNamespace

import pytest
import yaml

from cat_operator_gui.offline_tf import keep_transform
from cat_operator_gui.plans import (JobSpec, Plan, Settings, defaults, mode_plan,
                                    record_plan, save_map_plan)
from cat_operator_gui.processes import ProcessManager


@pytest.fixture
def configuration(tmp_path):
    root = Path(__file__).resolve().parents[2]
    shares = {name: str(root / name)
              for name in ('cat_bringup', 'cat_navigation', 'cat_operator_gui')}
    settings = defaults(shares)
    settings.data_dir = str(tmp_path)
    return settings, shares


def test_details_use_actual_arguments_and_reject_incompatible_inputs(configuration):
    settings, shares = configuration
    settings.use_zed = True
    settings.odom_source = 'vio'
    plan = mode_plan(settings, shares)
    assert 'odom_source:=vio' in plan.jobs[0].command
    assert plan.sources['ZED YAML'].endswith('zed_vio.yaml')
    assert 'zed_vio.yaml' in plan.text()
    settings.use_zed = False
    with pytest.raises(ValueError, match='VIO'):
        mode_plan(settings, shares)
    settings.odom_source = 'wheel'
    settings.mode = 'navigation'
    settings.use_lidar = False
    with pytest.raises(ValueError, match='LiDAR'):
        mode_plan(settings, shares)


def test_map_inputs_checked_before_start(configuration, tmp_path):
    settings, shares = configuration
    settings.mode = 'navigation'
    plan = mode_plan(settings, shares)
    assert f'waypoints_file:={settings.waypoints}' in plan.jobs[0].command
    assert plan.sources['巡回ルート YAML'] == settings.waypoints
    assert 'enable_panorama:=false' in plan.jobs[0].command
    image = tmp_path / 'map.pgm'
    image.write_text('P2\n1 1\n255\n0\n')
    modified = tmp_path / 'modified.yaml'
    original = yaml.safe_load(Path(settings.navigation_map).read_text())
    original.update(image=str(image), origin=[123., 123., 0.])
    modified.write_text(yaml.safe_dump(original))
    settings.navigation_map = str(modified)
    with pytest.raises(ValueError, match='origin'):
        mode_plan(settings, shares)


def test_offline_plan_only_replays_mapping_in_other_domain(configuration, tmp_path):
    settings, shares = configuration
    topics = {'/scan': 'sensor_msgs/msg/LaserScan', '/odom': 'nav_msgs/msg/Odometry',
              '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage'}
    bag = tmp_path / 'bag'
    bag.mkdir()
    metadata = {'rosbag2_bagfile_information': {'topics_with_message_count': [
        {'topic_metadata': {'name': name, 'type': kind}, 'message_count': 1}
        for name, kind in topics.items()]}}
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    settings.mode, settings.bag_path, settings.offline_domain = 'offline', str(bag), 80
    plan = mode_plan(settings, shares, live_domain=0)
    assert plan.jobs[0].env['ROS_DOMAIN_ID'] == '80'
    assert 'offline_mapping.launch.py' in plan.jobs[0].command
    assert 'base_driver' not in plan.text()
    playback = plan.jobs[1].command
    assert plan.jobs[1].name == 'playback'
    assert '/tf:=/offline/raw_tf' in playback
    assert '/cmd_vel_safe' not in playback
    with pytest.raises(ValueError, match='異なる'):
        mode_plan(settings, shares, live_domain=80)
    metadata['rosbag2_bagfile_information']['topics_with_message_count'].pop()
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    with pytest.raises(ValueError, match='/tf_static'):
        mode_plan(settings, shares)


def test_recording_and_map_save_do_not_overwrite_outputs(tmp_path):
    output = tmp_path / 'recording'
    plan = record_plan(output, ['/tf', '/scan', '/tf'])
    assert plan.jobs[0].command[-2:] == ['/scan', '/tf']
    output.mkdir()
    with pytest.raises(ValueError, match='存在'):
        record_plan(output, ['/tf'])
    output = tmp_path / 'map'
    plan = save_map_plan(output, domain=80)
    assert plan.jobs[0].env == {'ROS_DOMAIN_ID': '80'}
    output.with_suffix('.yaml').write_text('image: map.pgm')
    with pytest.raises(ValueError, match='同名'):
        save_map_plan(output)


def test_replayed_map_tf_removed_but_odometry_and_sensor_tf_preserved():
    def transform(parent, child):
        return SimpleNamespace(header=SimpleNamespace(frame_id=parent), child_frame_id=child)
    assert not keep_transform(transform('/map', '/odom'))
    assert not keep_transform(transform('world', 'map'))
    assert keep_transform(transform('odom', 'base_link'))
    assert keep_transform(transform('base_link', 'laser'))


def test_offline_launch_constructs_without_hardware(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    path = Path(__file__).resolve().parents[1] / 'launch/offline_mapping.launch.py'
    module = runpy.run_path(str(path))
    launch = module['generate_launch_description']()
    assert len(launch.entities) == 5


def test_invalid_profile_rejected_before_gui_start(tmp_path):
    path = tmp_path / 'settings.yaml'
    path.write_text('use_zed: "false"\n')
    with pytest.raises(ValueError, match='型'):
        Settings.load(path)


def test_process_group_shutdown_flushes_owned_job(tmp_path):
    marker = tmp_path / 'flushed'
    ready = tmp_path / 'ready'
    script = tmp_path / 'child.py'
    script.write_text(
        'import signal,time,sys\nfrom pathlib import Path\n'
        f'def stop(*_):\n Path({str(marker)!r}).write_text("flushed");sys.exit(0)\n'
        'signal.signal(signal.SIGINT,stop)\n'
        f'Path({str(ready)!r}).write_text("ready")\n'
        'while True: time.sleep(.02)\n')
    events = []
    manager = ProcessManager(tmp_path / 'logs', event_callback=lambda *args: events.append(args))
    source = tmp_path / 'config.yaml'
    source.write_text('original: 1\n')
    plan = Plan('test', [JobSpec('record', [sys.executable, str(script)])], {'設定': str(source)})
    manager.start(plan)
    source.write_text('edited: 2\n')
    assert Path(plan.sources['起動時コピー (設定)']).read_text() == 'original: 1\n'
    deadline = time.monotonic() + 5
    try:
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert ready.exists()
        manager.stop('record')
        while manager.jobs and time.monotonic() < deadline:
            manager.tick()
            time.sleep(.02)
        assert not manager.jobs
        assert marker.read_text() == 'flushed'
        assert events[-1] == ('record', 'finished', 0)
    finally:
        for job in manager.jobs.values():
            manager.send(job, 9)
            job.process.wait(timeout=2)
            job.output.close()


def test_settings_roundtrip_and_offscreen_widgets(configuration, tmp_path):
    from PyQt5.QtWidgets import QApplication
    from cat_operator_gui.window import OperatorWindow, TopicsDialog

    settings, shares = configuration
    settings.custom_presets = {'画像のみ': ['/camera/image']}
    settings_path = tmp_path / 'settings.yaml'
    settings.save(settings_path)
    assert asdict(Settings.load(settings_path)) == asdict(settings)
    app = QApplication.instance() or QApplication([])
    window = OperatorWindow(settings, shares, settings_path, no_ros=True)
    try:
        window.show()
        app.processEvents()
        assert window.presets.findText('画像のみ') >= 0
        window.presets.setCurrentText('画像のみ')
        assert window.selected_topics == {'/camera/image'}
        dialog = TopicsDialog({}, ['/camera/image'], False, window)
        assert dialog.selection() == ['/camera/image']
        assert window.collect().data_dir == str(tmp_path)
        assert window.live is None
    finally:
        window.close()
        app.processEvents()
