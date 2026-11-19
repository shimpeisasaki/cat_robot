"""Inspect launch actions only. Never execute a launch or start hardware."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


def load(name):
    package = (Path(__file__).parents[2] / 'cat_bringup' if name == 'bringup'
               else Path(__file__).parents[1])
    path = package / 'launch' / (name + '.launch.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(**values):
    result = LaunchContext()
    result.launch_configurations.update(values)
    return result


@pytest.mark.parametrize('entry', ['mapping', 'navigation'])
@pytest.mark.parametrize('mode', ['off', 'standalone', 'rtk'])
def test_gnss_mode_and_custom_inputs_reach_bringup(entry, mode):
    description = load(entry).generate_launch_description()
    ctx = context(gnss_mode=mode, rtk_source='matsudo', gnss_serial='test_serial',
                  serial_port='/dev/test_scan', rplidar_mount='custom',
                  rplidar_roll='.25', rplidar_yaw='.75')
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    include = next(action for action in description.entities
                   if isinstance(action, IncludeLaunchDescription)
                   and 'gnss_mode' in dict(action.launch_arguments))
    forwarded = {key: perform_substitutions(ctx, normalize_to_list_of_substitutions(value))
                 for key, value in include.launch_arguments}
    assert forwarded['gnss_mode'] == mode
    assert forwarded['rtk_source'] == 'matsudo'
    assert forwarded['gnss_serial'] == 'test_serial'
    assert forwarded['serial_port'] == '/dev/test_scan'
    assert forwarded['rplidar_mount'] == 'custom'
    assert forwarded['rplidar_roll'] == '.25' and forwarded['rplidar_yaw'] == '.75'


@pytest.mark.parametrize('entry,source,zed', [
    ('bringup', 'vio', 'true'), ('mapping', 'wheel', 'false'), ('navigation', 'wheel', 'false'),
])
def test_only_current_gnss_and_mount_entrances_are_declared(entry, source, zed):
    declarations = {action.name: action for action in load(entry).generate_launch_description().entities
                    if isinstance(action, DeclareLaunchArgument)}
    assert 'use_gnss' not in declarations
    assert declarations['rplidar_mount'].choices == ['front', 'custom']
    ctx = context()
    for action in declarations.values():
        action.execute(ctx)
    assert ctx.launch_configurations['gnss_mode'] == 'off'
    assert ctx.launch_configurations['odom_source'] == source
    assert ctx.launch_configurations['use_zed'] == zed
    if entry == 'navigation':
        assert 'localization_map' in declarations


@pytest.mark.parametrize('mode,enabled,filename', [
    ('off', False, 'gnss_launch_compatible.launch.py'),
    ('standalone', True, 'gnss_launch_compatible.launch.py'),
    ('rtk', True, 'gnss_rtk.launch.py'),
])
def test_bringup_selects_gnss_receiver_and_forwards_correction_inputs(mode, enabled, filename):
    description = load('bringup').generate_launch_description()
    ctx = context(gnss_mode=mode, rtk_source='matsudo', gnss_serial='test_receiver')
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    include = next(action for action in description.entities
                   if isinstance(action, IncludeLaunchDescription)
                   and 'device_serial_string' in dict(action.launch_arguments))
    assert include.condition.evaluate(ctx) == enabled
    # Both receiver files only construct actions; never execute their containers
    # or RTK OpaqueFunction. location is expanded by this public loading API.
    include.launch_description_source.get_launch_description(ctx)
    assert Path(include.launch_description_source.location).name == filename
    forwarded = {key: perform_substitutions(ctx, normalize_to_list_of_substitutions(value))
                 for key, value in include.launch_arguments}
    assert forwarded == {'rtk_source': 'matsudo', 'frame_id': 'gnss_antenna_link',
                         'device_serial_string': 'test_receiver'}


@pytest.mark.parametrize('mount,expected', [('front', '0'), ('custom', '.25')])
def test_mount_keeps_front_geometry_and_custom_override(mount, expected):
    module = load('bringup')
    value = module.rplidar_mount_value('rplidar_roll', '0')
    assert value.perform(context(rplidar_mount=mount, rplidar_roll='.25')) == expected


def test_disabled_does_not_require_calibration():
    module = load('localization')
    assert module.gnss_transform(context(use_gnss_reset='false')) == []


def test_enabled_requires_explicit_existing_calibration(tmp_path):
    module = load('localization')
    with pytest.raises(RuntimeError):
        module.gnss_transform(context(use_gnss_reset='true', gnss_map_params_file=''))
    path = tmp_path / 'calibration.yaml'
    path.write_text('/**: {ros__parameters: {}}')
    actions = module.gnss_transform(context(use_gnss_reset='true',
                                          gnss_map_params_file=str(path), use_sim_time='false'))
    assert len(actions) == 1 and isinstance(actions[0], Node)


def test_navigation_rejects_slam_before_creating_nodes():
    module = load('navigation')
    with pytest.raises(ValueError, match='saved-map'):
        module.map_nodes(context(slam='true', use_gnss_reset='true'))
