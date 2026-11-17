"""Inspect launch actions only. Never execute a launch or start hardware."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch_ros.actions import Node


def load(name):
    path = Path(__file__).parents[1] / 'launch' / (name + '.launch.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(**values):
    result = LaunchContext()
    result.launch_configurations.update(values)
    return result


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
