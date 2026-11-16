"""One sensor epoch across cloud/IMU; gaps and resets must stay visible."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path

import pytest

loader = SourceFileLoader('horizon_clock_impl', str(
    Path(__file__).parents[1] / 'scripts/horizon_time_bridge'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)
N = 1_000_000_000


def sample(clock, stream, source, host, steady=None):
    return clock.observe(stream, round(source*N), round(host*N),
                         round((host if steady is None else steady)*N))


def ready():
    clock = impl.SensorClock()
    assert sample(clock, 'cloud', 100., 1000.1) is None
    assert sample(clock, 'imu', 100.1, 1000.11) is None
    assert sample(clock, 'cloud', 101., 1001.1) is None
    assert sample(clock, 'imu', 101.1, 1001.11) == round(1001.11*N)
    return clock


def test_shared_minimum_delay_offset_preserves_relative_acquisition_time():
    clock = ready()
    assert clock.offset_ns == round(900.01*N)
    cloud = sample(clock, 'cloud', 101.11, 1001.22)
    imu = sample(clock, 'imu', 101.23, 1001.24)
    assert imu - cloud == round(.12*N)
    assert cloud == round(1001.12*N)  # Not rewritten to its arrival time.


def test_warmup_requires_both_streams_and_does_not_replay_buffered_data():
    clock = impl.SensorClock()
    assert sample(clock, 'cloud', 100., 1000.) is None
    assert sample(clock, 'cloud', 102., 1002.) is None
    assert clock.offset_ns is None
    assert sample(clock, 'imu', 102.01, 1002.01) == round(1002.01*N)


def test_delayed_input_is_dropped_without_reanchoring_then_fresh_input_recovers():
    clock = ready()
    offset = clock.offset_ns
    assert sample(clock, 'cloud', 101.1, 1001.8) is None
    assert clock.reason == 'delayed_input'
    assert sample(clock, 'cloud', 101.9, 1001.91) is not None
    assert clock.offset_ns == offset


def test_duplicate_and_small_reordering_are_not_republished():
    clock = ready()
    assert sample(clock, 'imu', 101.1, 1001.12) is None
    assert sample(clock, 'imu', 101.09, 1001.13) is None
    assert not clock.fault
    assert sample(clock, 'imu', 101.14, 1001.15) is not None


def test_sensor_reset_latches_both_streams_off():
    clock = ready()
    assert sample(clock, 'imu', .1, 1001.2) is None
    assert 'sensor_clock_reset' in clock.fault
    assert sample(clock, 'cloud', 101.3, 1001.31) is None


@pytest.mark.parametrize('jump', [-1., 1.])
def test_host_clock_jump_latches_output_off(jump):
    clock = ready()
    assert sample(clock, 'imu', 101.2, 1001.21+jump, steady=1001.21) is None
    assert 'host_clock_jump' in clock.fault


def test_sensor_forward_jump_cannot_create_future_ground():
    clock = ready()
    assert sample(clock, 'imu', 105., 1001.2) is None
    assert 'sensor_clock_ahead' in clock.fault


def test_sensor_jump_during_warmup_cannot_be_hidden_by_a_new_offset():
    clock = impl.SensorClock()
    assert sample(clock, 'imu', 100., 1000.) is None
    assert sample(clock, 'imu', 110., 1000.1) is None
    assert not clock.fault and clock.offset_ns is None
    assert clock.warmup_resets == 1
    assert sample(clock, 'cloud', 110.1, 1000.2) is None
    assert sample(clock, 'cloud', 111.15, 1001.25) is None  # IMU still stale.
    assert sample(clock, 'imu', 111.2, 1001.3) == round(1001.3*N)
    assert clock.offset_ns == round(890.1*N)


def test_sensor_reset_during_warmup_discards_the_old_epoch():
    clock = impl.SensorClock()
    assert sample(clock, 'cloud', 100., 1000.) is None
    assert sample(clock, 'imu', 100., 1000.01) is None
    assert sample(clock, 'cloud', 1., 1000.1) is None
    assert not clock.fault and clock.offset_ns is None
    assert clock.last_source == {'cloud': 1_000_000_000}
    assert clock.warmup_resets == 1


@pytest.mark.parametrize('source', [0, -1])
def test_invalid_stamp_does_not_anchor(source):
    clock = impl.SensorClock()
    assert sample(clock, 'cloud', source, 1000.) is None
    assert clock.started_ns is None


@pytest.mark.parametrize('value', [float('nan'), float('inf'), 0., -1.])
def test_invalid_configuration_is_rejected(value):
    with pytest.raises(ValueError):
        impl.SensorClock(max_delay=value)
