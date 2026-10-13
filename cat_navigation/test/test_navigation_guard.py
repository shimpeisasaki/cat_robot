"""Header and scan validation without a ROS graph."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from cat_control_interfaces.msg import ControlState
from sensor_msgs.msg import LaserScan
loader = SourceFileLoader('navigation_guard_impl', str(Path(__file__).parents[1]/'scripts/navigation_guard'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)
Safety = impl.NavigationGuard

def scan_subject():
    obj = NS(fresh={}, stamps={}, scan_max_age=.3, scan_timeout=.5, scan_frame='laser')
    obj.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000))
    obj.header_ok = lambda *args: Safety.header_ok(obj, *args)
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.header.stamp.sec = 99
    msg.header.stamp.nanosec = 900_000_000
    msg.range_min, msg.range_max, msg.ranges = .1, 40., [1.]
    return obj, msg


def test_invalid_scans_do_not_refresh_watchdog():
    obj, msg = scan_subject()
    Safety.scan(obj, msg)
    accepted = obj.fresh['scan']
    # Repeated stamp, old data, future data, wrong frame, empty returns.
    Safety.scan(obj, msg)
    msg.header.stamp.sec = 98
    Safety.scan(obj, msg)
    msg.header.stamp.sec = 101
    Safety.scan(obj, msg)
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = 'wrong'
    Safety.scan(obj, msg)
    msg.header.frame_id = 'laser'
    msg.ranges = [float('nan')]
    Safety.scan(obj, msg)
    assert obj.fresh['scan'] == accepted


def test_navigation_permission_does_not_depend_on_display_reason():
    client = Mock(srv_name='/navigate_to_pose/_action/cancel_goal')
    obj = NS(last_cancel=-float('inf'), cancel_clients=[client], cancel_futures={})
    Safety.base_status(obj, ControlState(mode=ControlState.DRIVE, source='navigation',
                                        reason='文言を変更しても走行許可は同じ'))
    client.call_async.assert_not_called()
    Safety.base_status(obj, ControlState(mode=ControlState.DRIVE, source='manual'))
    client.call_async.assert_called_once()


@pytest.mark.parametrize('map_age,odom_age,expected', [
    (-.2, .02, True),  # EMCL's normal future-dated transform.
    (-.4, .02, False),  # Beyond the configured future allowance.
    (1.6, .02, False),  # Stale localization must still stop navigation.
    (-.2, -.2, False),  # Odom retains its stricter clock check.
    (-.2, .6, False),  # Stale odom must still stop navigation.
])
def test_tf_time_checks_respect_emcl_tolerance(map_age, odom_age, expected):
    def lookup(parent, child, stamp):
        value = 100 - (map_age if parent == 'map' else odom_age)
        return NS(header=NS(stamp=NS(sec=int(value),
                                    nanosec=round((value-int(value))*1e9))))

    obj = NS(sensor_problem=lambda: None, map_tf_future_tolerance=.3,
             get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)),
             buffer=NS(lookup_transform=lookup), ready=Mock(),
             get_publishers_info_by_topic=lambda _: [NS(node_name='collision_monitor')])
    Safety.tick(obj)
    assert obj.ready.publish.call_args.args[0].data == expected
