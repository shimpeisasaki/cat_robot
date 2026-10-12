"""Header and scan validation without a ROS graph."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
from types import SimpleNamespace as NS
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
