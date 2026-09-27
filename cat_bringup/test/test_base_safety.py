"""Mock-only safety tests: no ROS graph, serial ports, or robot motion."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time
import math
import pytest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan

loader = SourceFileLoader('base_safety_impl', str(Path(__file__).parents[1]/'scripts/base_safety'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)
Safety = impl.BaseSafety


def subject():
    now = time.monotonic()
    obj = NS(mode=0, reason='', pending_free=None, pending_drive=None, stopped_since=None,
             ready_since=now-3, armed_at=math.inf, drive_source='navigation',
             raw={'navigation': Twist(), 'manual': Twist()},
             last_command={'navigation': -math.inf, 'manual': -math.inf},
             a=0, b=1, x=2, y=3, buttons=[0]*11,
             output=Mock(), lease=Mock(), status=Mock(),
             get_logger=Mock(), problem=lambda **kwargs: None, physical_problem=lambda: None,
             active={}, cancel_futures={}, rpm=[0, 0], last_tick=now,
             cancel_goals=Mock(), header_ok=lambda *args: True,
             stationary=True, joystick=True, motor_healthy=True, count_publishers=lambda _: 1, fresh={k: now for k in ('joy', 'motor')})
    obj.scan_max_age = .3
    obj.scan_timeout = .5
    obj.scan_frame = 'laser'
    obj.stamps = {}
    obj.stop = lambda reason: Safety.stop(obj, reason)
    obj.request_free = lambda: Safety.request_free(obj)
    obj.activate_drive = lambda source: Safety.activate_drive(obj, source)
    obj.request_drive = lambda source: Safety.request_drive(obj, source)
    return obj


def test_startup_zero_and_arm():
    obj = subject()
    Safety.tick(obj)
    assert obj.output.publish.call_args.args[0].linear.x == 0
    reply = Safety.arm(obj, None, NS(success=False, message=''))
    assert reply.success and obj.mode == 2


def test_b_beats_other_buttons():
    obj = subject(); obj.mode = 2
    msg = Joy(); msg.buttons = [1]*11
    Safety.joy(obj, msg)
    assert obj.mode == 0 and obj.pending_free is None
    assert obj.lease.publish.call_args.args[0].data == 0
    msg.buttons = [1, 0, 1, 0]+[0]*7
    Safety.joy(obj, msg)
    assert obj.mode == 0


def test_y_waits_for_wheels_to_stop():
    obj = subject(); obj.mode = 2; obj.stationary = False
    msg = Joy(); msg.buttons = [0, 0, 0, 1]+[0]*7
    Safety.joy(obj, msg)
    Safety.tick(obj)
    assert obj.mode == 0 and obj.pending_free is not None
    obj.stationary = True; obj.stopped_since = time.monotonic()-.4
    Safety.tick(obj)
    assert obj.mode == 1


def test_free_times_out_when_spinning():
    obj = subject(); obj.pending_free = time.monotonic()-4; obj.stationary = False
    Safety.tick(obj)
    assert obj.mode == 0 and obj.pending_free is None


def test_odom_or_tf_fault_latches_and_does_not_resume():
    obj = subject(); obj.mode = 2
    obj.problem = lambda *args, **kwargs: 'odom stale'
    Safety.tick(obj)
    assert obj.mode == 0
    obj.problem = lambda *args, **kwargs: None
    Safety.tick(obj)
    assert obj.mode == 0


def test_free_brakes_on_joy_loss():
    obj = subject(); obj.mode = 1
    obj.physical_problem = lambda: 'joy stale'
    Safety.tick(obj)
    assert obj.mode == 0


def test_existing_goal_can_be_explicitly_rearmed():
    obj = subject(); obj.active = {'navigate_to_pose': True}
    reply = Safety.arm(obj, None, NS(success=False, message=''))
    assert reply.success and obj.drive_source == 'navigation'


def test_a_selects_navigation_and_x_selects_manual():
    obj = subject()
    msg = Joy(); msg.buttons = [1, 0, 0, 0]+[0]*7
    Safety.joy(obj, msg)
    assert obj.pending_drive == 'navigation'
    msg.buttons = [0]*11; Safety.joy(obj, msg)
    obj.ready_since = time.monotonic()-3
    msg.buttons = [0, 0, 1, 0]+[0]*7
    Safety.joy(obj, msg)
    Safety.tick(obj)
    assert obj.mode == 2 and obj.drive_source == 'manual'


def test_manual_mode_does_not_require_map_to_odom():
    obj = subject()
    obj.pending_drive = 'manual'
    obj.problem = lambda **kwargs: 'Nav2, sensor, and TF failures must be ignored for manual drive'
    obj.ready_since = time.monotonic() - 3
    Safety.tick(obj)
    assert obj.mode == 2 and obj.drive_source == 'manual'
    # The same localization loss remains a stop condition for navigation.
    obj.mode = 2
    obj.drive_source = 'navigation'
    Safety.tick(obj)
    assert obj.mode == 0 and 'Nav2, sensor, and TF failures' in obj.reason


def test_old_commands_never_replayed():
    obj = subject()
    obj.raw['navigation'].linear.x = .2
    obj.last_command['navigation'] = time.monotonic()-1
    assert Safety.arm(obj, None, NS(success=False, message='')).success
    Safety.tick(obj)
    assert obj.output.publish.call_args.args[0].linear.x == 0.
    msg = Twist(); msg.linear.x = .1
    Safety.command(obj, 'navigation', msg); Safety.tick(obj)
    assert obj.output.publish.call_args.args[0].linear.x == .1
    obj.last_command['navigation'] -= 1
    Safety.tick(obj)
    assert obj.output.publish.call_args.args[0].linear.x == 0.


def test_timer_stall_brakes():
    obj = subject(); obj.mode = 2; obj.last_tick -= 1
    Safety.tick(obj)
    assert obj.mode == 0


def test_invalid_commands_brake():
    obj = subject(); obj.mode = 2
    msg = Twist(); msg.linear.x = float('nan')
    Safety.command(obj, 'navigation', msg)
    assert obj.mode == 0


def test_physical_freshness():
    obj = subject()
    assert Safety.physical_problem(obj) is None
    obj.fresh['joy'] -= 1
    assert 'joy' in Safety.physical_problem(obj)


def test_free_to_drive_request_brakes_then_arms_when_stationary():
    obj = subject(); obj.mode = 1
    Safety.request_drive(obj, 'navigation')
    assert obj.mode == 0 and obj.pending_drive == 'navigation'
    obj.ready_since = time.monotonic()-3
    Safety.tick(obj)
    assert obj.mode == 2 and obj.drive_source == 'navigation'
