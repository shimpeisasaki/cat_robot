"""No hardware or ROS graph: command handover and stop semantics."""
import math
import runpy
import time
from pathlib import Path
from types import SimpleNamespace, MethodType
from unittest.mock import Mock
from geometry_msgs.msg import Twist

Mux = runpy.run_path(str(Path(__file__).parents[1] / 'scripts/command_mux'))['CommandMux']


def subject():
    now = time.monotonic()
    obj = SimpleNamespace(
        joystick=False, source='manual', pending=None, mode=2, commands={}, buttons=[],
        joy_time=now, smooth_time=now, smoothed=Twist(), switch_time=math.inf,
        accept_after=now - 1., last_tick=now, zero_samples=0,
        motor_time=now, motors_online=True,
        target=Mock(), output=Mock(), lease=Mock(), status=Mock(),
        count_publishers=lambda topic: 1, valid=Mux.valid)
    for name in ['request', 'stop', 'tick', 'command', 'smooth']:
        setattr(obj, name, MethodType(getattr(Mux, name), obj))
    return obj


def velocity(x):
    msg = Twist()
    msg.linear.x = x
    return msg


def test_switch_ramps_to_zero_without_brake_and_discards_old_input():
    obj = subject()
    obj.smoothed = velocity(.4)
    obj.request('external')
    obj.command('external', velocity(.8))
    obj.tick()
    assert obj.target.publish.call_args.args[0].linear.x == 0.
    assert obj.output.publish.call_args.args[0].linear.x == .4
    assert obj.lease.publish.call_args.args[0].data == 2
    assert obj.source == 'manual'
    obj.switch_time -= .2
    for _ in range(3):
        obj.smooth(Twist())
    obj.tick()
    assert obj.source == 'external' and obj.pending is None
    obj.tick()
    assert obj.target.publish.call_args.args[0].linear.x == 0.
    obj.command('external', velocity(.6))
    obj.command('manual', velocity(-.9))
    obj.tick()
    assert obj.target.publish.call_args.args[0].linear.x == .6


def test_angular_motion_prevents_switch():
    obj = subject()
    obj.request('external')
    obj.switch_time -= .2
    msg = Twist()
    msg.angular.z = .1
    for _ in range(4):
        obj.smooth(msg)
    obj.tick()
    assert obj.pending == 'external'


def test_stale_selected_input_does_not_fall_back():
    obj = subject()
    obj.command('external', velocity(.8))
    obj.tick()
    assert obj.target.publish.call_args.args[0].linear.x == 0.
    assert obj.source == 'manual'


def test_brake_interrupts_handover():
    obj = subject()
    obj.request('external')
    obj.stop(0)
    obj.tick()
    assert obj.pending is None and obj.mode == 0
    assert obj.output.publish.call_args.args[0].linear.x == 0.


def test_stale_smoother_or_conflicting_publisher_stops():
    for conflict in [False, True]:
        obj = subject()
        if conflict:
            obj.count_publishers = lambda topic: 2
        else:
            obj.smooth_time -= 1.
        obj.tick()
        assert obj.mode == 0
