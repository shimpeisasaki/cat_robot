"""Route sequencing and cancellation against asynchronous action handles."""
import importlib.util
from importlib.machinery import SourceFileLoader
from concurrent.futures import Future
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace as NS
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from cat_control_interfaces.msg import ControlState
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
loader = SourceFileLoader('waypoint_route_impl', str(
    Path(__file__).parents[1] / 'scripts/waypoint_route'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)


def subject(monkeypatch, count=3, delayed_acceptance=False):
    clock = NS(now=100.)
    monkeypatch.setattr(impl.time, 'monotonic', lambda: clock.now)
    obj = NS(points=[(float(i), 0., 0.) for i in range(count)],
             intermediate_tree='intermediate.xml', final_tree='final.xml',
             goal=None, pending=False, cancel_requested=False, route_active=False,
             navigation_enabled=True, base_time=100., index=0, missed=[],
             advance_at=None, report=Mock(), status=Mock(), client=Mock())
    obj.poses = lambda: impl.make_poses(obj.points, impl.rclpy.time.Time().to_msg())
    for name in ('behavior_tree_for', 'start', 'send_waypoint', 'tick', 'accepted',
                 'finished', 'finish_route', 'stop_route', 'request_cancel',
                 'canceled', 'cancel', 'feedback', 'base_status'):
        setattr(obj, name, MethodType(getattr(impl.WaypointRoute, name), obj))
    sends, results, handles = [], [], []

    def send(goal, **kwargs):
        result = Future()
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        cancel = Future()
        cancel.set_result(NS(goals_canceling=[object()]))
        handle.cancel_goal_async.return_value = cancel
        accepted = Future()
        if not delayed_acceptance:
            accepted.set_result(handle)
        sends.append(accepted)
        results.append(result)
        handles.append(handle)
        return accepted

    obj.client.send_goal_async.side_effect = send
    return obj, clock, sends, results, handles


def test_sequence_uses_intermediate_then_final_and_preserves_pause(monkeypatch):
    obj, clock, _, results, _ = subject(monkeypatch)
    assert obj.start(None, Trigger.Response()).success
    for i in range(3):
        goal = obj.client.send_goal_async.call_args.args[0]
        assert goal.pose.pose.position.x == i
        assert goal.behavior_tree == ('final.xml' if i == 2 else 'intermediate.xml')
        results[i].set_result(NS(status=GoalStatus.STATUS_SUCCEEDED))
        obj.tick()
        assert obj.client.send_goal_async.call_count == i + 1
        clock.now += .21
        obj.base_time = clock.now
        obj.tick()
    assert not obj.route_active
    obj.report.assert_called_with('Route completed')


def test_single_waypoint_uses_final_checker(monkeypatch):
    obj, _, _, _, _ = subject(monkeypatch, count=1)
    obj.start(None, Trigger.Response())
    assert obj.client.send_goal_async.call_args.args[0].behavior_tree == 'final.xml'


def test_cancel_while_waiting_for_acceptance_never_sends_next(monkeypatch):
    obj, _, sends, results, handles = subject(monkeypatch, delayed_acceptance=True)
    obj.start(None, Trigger.Response())
    assert obj.cancel(None, Trigger.Response()).success
    sends[0].set_result(handles[0])
    handles[0].cancel_goal_async.assert_called_once()
    # Even success racing with cancellation must not advance the route.
    results[0].set_result(NS(status=GoalStatus.STATUS_SUCCEEDED))
    obj.tick()
    assert not obj.route_active
    assert obj.client.send_goal_async.call_count == 1


def test_disable_during_pause_cancels_route(monkeypatch):
    obj, clock, _, results, _ = subject(monkeypatch)
    obj.start(None, Trigger.Response())
    results[0].set_result(NS(status=GoalStatus.STATUS_SUCCEEDED))
    obj.base_status(ControlState(mode=ControlState.BRAKE))
    clock.now += 1.
    obj.tick()
    assert not obj.route_active
    assert obj.client.send_goal_async.call_count == 1


def test_aborted_waypoint_is_reported_and_skipped(monkeypatch):
    obj, _, _, results, _ = subject(monkeypatch)
    obj.start(None, Trigger.Response())
    results[0].set_result(NS(status=GoalStatus.STATUS_ABORTED))
    obj.tick()
    assert obj.missed == [1]
    assert obj.client.send_goal_async.call_count == 2


def test_stale_base_status_cancels_active_goal(monkeypatch):
    obj, clock, _, _, handles = subject(monkeypatch)
    obj.start(None, Trigger.Response())
    clock.now += .4
    obj.tick()
    handles[0].cancel_goal_async.assert_called_once()
    assert obj.cancel_requested
