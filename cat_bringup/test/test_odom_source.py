"""No hardware or ROS graph: numerical conversion and publication contract."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

loader = SourceFileLoader('odom_source_impl', str(Path(__file__).parents[1]/'scripts/odom_source'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)


def subject(source):
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    # T_camera_base, camera 0.27m behind and 0.56m above base.
    tf.transform.translation.x = .27
    tf.transform.translation.z = -.56
    obj = NS(source=source, pose_mode='planar', failed=False, origin=None, previous=None, last_stamp=None,
             input_frames=None, vio_jump_since=None, vio_jump_grace=.5,
             pub=Mock(), tf=Mock(), buffer=Mock(), get_logger=Mock(),
             get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)))
    obj.buffer.lookup_transform.return_value = tf
    obj.fail = lambda reason: impl.OdomSource.fail(obj, reason)
    return obj


def message(t=10., wheel=False):
    msg = Odometry()
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = round((t-int(t))*1e9)
    msg.header.frame_id = 'wheel_odom' if wheel else 'zed_odom'
    msg.child_frame_id = 'base_link' if wheel else 'zed_camera_link'
    msg.pose.pose.orientation.w = 1.
    return msg


def test_wheel_relay_and_single_tf():
    obj = subject('wheel'); msg = message(wheel=True)
    msg.pose.pose.position.x = 3.
    msg.twist.twist.linear.x = .2
    impl.OdomSource.receive(obj, msg)
    out = obj.pub.publish.call_args.args[0]
    assert out.header.frame_id == 'odom' and out.child_frame_id == 'base_link'
    assert out.pose.pose.position.x == 3. and out.twist.twist.linear.x == .2
    assert msg.header.frame_id == 'wheel_odom'
    assert obj.tf.sendTransform.call_count == 1


def test_vio_zero_origin_translation_and_velocity():
    obj = subject('vio'); msg = message(9.9)
    impl.OdomSource.receive(obj, msg)
    assert obj.pub.publish.call_args.args[0].pose.pose.position.x == 0.
    msg = message(); msg.pose.pose.position.x = .02
    impl.OdomSource.receive(obj, msg)
    out = obj.pub.publish.call_args.args[0]
    assert np.isclose(out.pose.pose.position.x, .02)
    assert np.isclose(out.twist.twist.linear.x, .2)
    assert out.pose.pose.position.z == 0.


def test_camera_lever_arm_during_base_spin():
    obj = subject('vio')
    impl.OdomSource.receive(obj, message(9.9))
    msg = message()
    theta = .05
    # Base remains at x=.27; the rear-mounted camera moves around it.
    msg.pose.pose.position.x = .27*(1-np.cos(theta))
    msg.pose.pose.position.y = -.27*np.sin(theta)
    msg.pose.pose.orientation.z = np.sin(theta/2)
    msg.pose.pose.orientation.w = np.cos(theta/2)
    impl.OdomSource.receive(obj, msg)
    out = obj.pub.publish.call_args.args[0]
    assert abs(out.pose.pose.position.x) < 1e-10
    assert abs(out.pose.pose.position.y) < 1e-10
    assert abs(out.twist.twist.linear.x) < 1e-10
    assert np.isclose(out.twist.twist.angular.z, .5)


def test_gap_latches_output_off():
    obj = subject('vio'); obj.last_stamp = 8.
    impl.OdomSource.receive(obj, message())
    assert obj.failed
    obj.pub.publish.assert_not_called()


def test_single_pose_jump_is_dropped_and_good_sample_recovers():
    obj = subject('vio')
    impl.OdomSource.receive(obj, message(9.9))
    msg = message(); msg.pose.pose.position.x = 5.
    impl.OdomSource.receive(obj, msg)
    assert not obj.failed and obj.pub.publish.call_count == 1
    msg = message(10.05); msg.pose.pose.position.x = .01
    impl.OdomSource.receive(obj, msg)
    assert not obj.failed and obj.pub.publish.call_count == 2
    assert obj.vio_jump_since is None


def test_pose_jumps_inside_grace_period_do_not_latch_output_off():
    obj = subject('vio')
    impl.OdomSource.receive(obj, message(9.5))
    msg = message(9.51); msg.pose.pose.position.x = 5.
    impl.OdomSource.receive(obj, msg)
    msg = message(10.0); msg.pose.pose.position.x = 5.1
    impl.OdomSource.receive(obj, msg)
    assert not obj.failed and obj.pub.publish.call_count == 1


def test_pose_jumps_beyond_grace_period_latch_output_off():
    obj = subject('vio')
    impl.OdomSource.receive(obj, message(9.5))
    msg = message(9.51); msg.pose.pose.position.x = 5.
    impl.OdomSource.receive(obj, msg)
    msg = message(10.01); msg.pose.pose.position.x = 5.1
    impl.OdomSource.receive(obj, msg)
    assert obj.failed and obj.pub.publish.call_count == 1


def test_old_and_duplicate_messages_not_republished():
    obj = subject('wheel')
    impl.OdomSource.receive(obj, message(1., wheel=True))
    assert obj.pub.publish.call_count == 0
    impl.OdomSource.receive(obj, message(wheel=True))
    impl.OdomSource.receive(obj, message(wheel=True))
    assert obj.pub.publish.call_count == 1


def test_yaw_wrap():
    v = impl.velocity(np.array([0., 0., np.pi-.01]), np.array([0., 0., -np.pi+.01]), .1)
    assert np.isclose(v[2], .2)


def test_starting_heading_normalized():
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('z', 1.2).as_matrix()
    t[:3, 3] = [2., 3., .5]
    assert np.allclose(impl.origin_for(t)@t, np.eye(4))


def spatial_message(obj, base, stamp):
    mounting = obj.buffer.lookup_transform.return_value.transform
    camera = base @ np.linalg.inv(impl.matrix(mounting.translation, mounting.rotation))
    msg = message(stamp)
    for axis, value in zip(('x', 'y', 'z'), camera[:3, 3]):
        setattr(msg.pose.pose.position, axis, float(value))
    for axis, value in zip(('x', 'y', 'z', 'w'), Rotation.from_matrix(camera[:3, :3]).as_quat()):
        setattr(msg.pose.pose.orientation, axis, float(value))
    return msg


def test_spatial_vio_preserves_slope_height_and_attitude_in_single_tf():
    obj = subject('vio')
    obj.pose_mode = 'spatial'
    base = np.eye(4)
    base[:3, :3] = Rotation.from_euler('xyz', [.05, -.12, .7]).as_matrix()
    base[:3, 3] = [2., 3., .2]
    origin = impl.origin_for(base)
    impl.OdomSource.receive(obj, spatial_message(obj, base, 9.9))
    base[:3, 3] += [.01, .01, .005]
    base[:3, :3] = base[:3, :3] @ Rotation.from_euler('y', -.01).as_matrix()
    impl.OdomSource.receive(obj, spatial_message(obj, base, 10.))
    out = obj.pub.publish.call_args.args[0]
    actual = impl.matrix(out.pose.pose.position, out.pose.pose.orientation)
    assert np.allclose(actual, origin @ base)
    assert np.isclose(out.pose.pose.position.z, .005)
    assert abs(out.pose.pose.orientation.x) > .01
    assert abs(out.pose.pose.orientation.y) > .01
    tf = obj.tf.sendTransform.call_args.args[0]
    assert np.allclose(impl.matrix(tf.transform.translation, tf.transform.rotation), actual)
    assert obj.tf.sendTransform.call_count == 2
    assert np.all(np.diag(np.array(out.pose.covariance).reshape(6, 6)) < 1e6)


def test_spatial_velocity_uses_body_frame_and_pitch_rate():
    previous = np.eye(4)
    current = np.eye(4)
    current[:3, :3] = Rotation.from_euler('y', -.1).as_matrix()
    current[:3, 3] = current[:3, :3] @ [.02, 0., 0.]
    speed = impl.spatial_velocity(previous, current, .1)
    assert np.allclose(speed[:3], [.2, 0., 0.])
    assert np.allclose(speed[3:], [0., -1., 0.])


def test_spatial_camera_lever_arm_during_pitch_does_not_translate_base():
    obj = subject('vio')
    obj.pose_mode = 'spatial'
    base = np.eye(4)
    impl.OdomSource.receive(obj, spatial_message(obj, base, 9.9))
    base[:3, :3] = Rotation.from_euler('y', .05).as_matrix()
    impl.OdomSource.receive(obj, spatial_message(obj, base, 10.))
    out = obj.pub.publish.call_args.args[0]
    p = out.pose.pose.position
    assert np.allclose([p.x, p.y, p.z], 0.)
    v = out.twist.twist.linear
    assert np.allclose([v.x, v.y, v.z], 0.)
    assert np.isclose(out.twist.twist.angular.y, .5)


def test_spatial_vertical_jump_is_dropped_and_gap_latches_off():
    obj = subject('vio')
    obj.pose_mode = 'spatial'
    base = np.eye(4)
    impl.OdomSource.receive(obj, spatial_message(obj, base, 9.5))
    base[2, 3] = 5.
    impl.OdomSource.receive(obj, spatial_message(obj, base, 9.51))
    assert not obj.failed and obj.pub.publish.call_count == 1
    impl.OdomSource.receive(obj, spatial_message(obj, base, 10.01))
    assert obj.failed and obj.pub.publish.call_count == 1


def test_spatial_roll_jump_is_rejected_even_when_yaw_is_unchanged():
    obj = subject('vio')
    obj.pose_mode = 'spatial'
    base = np.eye(4)
    impl.OdomSource.receive(obj, spatial_message(obj, base, 9.9))
    base[:3, :3] = Rotation.from_euler('x', 1.).as_matrix()
    impl.OdomSource.receive(obj, spatial_message(obj, base, 10.))
    assert obj.pub.publish.call_count == 1
