"""Shared route YAML format and map-frame visualizations."""
import math
import os
import tempfile
from pathlib import Path

import yaml
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


def load_waypoints(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or data.get('frame_id', 'map') != 'map':
        raise ValueError('Waypoint file must be a mapping with frame_id: map')
    points = data.get('waypoints')
    if not isinstance(points, list) or not points:
        raise ValueError('waypoints must be a non-empty list')
    result = []
    for i, point in enumerate(points, 1):
        if not isinstance(point, dict) or not {'x', 'y'} <= point.keys():
            raise ValueError(f'Waypoint {i} requires x and y')
        if point.keys() - {'x', 'y', 'yaw'}:
            raise ValueError(f'Waypoint {i}: supported fields are x, y, yaw (radians)')
        values = [point['x'], point['y'], point.get('yaw', 0.0)]
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and
                   math.isfinite(v) for v in values):
            raise ValueError(f'Waypoint {i}: x, y, yaw must be finite numbers')
        result.append(tuple(float(v) for v in values))
    return result


def make_poses(points, stamp):
    poses = []
    for x, y, yaw in points:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        poses.append(pose)
    return poses


def make_markers(points, stamp):
    clear = Marker(action=Marker.DELETEALL)
    array = MarkerArray(markers=[clear])
    for i, pose in enumerate(make_poses(points, stamp)):
        arrow = Marker()
        arrow.header = pose.header
        arrow.ns, arrow.id = 'route', i
        arrow.type, arrow.action = Marker.ARROW, Marker.ADD
        arrow.pose = pose.pose
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .4, .08, .08
        arrow.color.g, arrow.color.b, arrow.color.a = .8, 1., 1.
        label = Marker()
        label.header = pose.header
        label.ns, label.id = 'route_labels', i
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x, label.pose.position.y = pose.pose.position.x, pose.pose.position.y
        label.pose.position.z = .4
        label.pose.orientation.w = 1.
        label.scale.z = .2
        label.color.r = label.color.g = label.color.b = label.color.a = 1.
        label.text = str(i + 1)
        array.markers.extend([arrow, label])
    return array


def save_waypoints(path, points):
    if not points:
        raise ValueError('No waypoints to save')
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    data = {'frame_id': 'map', 'waypoints': [
        {'x': float(x), 'y': float(y), 'yaw': float(yaw)} for x, y, yaw in points]}
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                         dir=target.parent, delete=False) as stream:
            temporary = stream.name
            yaml.safe_dump(data, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)
    return target
