"""Small, deterministic VLP-16 ray caster for offline terrain experiments."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


# Nominal channel order and vertical offsets from velodyne_pointcloud's
# VLP16db.yaml. Individual real sensors can differ from this calibration.
VLP16_ANGLES_DEG = [-15, 1, -13, 3, -11, 5, -9, 7,
                    -7, 9, -5, 11, -3, 13, -1, 15]
VLP16_OFFSETS_M = [0.0112, -0.0007, 0.0097, -0.0022,
                   0.0081, -0.0037, 0.0066, -0.0051,
                   0.0051, -0.0066, 0.0037, -0.0081,
                   0.0022, -0.0097, 0.0007, -0.0112]


def _numbers(values, count, label):
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (count,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{label} must contain {count} finite numbers')
    return result


def _bounds(values, label):
    result = _numbers(values, 4, label)
    if result[0] >= result[1] or result[2] >= result[3]:
        raise ValueError(f'{label} requires xmin < xmax and ymin < ymax')
    return result


def _rotation(rpy_deg):
    roll, pitch, yaw = np.deg2rad(rpy_deg)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


@dataclass
class Scan:
    sensor_points: np.ndarray
    world_points: np.ndarray
    rings: np.ndarray


class TerrainScene:
    """Cast nominal VLP-16 rays against horizontal patches and solid boxes."""

    def __init__(self, description):
        if not isinstance(description, dict):
            raise ValueError('scene root must be a YAML mapping')
        sensor = description.get('sensor', {})
        robot = description.get('robot', {})
        if not isinstance(sensor, dict) or not isinstance(robot, dict):
            raise ValueError('sensor and robot must be YAML mappings')

        self.sensor_xyz = _numbers(
            sensor.get('xyz_m', [-0.347981, 0, 0.515158]), 3, 'sensor.xyz_m')
        self.sensor_rpy_deg = _numbers(
            sensor.get('rpy_deg', [160, 0, -90]), 3, 'sensor.rpy_deg')
        self.mount_rotation = _rotation(self.sensor_rpy_deg)
        self.vertical_angles_deg = _numbers(
            sensor.get('vertical_angles_deg', VLP16_ANGLES_DEG), 16,
            'sensor.vertical_angles_deg')
        self.vertical_offsets_m = _numbers(
            sensor.get('vertical_offsets_m', VLP16_OFFSETS_M), 16,
            'sensor.vertical_offsets_m')
        self.azimuth_samples = int(sensor.get('azimuth_samples', 900))
        self.rate_hz = float(sensor.get('rate_hz', 10.0))
        self.min_range_m = float(sensor.get('min_range_m', 0.1))
        self.max_range_m = float(sensor.get('max_range_m', 30.0))
        self.noise_std_m = float(sensor.get('range_noise_std_m', 0.0))
        self.dropout_probability = float(sensor.get('dropout_probability', 0.0))
        if (self.azimuth_samples < 16 or self.azimuth_samples > 10000 or
                not np.isfinite([self.rate_hz, self.min_range_m,
                                 self.max_range_m, self.noise_std_m,
                                 self.dropout_probability]).all() or
                self.rate_hz <= 0 or self.min_range_m < 0 or
                self.max_range_m <= self.min_range_m or
                self.noise_std_m < 0 or
                not 0 <= self.dropout_probability < 1):
            raise ValueError('invalid sensor sampling, range, or noise parameters')

        self.initial_pose = _numbers(
            robot.get('pose_xyzyaw_deg', [0, 0, 0, 0]), 4,
            'robot.pose_xyzyaw_deg')
        self.speed_mps = float(robot.get('linear_speed_mps', 0.0))
        self.yaw_rate_deg_s = float(robot.get('yaw_rate_deg_s', 0.0))
        if not np.isfinite([self.speed_mps, self.yaw_rate_deg_s]).all():
            raise ValueError('robot speed and yaw rate must be finite')
        self.motion = []
        for segment in robot.get('motion', []):
            duration = float(segment['duration_s'])
            speed = float(segment.get('linear_speed_mps', 0.0))
            turn = float(segment.get('yaw_rate_deg_s', 0.0))
            if (not np.isfinite([duration, speed, turn]).all() or
                    duration <= 0):
                raise ValueError('motion segments need positive finite duration')
            self.motion.append((duration, speed, turn))

        self.surfaces = []
        self.boxes = []
        for item in description.get('surfaces', []):
            name = item.get('name', 'surface')
            bounds = _bounds(item['bounds_xy_m'], f'{name}.bounds_xy_m')
            height = float(item['height_m'])
            if not np.isfinite(height):
                raise ValueError(f'{name}.height_m must be finite')
            holes = [_bounds(hole, f'{name}.holes') for hole in item.get('holes', [])]
            self.surfaces.append((bounds, height, holes))
        for item in description.get('boxes', []):
            name = item.get('name', 'box')
            center = _numbers(item['center_m'], 3, f'{name}.center_m')
            size = _numbers(item['size_m'], 3, f'{name}.size_m')
            yaw = float(item.get('yaw_deg', 0.0))
            if np.any(size <= 0) or not np.isfinite(yaw):
                raise ValueError(f'{name} must have positive size and finite yaw')
            self.boxes.append((center, size, yaw))
        if not self.surfaces and not self.boxes:
            raise ValueError('scene needs at least one surface or box')

        azimuth = np.linspace(-np.pi, np.pi, self.azimuth_samples, endpoint=False)
        elevation = np.deg2rad(self.vertical_angles_deg)[:, None]
        horizontal = np.cos(elevation)
        self.directions_sensor = np.stack(np.broadcast_arrays(
            horizontal * np.cos(azimuth),
            horizontal * np.sin(azimuth),
            np.sin(elevation)), axis=-1).reshape(-1, 3)
        self.origins_sensor = np.zeros_like(self.directions_sensor)
        self.origins_sensor[:, 2] = np.repeat(
            self.vertical_offsets_m, self.azimuth_samples)
        self.rings = np.repeat(
            np.arange(16, dtype=np.uint16), self.azimuth_samples)

    @classmethod
    def from_file(cls, filename):
        with Path(filename).open(encoding='utf-8') as stream:
            return cls(yaml.safe_load(stream))

    def pose_at(self, seconds):
        """Integrate commanded planar motion; stop after the last segment."""
        if not np.isfinite(seconds) or seconds < 0:
            raise ValueError('time must be finite and nonnegative')
        pose = self.initial_pose.copy()
        remaining = float(seconds)
        segments = self.motion if self.motion else [
            (remaining, self.speed_mps, self.yaw_rate_deg_s)]
        for duration, speed, turn_deg in segments:
            dt = min(remaining, duration)
            if dt <= 0:
                break
            heading = np.deg2rad(pose[3])
            turn = np.deg2rad(turn_deg)
            if abs(turn) < 1e-12:
                pose[0] += speed * dt * np.cos(heading)
                pose[1] += speed * dt * np.sin(heading)
            else:
                end_heading = heading + turn * dt
                pose[0] += speed / turn * (
                    np.sin(end_heading) - np.sin(heading))
                pose[1] += speed / turn * (
                    np.cos(heading) - np.cos(end_heading))
            pose[3] += turn_deg * dt
            remaining -= dt
        return pose

    def visible_surface_patches(self):
        """Return floor rectangles with cutouts removed for RViz geometry."""
        result = []
        for bounds, height, holes in self.surfaces:
            patches = [bounds]
            for hole in holes:
                remainder = []
                for rect in patches:
                    x0 = max(rect[0], hole[0])
                    x1 = min(rect[1], hole[1])
                    y0 = max(rect[2], hole[2])
                    y1 = min(rect[3], hole[3])
                    if x0 >= x1 or y0 >= y1:
                        remainder.append(rect)
                        continue
                    for piece in ([rect[0], x0, rect[2], rect[3]],
                                  [x1, rect[1], rect[2], rect[3]],
                                  [x0, x1, rect[2], y0],
                                  [x0, x1, y1, rect[3]]):
                        if piece[0] < piece[1] and piece[2] < piece[3]:
                            remainder.append(np.asarray(piece))
                patches = remainder
            result.extend((patch, height) for patch in patches)
        return result

    def cast(self, pose_xyzyaw_deg=None, rng=None):
        pose = self.initial_pose if pose_xyzyaw_deg is None else _numbers(
            pose_xyzyaw_deg, 4, 'pose_xyzyaw_deg')
        base_rotation = _rotation([0, 0, pose[3]])
        sensor_rotation = base_rotation @ self.mount_rotation
        directions_world = self.directions_sensor @ sensor_rotation.T
        origins_world = (self.origins_sensor @ sensor_rotation.T +
                         base_rotation @ self.sensor_xyz + pose[:3])
        distances = np.full(len(directions_world), np.inf)

        for bounds, height, holes in self.surfaces:
            with np.errstate(divide='ignore', invalid='ignore'):
                length = (height - origins_world[:, 2]) / directions_world[:, 2]
            x = origins_world[:, 0] + length * directions_world[:, 0]
            y = origins_world[:, 1] + length * directions_world[:, 1]
            hit = ((length > 0) & np.isfinite(length) &
                   (x >= bounds[0]) & (x <= bounds[1]) &
                   (y >= bounds[2]) & (y <= bounds[3]))
            for hole in holes:
                hit &= ~((x >= hole[0]) & (x <= hole[1]) &
                         (y >= hole[2]) & (y <= hole[3]))
            distances = np.minimum(distances, np.where(hit, length, np.inf))

        for center, size, yaw_deg in self.boxes:
            box_rotation = _rotation([0, 0, -yaw_deg])
            local_origins = (origins_world - center) @ box_rotation.T
            local_directions = directions_world @ box_rotation.T
            lower = -size / 2
            upper = size / 2
            with np.errstate(divide='ignore', invalid='ignore'):
                t1 = (lower - local_origins) / local_directions
                t2 = (upper - local_origins) / local_directions
            inside_parallel = ((local_origins >= lower) &
                               (local_origins <= upper))
            parallel = np.abs(local_directions) < 1e-12
            entry = np.where(parallel, -np.inf, np.minimum(t1, t2))
            leave = np.where(parallel, np.inf, np.maximum(t1, t2))
            near = np.max(entry, axis=1)
            far = np.min(leave, axis=1)
            hit = (np.all(~parallel | inside_parallel, axis=1) &
                   (far >= np.maximum(near, 0)))
            length = np.where(near > 0, near, far)
            distances = np.minimum(distances, np.where(hit, length, np.inf))

        visible = ((distances >= self.min_range_m) &
                   (distances <= self.max_range_m) & np.isfinite(distances))
        if rng is not None and self.dropout_probability:
            visible &= rng.random(len(visible)) >= self.dropout_probability
        lengths = distances[visible]
        if rng is not None and self.noise_std_m:
            lengths = lengths + rng.normal(0, self.noise_std_m, len(lengths))
        visible_indices = np.flatnonzero(visible)
        in_range = (lengths >= self.min_range_m) & (lengths <= self.max_range_m)
        visible_indices = visible_indices[in_range]
        lengths = lengths[in_range]
        sensor_points = (self.origins_sensor[visible_indices] +
                         self.directions_sensor[visible_indices] * lengths[:, None])
        world_points = (origins_world[visible_indices] +
                        directions_world[visible_indices] * lengths[:, None])
        return Scan(sensor_points.astype(np.float32),
                    world_points.astype(np.float32),
                    self.rings[visible_indices].copy())
