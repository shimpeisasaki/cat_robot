"""Offline geometry checks for the synthetic VLP-16 scene generator."""

from pathlib import Path

import numpy as np
import pytest

from cat_terrain.scene import TerrainScene

PACKAGE = Path(__file__).resolve().parents[1]



def scene(name):
    return TerrainScene.from_file(PACKAGE / 'scenes' / name)


def test_flat_ground_has_forward_returns_at_zero_height():
    ground = scene('flat_boxes.yaml')
    ground.boxes = []
    scan = ground.cast()
    x, y, z = scan.world_points.T
    forward = (x > 0.35) & (x < 3.0) & (np.abs(y) < 0.05)
    assert np.count_nonzero(forward) > 20
    assert np.max(np.abs(z)) < 1e-5
    assert len(np.unique(scan.rings)) == 16


def test_box_is_visible_above_ground():
    scan = scene('flat_boxes.yaml').cast()
    x, y, z = scan.world_points.T
    box = (x >= 1.5) & (x <= 2.1) & (np.abs(y) <= 0.6)
    assert np.any(box & (z > 0.02))


def test_lowered_patch_and_open_gap_have_different_returns():
    lower = scene('drop_6cm.yaml').cast().world_points
    in_lower = ((lower[:, 0] > 1.7) & (lower[:, 0] < 3.0) &
                (np.abs(lower[:, 1]) < 0.8))
    assert np.any(in_lower & np.isclose(lower[:, 2], -0.06, atol=1e-5))

    gap = scene('gap_50cm.yaml').cast().world_points
    inside_gap = ((gap[:, 0] > 1.71) & (gap[:, 0] < 2.19) &
                  (np.abs(gap[:, 1]) < 0.99))
    assert not np.any(inside_gap)


def test_scene_geometry_leaves_the_high_floor_open_over_a_drop():
    patches = scene('drop_6cm.yaml').visible_surface_patches()
    high_floor_at_drop = [height for bounds, height in patches if
                          bounds[0] <= 2.0 <= bounds[1] and
                          bounds[2] <= 0.0 <= bounds[3] and height == 0.0]
    lower_floor_at_drop = [height for bounds, height in patches if
                           bounds[0] <= 2.0 <= bounds[1] and
                           bounds[2] <= 0.0 <= bounds[3] and height == -0.06]
    assert not high_floor_at_drop
    assert lower_floor_at_drop


def test_motion_segments_move_cloud_and_stop_at_end():
    moving = scene('drop_6cm.yaml')
    assert moving.pose_at(2.0)[0] == pytest.approx(0.0)
    assert moving.pose_at(4.0)[0] == pytest.approx(0.6)
    assert moving.pose_at(10.0)[0] == pytest.approx(0.9)
    first = moving.cast(moving.pose_at(0.0)).sensor_points
    later = moving.cast(moving.pose_at(4.0)).sensor_points
    assert not np.array_equal(first, later)


def test_turning_follows_an_arc():
    description = {
        'surfaces': [{'bounds_xy_m': [-2, 4, -2, 4], 'height_m': 0}],
        'robot': {'motion': [
            {'duration_s': 2, 'linear_speed_mps': 1},
            {'duration_s': 2, 'linear_speed_mps': 1, 'yaw_rate_deg_s': 45},
        ]},
    }
    moving = TerrainScene(description)
    pose = moving.pose_at(4.0)
    assert pose[0] > 2.5
    assert pose[1] > 1.0
    assert pose[3] == pytest.approx(90.0)
    assert np.array_equal(pose, moving.pose_at(6.0))


def test_invalid_floor_bounds_are_rejected():
    with pytest.raises(ValueError, match='xmin < xmax'):
        TerrainScene({'surfaces': [
            {'bounds_xy_m': [2, 1, -1, 1], 'height_m': 0}
        ]})
