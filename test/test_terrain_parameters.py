"""Ensure the shared detector config applies to real and renamed simulation nodes."""

from pathlib import Path

import pytest
import rclpy
from rclpy.context import Context


@pytest.mark.parametrize('name', ['terrain_step_detector', 'sim_terrain_step_detector'])
def test_detector_parameters_survive_node_rename(name):
    config = Path(__file__).resolve().parents[1] / 'config' / 'terrain.yaml'
    context = Context()
    rclpy.init(
        args=['--ros-args', '--params-file', str(config)],
        context=context, domain_id=132)
    node = None
    try:
        node = rclpy.create_node(name, context=context)
        # Deliberately different defaults detect a YAML node selector mismatch.
        assert node.declare_parameter('step_height', 0.99).value == 0.03
        assert node.declare_parameter('history_seconds', 0.1).value == 2.0
        assert node.declare_parameter('sensor_x', -1.0).value == -0.347981
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()
