"""Exercise the operator CLI with a fake ros2 executable, never a ROS graph."""
import os
from pathlib import Path
import subprocess

import pytest


@pytest.mark.parametrize('arguments,expected', [
    ([], 0), (['help'], 0), (['--help'], 0),
    (['rearm'], 2), (['request_navigation'], 2),
    (['arm', 'brake'], 2), (['unknown'], 2),
])
def test_help_and_invalid_actions_never_contact_ros(
        tmp_path: Path, arguments: list[str], expected: int) -> None:
    executable = tmp_path / 'ros2'
    receipt = tmp_path / 'contacted'
    executable.write_text('#!/bin/sh\n: > "$RECEIPT"\nexit 41\n')
    executable.chmod(0o755)
    script = Path(__file__).parents[1] / 'scripts/robot_control'

    result = subprocess.run(
        ['/bin/sh', str(script), *arguments], check=False, capture_output=True,
        env={**os.environ, 'PATH': str(tmp_path), 'RECEIPT': str(receipt)})

    assert result.returncode == expected
    assert not receipt.exists()


@pytest.mark.parametrize('action', ['status', 'motors', 'arm', 'brake', 'manual', 'free'])
def test_explicit_action_preserves_ros_cli_failure(tmp_path: Path, action: str) -> None:
    executable = tmp_path / 'ros2'
    executable.write_text('#!/bin/sh\nexit 41\n')
    executable.chmod(0o755)
    script = Path(__file__).parents[1] / 'scripts/robot_control'

    result = subprocess.run(
        ['/bin/sh', str(script), action], check=False, capture_output=True,
        env={**os.environ, 'PATH': str(tmp_path)})

    assert result.returncode == 41
