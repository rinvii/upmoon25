"""
Tests for camera pan 0–180° clamping and arduino_driver constant alignment.

Runs without ROS: `pytest tests/` with PYTHONPATH=lunar/src (see root pyproject.toml).
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[1]
_ARDUINO_DRIVER = _REPO_ROOT / "src/frontend/frontend/arduino_driver.py"


def test_clamp_pan_degrees_bounds() -> None:
    from lunar.camera_pan_limits import PAN_DEG_MAX, PAN_DEG_MIN, clamp_pan_degrees

    assert PAN_DEG_MIN == 0
    assert PAN_DEG_MAX == 180
    assert clamp_pan_degrees(-100) == 0
    assert clamp_pan_degrees(0) == 0
    assert clamp_pan_degrees(90) == 90
    assert clamp_pan_degrees(180) == 180
    assert clamp_pan_degrees(999) == 180


def test_keyboard_tui_matches_shared_clamp() -> None:
    """RobotActuators.set_pan must use the same clamp as ``camera_pan_limits``."""
    from lunar.camera_pan_limits import clamp_pan_degrees
    from lunar.keyboard_tui import RobotActuators

    ra = RobotActuators.__new__(RobotActuators)
    ra.serial = None

    published: list[int] = []

    def _capture_publish(_name: str, value: int) -> bool:
        published.append(int(value))
        return True

    setattr(ra, "_publish_int", _capture_publish)

    assert ra.set_pan(500) is True
    assert published[-1] == clamp_pan_degrees(500)

    assert ra.set_pan(-1) is True
    assert published[-1] == clamp_pan_degrees(-1)


def test_arduino_driver_pan_constants_ast() -> None:
    """Avoid importing arduino_driver ( pulls rclpy ); verify source still declares 0 / 180."""

    tree = ast.parse(_ARDUINO_DRIVER.read_text(encoding="utf-8"))
    values: dict[str, int] = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in ("PAN_MIN", "PAN_MAX"):
                    values[target.id] = ast.literal_eval(node.value)
    assert values.get("PAN_MIN") == 0
    assert values.get("PAN_MAX") == 180


def _have_ros_and_cv_for_dashboard() -> bool:
    import importlib.util

    try:
        for name in ("rclpy", "cv2"):
            if importlib.util.find_spec(name) is None:
                return False
    except Exception:
        return False
    return True


@pytest.mark.skipif(
    not _have_ros_and_cv_for_dashboard(),
    reason="rclpy / cv2 not installed (optional dashboard integration test)",
)
def test_dashboard_publish_pan_uses_shared_clamp() -> None:
    from unittest.mock import MagicMock, patch

    pytest.importorskip("std_msgs")

    with patch("lunar.dashboard.bridge.store"):
        from lunar.camera_pan_limits import clamp_pan_degrees
        from lunar.dashboard.bridge import DashboardBridge

        bridge = DashboardBridge.__new__(DashboardBridge)
        bridge.pub_pan = MagicMock()

        bridge.publish_pan(500)
        assert bridge.pub_pan.publish.call_args[0][0].data == clamp_pan_degrees(500)

        bridge.publish_pan_jog(1)
        assert bridge.pub_pan.publish.call_args[0][0].data == 1