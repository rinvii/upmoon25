"""Canonical ROS topic names for lunar keyboard teleop (publish + subscribe sides).

Used by the Textual keyboard TUI, legacy raw subsystem keyboard, and offline contract tests.

Absolute pan angles published on `/cmd/pan` use degrees in ``[PAN_ANGLE_MIN, PAN_ANGLE_MAX]``.
Keep these bounds aligned with ``PAN_MIN`` / ``PAN_MAX`` in ``arduino_driver.py``.
"""

from __future__ import annotations

PAN_ANGLE_MIN = 0
PAN_ANGLE_MAX = 180
BUCKET_POS_MIN = 0
BUCKET_POS_MAX = 40


def clamp_pan_angle(value: int | float) -> int:
    """Clamp an intended absolute pan angle (degrees) for `/cmd/pan` when sending setpoints."""
    return max(PAN_ANGLE_MIN, min(PAN_ANGLE_MAX, int(value)))


# Outbound commands (geometry_msgs/Twist on drive; std_msgs/Int16 on others).
KEYBOARD_PUBLISHER_TOPICS: dict[str, str] = {
    "drive": "cmd/velocity",
    "camera-height": "/cmd/camera_height",
    "pan": "/cmd/pan",
    "bucket-pos": "/cmd/bucket_pos",
    "bucket-vel": "/cmd/bucket_vel",
    "conveyor": "/cmd/conveyor",
}

# Inbound telemetry consumed by the keyboard TUI for dashboard/IR/encoder display.
KEYBOARD_SENSOR_TOPICS: dict[str, str] = {
    "ir_general": "/sensor/ir",
    "ir_right": "/sensor/ir/right",
    "ir_left": "/sensor/ir/left",
    "encoder_left": "/sensor/encoder/left",
    "encoder_right": "/sensor/encoder/right",
    "encoder_telemetry": "/sensor/encoder/telemetry",
}
