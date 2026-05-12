"""Camera pan angle bounds (degrees), shared by lunar tools.

Keep in sync with ``frontend.arduino_driver`` PAN_MIN / PAN_MAX and firmware 0–180°
``constrain`` on the Arduino pan servo command.
"""

PAN_DEG_MIN = 0
PAN_DEG_MAX = 180


def clamp_pan_degrees(value: int) -> int:
    """Clamp a commanded pan angle to the allowed degree range."""
    return max(PAN_DEG_MIN, min(PAN_DEG_MAX, int(value)))
