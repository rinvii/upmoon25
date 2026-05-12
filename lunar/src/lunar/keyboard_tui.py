from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

from lunar.camera_pan_limits import clamp_pan_degrees

from .keyboard_topics import KEYBOARD_PUBLISHER_TOPICS, KEYBOARD_SENSOR_TOPICS

ARDUINO_CAM_HEIGHT_PIN = 9
ARDUINO_PAN_PIN = 3


def _clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


def _clamp_float(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _detect_arduino_port() -> Optional[Path]:
    import serial.tools.list_ports

    try:
        for port in serial.tools.list_ports.comports():
            vid = None if port.vid is None else f"{port.vid:04x}"
            pid = None if port.pid is None else f"{port.pid:04x}"
            if vid == "2341" and pid == "0043":
                return Path(port.device)
    except Exception:
        pass

    for fallback in ["/dev/ttyACM2", "/dev/ttyACM1", "/dev/ttyACM0", "/dev/ttyUSB0"]:
        path = Path(fallback)
        if path.exists():
            return path
    return None


class RobotActuators:
    def __init__(self, *, direct_serial: bool = False):
        self.direct_serial = direct_serial
        self.mode = "starting"
        self.status = "initializing"
        self.serial_port = _detect_arduino_port()
        self.serial = None
        self.node = None
        self.created_context = False
        self.publishers = {}
        self.telemetry = {
            "ir_left": 0,
            "ir_right": 0,
            "enc_left": 0,
            "enc_right": 0,
        }
        self.control_path = "unknown"

    def open(self) -> None:
        self._open_ros()
        if self.direct_serial:
            self._open_serial()

    def _open_ros(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import Twist
            from std_msgs.msg import Int16, Int32
        except Exception as exc:
            self.mode = "offline"
            self.status = f"ROS unavailable: {exc}"
            return

        if not rclpy.ok():
            rclpy.init()
            self.created_context = True

        self.node = Node("lunar_keyboard_tui")
        kt = KEYBOARD_PUBLISHER_TOPICS
        self.publishers = {
            "drive": self.node.create_publisher(Twist, kt["drive"], 10),
            "camera-height": self.node.create_publisher(Int16, kt["camera-height"], 10),
            "pan": self.node.create_publisher(Int16, kt["pan"], 10),
            "bucket-pos": self.node.create_publisher(Int16, kt["bucket-pos"], 10),
            "bucket-vel": self.node.create_publisher(Int16, kt["bucket-vel"], 10),
            "conveyor": self.node.create_publisher(Int16, kt["conveyor"], 10),
        }
        ks = KEYBOARD_SENSOR_TOPICS
        self.node.create_subscription(Int16, ks["ir_general"], self._on_ir, 10)
        self.node.create_subscription(Int16, ks["ir_right"], self._on_ir_right, 10)
        self.node.create_subscription(Int16, ks["ir_left"], self._on_ir_left, 10)
        self.node.create_subscription(Int32, ks["encoder_left"], self._on_enc_left, 10)
        self.node.create_subscription(Int32, ks["encoder_right"], self._on_enc_right, 10)
        self.mode = "ros-topic"
        self.control_path = "ROS topics -> robot stack"
        self.status = "publishing robot command topics"

    def _open_serial(self) -> None:
        if self.serial_port is None:
            self.status = "ROS ready; Arduino serial not detected"
            return

        try:
            import serial

            self.serial = serial.Serial(str(self.serial_port), 115200, timeout=0, write_timeout=0.05)
            # Uno-class boards reset on port open. Direct serial is best used when
            # arduino_driver is not running.
            time.sleep(2.0)
            self.serial.reset_input_buffer()
            self.mode = "ros-topic + direct serial"
            self.control_path = "direct Arduino serial for enabled servo"
            self.status = f"ROS ready; Arduino on {self.serial_port}"
        except Exception as exc:
            self.serial = None
            self.status = f"ROS ready; serial unavailable: {exc}"

    def _spin_once(self) -> None:
        if self.node is None:
            return
        try:
            import rclpy

            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception:
            pass

    def poll(self) -> None:
        self._spin_once()

    def _on_ir(self, msg) -> None:
        # Backwards compatible single IR topic; mirror to right channel.
        self.telemetry["ir_right"] = int(msg.data)

    def _on_ir_right(self, msg) -> None:
        self.telemetry["ir_right"] = int(msg.data)

    def _on_ir_left(self, msg) -> None:
        self.telemetry["ir_left"] = int(msg.data)

    def _on_enc_left(self, msg) -> None:
        self.telemetry["enc_left"] = int(msg.data)

    def _on_enc_right(self, msg) -> None:
        self.telemetry["enc_right"] = int(msg.data)

    def _publish_int(self, name: str, value: int) -> bool:
        if self.node is None or name not in self.publishers:
            return False

        try:
            from std_msgs.msg import Int16

            msg = Int16()
            msg.data = int(value)
            self.publishers[name].publish(msg)
            self._spin_once()
            self.status = f"published {name}={value}"
            return True
        except Exception as exc:
            self.status = f"{name} publish failed: {exc}"
            return False

    def set_drive(self, linear: float, angular: float) -> bool:
        if self.node is None or "drive" not in self.publishers:
            return False

        try:
            from geometry_msgs.msg import Twist

            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.publishers["drive"].publish(msg)
            self._spin_once()
            self.status = f"published drive linear={linear:.1f} angular={angular:.1f}"
            return True
        except Exception as exc:
            self.status = f"drive publish failed: {exc}"
            return False

    def set_camera_height(self, value: int) -> bool:
        value = _clamp(value, 0, 100)
        if self.serial is not None:
            try:
                self.serial.write(f"{ARDUINO_CAM_HEIGHT_PIN}:{value}\n".encode("utf-8"))
                self.serial.flush()
                self.status = f"sent camera-height={value} to Arduino pin {ARDUINO_CAM_HEIGHT_PIN}"
                return True
            except Exception as exc:
                self.status = f"serial write failed: {exc}"
                return False
        return self._publish_int("camera-height", value)

    def set_pan(self, value: int) -> bool:
        value = clamp_pan_degrees(value)
        if self.serial is not None:
            try:
                self.serial.write(f"{ARDUINO_PAN_PIN}:{value}\n".encode("utf-8"))
                self.serial.flush()
                self.status = f"sent pan={value} to Arduino pin {ARDUINO_PAN_PIN}"
                return True
            except Exception as exc:
                self.status = f"serial write failed: {exc}"
                return False
        return self._publish_int("pan", int(value))

    def stop_pan(self) -> bool:
        return self._publish_int("pan", 0)

    def set_bucket_pos(self, value: int) -> bool:
        return self._publish_int("bucket-pos", _clamp(value, 0, 100))

    def set_bucket_vel(self, value: int) -> bool:
        return self._publish_int("bucket-vel", int(value))

    def set_conveyor(self, value: int) -> bool:
        return self._publish_int("conveyor", 1 if int(value) else 0)

    def stop_all(self) -> None:
        self.set_drive(0.0, 0.0)
        self.set_bucket_vel(0)
        self.set_conveyor(0)

    def close(self) -> None:
        self.stop_all()

        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None

        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            self.node = None

        if self.created_context:
            try:
                import rclpy

                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
            self.created_context = False


def run_keyboard_tui(
    *,
    step: int = 5,
    ros_only: bool = True,
    subsystems: Optional[set[str]] = None,
    drive_speed: float = 35.0,
    turn_speed: float = 35.0,
    bucket_speed: int = 40,
    drive_timeout: float = 0.35,
    bucket_timeout: float = 0.35,
    publish_hz: float = 30.0,
    drive_mode: str = "latch",
) -> None:
    try:
        from textual import work
        from textual.app import App, ComposeResult
        from textual.binding import Binding
        from textual.containers import Vertical
        from textual.reactive import reactive
        from textual.widgets import Header, ProgressBar, Static
    except Exception as exc:
        raise RuntimeError(
            "Textual is not installed in this lunar environment. "
            "Install the pinned dependency with `uv sync` or use `lunar keyboard --raw`."
        ) from exc

    enabled = subsystems or {"camera-height"}
    normalized_drive_mode = drive_mode.strip().lower()
    if normalized_drive_mode not in {"latch", "hold"}:
        raise RuntimeError("--drive-mode must be either 'latch' or 'hold'")

    class LunarKeyboardTui(App):
        ENABLE_COMMAND_PALETTE = False

        CSS = """
        Screen {
            background: #101418;
            color: #f4f7fb;
        }

        #shell {
            height: 100%;
            width: 100%;
            padding: 0 1;
        }

        #title {
            text-style: bold;
            color: #8bd3ff;
            height: 1;
            margin-bottom: 0;
        }

        .section {
            color: #8bd3ff;
            color: #a7f3d0;
            text-style: bold;
            height: 1;
        }

        .line {
            height: 1;
            color: #f4f7fb;
        }

        .muted {
            color: #93a4b7;
        }

        .hot {
            color: #a7f3d0;
            text-style: bold;
        }

        #warn {
            color: #ffd166;
            height: 1;
        }
        """

        BINDINGS = [
            Binding("space", "stop", "Stop", show=False),
            Binding("q", "quit", "Quit", show=False),
        ]

        camera_height = reactive(0)
        pan = reactive(90)
        bucket_pos = reactive(0)
        bucket_vel = reactive(0)
        conveyor = reactive(0)
        drive_linear = reactive(0.0)
        drive_angular = reactive(0.0)
        mode = reactive("starting")
        connection = reactive("initializing")
        control_path = reactive("starting")
        last_result = reactive("waiting for input")
        uptime = reactive("0.0s")

        def __init__(self, *, actuator: RobotActuators):
            super().__init__()
            self.actuator = actuator
            self.start_ts = time.monotonic()
            self.height_step = _clamp(step, 1, 25)
            self.bucket_pos_step = 1
            self.last_drive_ts = 0.0
            self.last_bucket_ts = 0.0
            self.drive_timeout = max(0.12, float(drive_timeout))
            self.bucket_timeout = max(0.12, float(bucket_timeout))
            self.drive_publish_period = 1.0 / max(5.0, min(60.0, float(publish_hz)))
            self.last_drive_publish_ts = 0.0
            self.drive_stop_sent = True
            self.drive_mode = normalized_drive_mode
            self.drive_speed = _clamp_float(drive_speed, 0.0, 100.0)
            self.turn_speed = _clamp_float(turn_speed, 0.0, 100.0)
            self.bucket_speed = _clamp(bucket_speed, 0, 100)
            self.drive_speed_step = 5.0
            self.bucket_speed_step = 5
            self.active_drive_key = None
            self.active_bucket_key = None
            self.last_height_ts = 0.0
            self.last_pan_ts = 0.0
            self.position_refresh_until = 0.0
            self.position_refresh_period = 0.08
            self.last_position_refresh_ts = 0.0
            self.key_count = 0
            self.last_key = "-"
            self.last_key_ts = 0.0

        def compose(self) -> ComposeResult:
            yield Header(show_clock=True)
            with Vertical(id="shell"):
                yield Static("", id="top", classes="line hot")
                yield Static("", id="path", classes="line")
                yield Static("", id="warn", classes="line")
                yield Static("COMMANDS", classes="section")
                yield Static("DRIVE: W fwd | S rev | A left | D right | SPACE stop | Q quit", classes="line")
                yield Static("SERVOS: U/J cam height | 0/1 min/max | H/L pan angle | M center", classes="line")
                yield Static("MINING: I/K bucket pos | R/F chain | C conveyor", classes="line")
                yield Static("ADJUST: [ ] W/S speed | , . A/D turn | - / R/F chain", classes="line")
                yield Static("VALUES", classes="section")
                yield Static("", id="drive_value", classes="line")
                yield Static("", id="slider_drive", classes="line")
                yield Static("", id="slider_turn", classes="line")
                yield Static("", id="slider_chain", classes="line")
                yield Static("", id="servo_value", classes="line")
                yield Static("", id="mining_value", classes="line")
                yield Static("", id="sensor_value", classes="line")
                yield Static("", id="timing_value", classes="line")
                yield ProgressBar(total=100, show_eta=False, id="height_bar")
                yield Static("LAST", classes="section")
                yield Static("", id="log", classes="line")

        def on_mount(self) -> None:
            self._refresh_view()
            self.set_interval(0.1, self._clock_tick)
            self.set_interval(0.05, self._watchdog)
            self.set_interval(self.drive_publish_period, self._drive_tick)
            self.set_interval(0.1, self._bucket_tick)
            self.set_interval(self.position_refresh_period, self._position_refresh_tick)
            self.connect_actuator()

        @work(thread=True)
        def connect_actuator(self) -> None:
            self.actuator.open()
            self.call_from_thread(self._apply_actuator_state)

        def _apply_actuator_state(self) -> None:
            self.mode = self.actuator.mode
            self.connection = self.actuator.status
            self.control_path = self.actuator.control_path
            self._refresh_view()

        def _clock_tick(self) -> None:
            self.actuator.poll()
            self.uptime = f"{time.monotonic() - self.start_ts:.1f}s"
            self._refresh_view()

        def _refresh_view(self) -> None:
            if not self.is_mounted:
                return
            self.query_one("#top", Static).update(
                f"LUNAR KEYBOARD | mode={self.mode} | enabled={','.join(sorted(enabled))}"
            )
            self.query_one("#path", Static).update(
                f"path={self.control_path} | connection={self.connection}"
            )
            warn = ""
            if enabled != {"camera-height"} and self.control_path.startswith("direct"):
                warn = "WARNING: full robot control should use lunar run robot + --ros-only"
            elif enabled != {"camera-height"} and not self.control_path.startswith("ROS"):
                warn = "WAITING: robot stack/ROS path not ready"
            self.query_one("#warn", Static).update(warn)
            self.query_one("#drive_value", Static).update(
                f"drive      linear={self.drive_linear:>6.1f}  angular={self.drive_angular:>6.1f}  "
                f"mode={self.drive_mode}  active={self.active_drive_key or '-'}  "
                f"timeout={self.drive_timeout:.2f}s  hz={1.0 / self.drive_publish_period:.0f}"
            )
            self.query_one("#slider_drive", Static).update(
                f"W/S speed  {self._slider(self.drive_speed, 0, 100)} {self.drive_speed:>5.1f}"
            )
            self.query_one("#slider_turn", Static).update(
                f"A/D turn   {self._slider(self.turn_speed, 0, 100)} {self.turn_speed:>5.1f}"
            )
            self.query_one("#slider_chain", Static).update(
                f"R/F chain  {self._slider(self.bucket_speed, 0, 100)} {self.bucket_speed:>3}"
            )
            self.query_one("#height_bar", ProgressBar).update(progress=self.camera_height)
            self.query_one("#servo_value", Static).update(
                f"servos     cam_height={self.camera_height:>3}%  pan={self.pan:>3}  step={self.height_step}"
            )
            self.query_one("#mining_value", Static).update(
                f"mining     bucket_pos={self.bucket_pos:>3}%  chain={self.bucket_vel:>4}  "
                f"active_chain={self.active_bucket_key or '-'}  conveyor={'ON ' if self.conveyor else 'OFF'}"
            )
            telemetry = self.actuator.telemetry
            self.query_one("#sensor_value", Static).update(
                f"sensors    ir_left={telemetry['ir_left']:>4}  ir_right={telemetry['ir_right']:>4}  "
                f"enc_left={telemetry['enc_left']:>7}  enc_right={telemetry['enc_right']:>7}"
            )
            key_age = time.monotonic() - self.last_key_ts if self.last_key_ts else 0.0
            self.query_one("#timing_value", Static).update(
                f"input      last_key={self.last_key:<6} age={key_age:>4.2f}s  "
                f"count={self.key_count:<5} uptime={self.uptime}"
            )
            self.query_one("#log", Static).update(f"{self.last_result} | status={self.actuator.status}")

        def _slider(self, value: float, low: float, high: float, width: int = 18) -> str:
            span = max(1.0, float(high) - float(low))
            ratio = _clamp_float((float(value) - float(low)) / span, 0.0, 1.0)
            filled = int(round(ratio * width))
            return "[" + ("#" * filled) + ("-" * (width - filled)) + "]"

        def _set_result(self, ok: bool) -> None:
            self.mode = self.actuator.mode
            self.connection = self.actuator.status
            self.control_path = self.actuator.control_path
            self.last_result = "sent" if ok else "not sent"
            self._refresh_view()

        def _stop_drive(self, reason: str = "drive stop") -> None:
            self.drive_linear = 0.0
            self.drive_angular = 0.0
            self.active_drive_key = None
            self.drive_stop_sent = True
            self.actuator.set_drive(0.0, 0.0)
            self.last_result = reason
            self._refresh_view()

        def _set_drive(self, linear: float, angular: float, key: str) -> None:
            if "drive" not in enabled:
                self.last_result = "drive disabled"
                self._refresh_view()
                return
            if self.drive_mode == "latch" and self.active_drive_key == key:
                self._stop_drive(f"drive toggle stop ({key})")
                return
            self.drive_linear = float(linear)
            self.drive_angular = float(angular)
            self.active_drive_key = key
            self.last_drive_ts = time.monotonic()
            self.drive_stop_sent = False
            self._set_result(self.actuator.set_drive(self.drive_linear, self.drive_angular))

        def _set_height(self, value: int) -> None:
            if "camera-height" not in enabled:
                self.last_result = "camera-height disabled"
                self._refresh_view()
                return
            self.camera_height = _clamp(value, 0, 100)
            self.last_height_ts = time.monotonic()
            self.position_refresh_until = max(self.position_refresh_until, self.last_height_ts + 0.45)
            self._set_result(self.actuator.set_camera_height(self.camera_height))
            self.last_result = f"camera height -> {self.camera_height}"
            self._refresh_view()

        def _set_pan(self, value: int) -> None:
            if "pan" not in enabled:
                self.last_result = "pan disabled"
                self._refresh_view()
                return
            self.pan = clamp_pan_degrees(value)
            self.last_pan_ts = time.monotonic()
            self.position_refresh_until = max(self.position_refresh_until, self.last_pan_ts + 0.75)
            self._set_result(self.actuator.set_pan(self.pan))
            self.last_result = f"pan -> {self.pan}"
            self._refresh_view()

        def _set_bucket_pos(self, value: int) -> None:
            if "bucket-pos" not in enabled:
                self.last_result = "bucket position disabled"
                self._refresh_view()
                return
            self.bucket_pos = _clamp(value, 0, 100)
            self._set_result(self.actuator.set_bucket_pos(self.bucket_pos))

        def _set_bucket_vel(self, value: int) -> None:
            if "bucket-vel" not in enabled:
                self.last_result = "bucket chain disabled"
                self._refresh_view()
                return
            self.bucket_vel = int(value)
            self.last_bucket_ts = time.monotonic()
            self._set_result(self.actuator.set_bucket_vel(self.bucket_vel))

        def _current_drive_command(self) -> tuple[float, float]:
            if self.active_drive_key == "w":
                return self.drive_speed, 0.0
            if self.active_drive_key == "s":
                return -self.drive_speed, 0.0
            if self.active_drive_key == "a":
                return 0.0, self.turn_speed
            if self.active_drive_key == "d":
                return 0.0, -self.turn_speed
            return 0.0, 0.0

        def _current_bucket_command(self) -> int:
            if self.active_bucket_key == "r":
                return self.bucket_speed
            if self.active_bucket_key == "f":
                return -self.bucket_speed
            return 0

        def _refresh_active_latches(self) -> None:
            if self.active_drive_key is not None:
                self.drive_linear, self.drive_angular = self._current_drive_command()
                self.last_drive_ts = time.monotonic()
                self.drive_stop_sent = False
                self.actuator.set_drive(self.drive_linear, self.drive_angular)
            if self.active_bucket_key is not None:
                self.bucket_vel = self._current_bucket_command()
                self.last_bucket_ts = time.monotonic()
                self.actuator.set_bucket_vel(self.bucket_vel)

        def _adjust_drive_speed(self, delta: float) -> None:
            self.drive_speed = _clamp_float(self.drive_speed + delta, 0.0, 100.0)
            if self.active_drive_key in {"w", "s"}:
                self._refresh_active_latches()
            self.last_result = f"W/S speed -> {self.drive_speed:.1f}"
            self._refresh_view()

        def _adjust_turn_speed(self, delta: float) -> None:
            self.turn_speed = _clamp_float(self.turn_speed + delta, 0.0, 100.0)
            if self.active_drive_key in {"a", "d"}:
                self._refresh_active_latches()
            self.last_result = f"A/D turn -> {self.turn_speed:.1f}"
            self._refresh_view()

        def _adjust_bucket_speed(self, delta: int) -> None:
            self.bucket_speed = _clamp(self.bucket_speed + delta, 0, 100)
            if self.active_bucket_key is not None:
                self._refresh_active_latches()
            self.last_result = f"R/F chain speed -> {self.bucket_speed}"
            self._refresh_view()

        def _set_bucket_vel_latch(self, value: int, key: str) -> None:
            if "bucket-vel" not in enabled:
                self.last_result = "bucket chain disabled"
                self._refresh_view()
                return
            if self.active_bucket_key == key:
                self.bucket_vel = 0
                self.active_bucket_key = None
                self._set_result(self.actuator.set_bucket_vel(0))
                self.last_result = f"bucket chain toggle stop ({key})"
                self._refresh_view()
                return
            self.bucket_vel = int(value)
            self.active_bucket_key = key
            self.last_bucket_ts = time.monotonic()
            self._set_result(self.actuator.set_bucket_vel(self.bucket_vel))

        def _set_conveyor(self, value: int) -> None:
            if "conveyor" not in enabled:
                self.last_result = "conveyor disabled"
                self._refresh_view()
                return
            self.conveyor = 1 if int(value) else 0
            self._set_result(self.actuator.set_conveyor(self.conveyor))

        def _watchdog(self) -> None:
            now = time.monotonic()
            if self.drive_mode == "hold" and "drive" in enabled and (self.drive_linear or self.drive_angular):
                if now - self.last_drive_ts > self.drive_timeout:
                    self.drive_linear = 0.0
                    self.drive_angular = 0.0
                    self.active_drive_key = None
                    self.actuator.set_drive(0.0, 0.0)
                    self.drive_stop_sent = True
                    self.last_result = "drive watchdog stop"
                    self._refresh_view()

            if "bucket-vel" in enabled and self.bucket_vel:
                if now - self.last_bucket_ts > self.bucket_timeout:
                    # Bucket chain is latch-style; keep publishing until explicitly stopped.
                    self.last_bucket_ts = now

        def _drive_tick(self) -> None:
            if "drive" not in enabled:
                return

            now = time.monotonic()
            active = bool(self.drive_linear or self.drive_angular)
            if active and (self.drive_mode == "latch" or now - self.last_drive_ts <= self.drive_timeout):
                if now - self.last_drive_publish_ts >= self.drive_publish_period:
                    self.actuator.set_drive(self.drive_linear, self.drive_angular)
                    self.last_drive_publish_ts = now
                return

            if self.drive_mode == "hold" and active:
                self.drive_linear = 0.0
                self.drive_angular = 0.0
                self.active_drive_key = None
                self.drive_stop_sent = False

            if not self.drive_stop_sent:
                self.actuator.set_drive(0.0, 0.0)
                self.drive_stop_sent = True
                self.last_result = "drive stop"
                self._refresh_view()

        def _bucket_tick(self) -> None:
            if "bucket-vel" not in enabled or not self.bucket_vel:
                return
            self.actuator.set_bucket_vel(self.bucket_vel)

        def _position_refresh_tick(self) -> None:
            now = time.monotonic()
            if now > self.position_refresh_until:
                return
            if now - self.last_position_refresh_ts < self.position_refresh_period:
                return
            self.last_position_refresh_ts = now

            sent = False
            if "camera-height" in enabled and self.last_height_ts > 0:
                self.actuator.set_camera_height(self.camera_height)
                sent = True
            if "pan" in enabled and self.last_pan_ts > 0:
                self.actuator.set_pan(self.pan)
                sent = True
            if sent:
                self.mode = self.actuator.mode
                self.connection = self.actuator.status
                self.control_path = self.actuator.control_path

        def on_key(self, event) -> None:
            key = event.key
            self.key_count += 1
            self.last_key = key
            self.last_key_ts = time.monotonic()
            if hasattr(event, "prevent_default"):
                event.prevent_default()
            if hasattr(event, "stop"):
                event.stop()

            handlers = {
                "w": self.action_drive_forward,
                "s": self.action_drive_reverse,
                "a": self.action_drive_left,
                "d": self.action_drive_right,
                "u": self.action_height_up,
                "up": self.action_height_up,
                "j": self.action_height_down,
                "down": self.action_height_down,
                "0": self.action_height_min,
                "1": self.action_height_max,
                "h": self.action_pan_left,
                "left": self.action_pan_left,
                "l": self.action_pan_right,
                "right": self.action_pan_right,
                "m": self.action_pan_center,
                "i": self.action_bucket_up,
                "k": self.action_bucket_down,
                "r": self.action_bucket_chain_forward,
                "f": self.action_bucket_chain_reverse,
                "c": self.action_conveyor_toggle,
                "[": self.action_drive_speed_down,
                "left_square_bracket": self.action_drive_speed_down,
                "]": self.action_drive_speed_up,
                "right_square_bracket": self.action_drive_speed_up,
                ",": self.action_turn_speed_down,
                "comma": self.action_turn_speed_down,
                ".": self.action_turn_speed_up,
                "period": self.action_turn_speed_up,
                "-": self.action_bucket_speed_down,
                "minus": self.action_bucket_speed_down,
                "/": self.action_bucket_speed_up,
                "slash": self.action_bucket_speed_up,
                "=": self.action_bucket_speed_up,
                "+": self.action_bucket_speed_up,
                "equals": self.action_bucket_speed_up,
                "equal_sign": self.action_bucket_speed_up,
                "plus": self.action_bucket_speed_up,
                "plus_sign": self.action_bucket_speed_up,
                "kp_add": self.action_bucket_speed_up,
                "space": self.action_stop,
                "q": self.action_quit,
            }
            handler = handlers.get(key)
            if handler is not None:
                handler()

        def action_drive_forward(self) -> None:
            self._set_drive(self.drive_speed, 0.0, "w")

        def action_drive_reverse(self) -> None:
            self._set_drive(-self.drive_speed, 0.0, "s")

        def action_drive_left(self) -> None:
            self._set_drive(0.0, self.turn_speed, "a")

        def action_drive_right(self) -> None:
            self._set_drive(0.0, -self.turn_speed, "d")

        def action_height_up(self) -> None:
            self._set_height(self.camera_height + self.height_step)

        def action_height_down(self) -> None:
            self._set_height(self.camera_height - self.height_step)

        def action_height_min(self) -> None:
            self._set_height(0)

        def action_height_max(self) -> None:
            self._set_height(100)

        def action_pan_left(self) -> None:
            self._set_pan(self.pan - self.height_step)

        def action_pan_right(self) -> None:
            self._set_pan(self.pan + self.height_step)

        def action_pan_center(self) -> None:
            self._set_pan(90)

        def action_bucket_up(self) -> None:
            self._set_bucket_pos(self.bucket_pos + self.bucket_pos_step)

        def action_bucket_down(self) -> None:
            self._set_bucket_pos(self.bucket_pos - self.bucket_pos_step)

        def action_bucket_chain_forward(self) -> None:
            self._set_bucket_vel_latch(self.bucket_speed, "r")

        def action_bucket_chain_reverse(self) -> None:
            self._set_bucket_vel_latch(-self.bucket_speed, "f")

        def action_conveyor_toggle(self) -> None:
            self._set_conveyor(0 if self.conveyor else 1)

        def action_drive_speed_down(self) -> None:
            self._adjust_drive_speed(-self.drive_speed_step)

        def action_drive_speed_up(self) -> None:
            self._adjust_drive_speed(self.drive_speed_step)

        def action_turn_speed_down(self) -> None:
            self._adjust_turn_speed(-self.drive_speed_step)

        def action_turn_speed_up(self) -> None:
            self._adjust_turn_speed(self.drive_speed_step)

        def action_bucket_speed_down(self) -> None:
            self._adjust_bucket_speed(-self.bucket_speed_step)

        def action_bucket_speed_up(self) -> None:
            self._adjust_bucket_speed(self.bucket_speed_step)

        def action_stop(self) -> None:
            self.drive_linear = 0.0
            self.drive_angular = 0.0
            self.active_drive_key = None
            self.bucket_vel = 0
            self.active_bucket_key = None
            self.conveyor = 0
            self.actuator.stop_all()
            self.last_result = "stop all sent"
            self._refresh_view()

        def action_quit(self) -> None:
            self.exit()

        def on_unmount(self) -> None:
            self.actuator.close()

    # Direct serial is only useful for standalone servo testing. Full robot
    # operation should go through arduino_driver so one process owns the Arduino.
    actuator = RobotActuators(direct_serial=not ros_only and enabled in ({"camera-height"}, {"pan"}))
    LunarKeyboardTui(actuator=actuator).run()
