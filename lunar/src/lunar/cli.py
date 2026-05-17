from __future__ import annotations

import json
import os
import select
import signal
import shlex
import shutil
import subprocess
import sys
import termios
import time
import tty
from pathlib import Path
from dataclasses import asdict
from enum import Enum
from typing import Any, Optional, TextIO, cast

import typer
import serial.tools.list_ports

from .config import Config, CONFIG_PATH, find_repo_root
from .keyboard_topics import BUCKET_POS_MAX, BUCKET_POS_MIN, KEYBOARD_PUBLISHER_TOPICS, clamp_pan_angle
from .process import kill_all, save_state, spawn
from .run_session import append_process_banner, bag_record_command, create_run_session, write_run_meta

app = typer.Typer(add_completion=False)
ARDUINO_PAN_PIN = 3
ARDUINO_CAM_HEIGHT_PIN = 9
ARDUINO_BUCKET_PIN = 10
ARDUINO_CONVEYOR_PIN = 7


class RunProfile(str, Enum):
    """Execution profiles for ``lunar run`` (robot / nav / dig are the primary autonomy split)."""

    ROBOT = "robot"
    RC = "rc"
    AUTONOMY = "autonomy"
    NAV = "nav"
    NAV_DIG = "nav-dig"
    TEST_ENCODER = "test-encoder"
    DIG = "dig"
    DIG_BACKUP = "dig-backup"


def _short_segment_nav_stack_commands(
    *,
    grid_preset: str,
    include_navigation_controller: bool,
    include_nav_mission: bool = False,
) -> list[tuple[str, str]]:
    """
    ROS processes for navigation autonomy bring-up (field / sim).

    Perception health, local terrain grid, flag hints, shadow supervisor, optional
    ``navigation_controller`` (zone goal → dig when zones are marked), and optional
    ``nav_mission_executor`` (MAP_EXPLORE → … → handoff publishes ``/autonomy/dig_arm``).
    """
    preset = str(grid_preset).strip().lower() or "standard"
    cmds: list[tuple[str, str]] = [
        ("perception_health", "ros2 run backend perception_health"),
        ("local_terrain_grid", f"ros2 run backend local_terrain_grid --ros-args -p grid_preset:={shlex.quote(preset)}"),
        ("flag_detector", "ros2 run backend flag_detector"),
        ("autonomy_supervisor", "ros2 run backend autonomy_supervisor --ros-args -p allow_motion:=false"),
    ]
    if include_navigation_controller:
        cmds.append(
            (
                "navigation_controller",
                "ros2 run backend navigation_controller --ros-args "
                "-p claim_cmd_vel:=true "
                "-p use_zone_goal:=true -p zone_goal_id:=dig -p goal_preference:=zone",
            )
        )
    if include_nav_mission:
        cmds.append(("nav_mission_executor", "ros2 run backend nav_mission_executor"))
    return cmds


def _resolve_dig_drive_params(
    *,
    profile_label: str,
    calibrated_rotary: Optional[int],
    dig_timing_ms: Optional[int],
) -> tuple[bool, int]:
    """Return (use_timed_legs_ms, value) where value is ms per drive leg or encoder ticks."""
    use_ms = dig_timing_ms is not None and int(dig_timing_ms) > 0
    use_enc = calibrated_rotary is not None and int(calibrated_rotary) > 0
    if use_ms and use_enc:
        typer.secho(
            f"Profile '{profile_label}' accepts only one of --calibrated-rotary or --dig-timing-ms.",
            fg=typer.colors.RED,
            err=True,
        )
        raise typer.Exit(code=2)
    if not use_ms and not use_enc:
        typer.secho(
            f"Profile '{profile_label}' requires --calibrated-rotary <positive_ticks> "
            "or --dig-timing-ms <positive_ms> (same ms for forward and backward drive legs; no encoder).",
            fg=typer.colors.RED,
            err=True,
        )
        raise typer.Exit(code=2)
    if use_ms:
        assert dig_timing_ms is not None
        return True, int(dig_timing_ms)
    assert calibrated_rotary is not None
    return False, int(calibrated_rotary)


class ControlTarget(str, Enum):
    AUTO = "auto"
    SIM = "sim"
    ROBOT = "robot"


class Actuator(str, Enum):
    DRIVE = "drive"
    BUCKET_VEL = "bucket-vel"
    BUCKET_POS = "bucket-pos"
    CONVEYOR = "conveyor"
    CAMERA_HEIGHT = "camera-height"
    PAN = "pan"
    STOP_ALL = "stop-all"


def _detect_ros_setup_script() -> Path | None:
    ros_distro = os.environ.get("ROS_DISTRO")
    if ros_distro:
        candidate = Path("/opt/ros") / ros_distro / "setup.bash"
        if candidate.exists():
            return candidate

    opt_ros = Path("/opt/ros")
    if not opt_ros.exists():
        return None

    candidates = [p / "setup.bash" for p in opt_ros.iterdir() if (p / "setup.bash").exists()]
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]

    preferred = ["galactic", "humble", "foxy", "iron", "jazzy", "rolling"]
    by_name = {c.parent.name: c for c in candidates}
    for name in preferred:
        if name in by_name:
            return by_name[name]
    return sorted(candidates)[0]


def ensure_env(force: bool = False):
    """Ensure the ROS workspace is sourced in the current process."""
    root = find_repo_root()
    setup_script = root / "install" / "setup.bash"
    ros_setup_script = _detect_ros_setup_script()
    cfg = Config.load()

    # If the workspace is already in the path, skip unless forced.
    # Still make sure the requested domain id is applied.
    workspace_path = str(root / "install")
    if not force and workspace_path in os.environ.get("COLCON_PREFIX_PATH", ""):
        os.environ["ROS_DOMAIN_ID"] = str(cfg.domain_id)
        return

    source_parts = []
    if ros_setup_script and ros_setup_script.exists():
        source_parts.append(f"source {ros_setup_script}")
    if setup_script.exists():
        source_parts.append(f"source {setup_script}")

    if source_parts:
        command = " && ".join(source_parts + ["env"])
        proc = subprocess.run(["bash", "-c", command], capture_output=True, text=True)
        for line in proc.stdout.splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                os.environ[key] = value

    # Apply Domain ID from config after sourcing so it cannot be lost.
    os.environ["ROS_DOMAIN_ID"] = str(cfg.domain_id)

    # Add Gazebo plugin path.
    plugin_path = str(root / "src" / "sensor" / "build")
    ros_plugin_path = ""
    if ros_setup_script and ros_setup_script.exists():
        ros_plugin_path = str(ros_setup_script.parent / "lib")
    current_gz_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")

    for path in [plugin_path, ros_plugin_path]:
        if path and path not in current_gz_path.split(":"):
            current_gz_path = f"{path}:{current_gz_path}" if current_gz_path else path
    os.environ["GAZEBO_PLUGIN_PATH"] = current_gz_path


def _subscriber_count(topic: str) -> int:
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic],
            capture_output=True,
            text=True,
            timeout=1.5,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return 0

    for line in result.stdout.splitlines():
        if "Subscription count:" in line:
            try:
                return int(line.split(":", 1)[1].strip())
            except ValueError:
                return 0
    return 0


def _topic_counts(topic: str) -> tuple[int, int]:
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic],
            capture_output=True,
            text=True,
            timeout=1.5,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return 0, 0

    publishers = 0
    subscribers = 0
    for line in result.stdout.splitlines():
        if "Publisher count:" in line:
            try:
                publishers = int(line.split(":", 1)[1].strip())
            except ValueError:
                publishers = 0
        elif "Subscription count:" in line:
            try:
                subscribers = int(line.split(":", 1)[1].strip())
            except ValueError:
                subscribers = 0
    return publishers, subscribers


def _topic_sample(topic: str) -> str:
    try:
        result = subprocess.run(
            ["timeout", "1.5", "ros2", "topic", "echo", "--once", topic],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return "unavailable"
    text = " ".join(line.strip() for line in result.stdout.splitlines() if line.strip())
    if not text:
        err = " ".join(line.strip() for line in result.stderr.splitlines() if line.strip())
        return err or "no sample within 1.5s"
    return text[:180]


def _print_dig_preflight(*, encoder_topic: str, timed_drive_only: bool, dig_shell_cmd: str) -> None:
    typer.secho("=== dig preflight ===", fg=typer.colors.CYAN, bold=True)
    typer.echo(f"command: {dig_shell_cmd}")
    topics = [
        ("wheel command", "/cmd/velocity", False),
        ("IR general", "/sensor/ir", True),
        ("IR left", "/sensor/ir/left", True),
        ("IR right", "/sensor/ir/right", True),
        ("bucket position command", "/cmd/bucket_pos", False),
        ("bucket chain command", "/cmd/bucket_vel", False),
        ("conveyor command", "/cmd/conveyor", False),
    ]
    if not timed_drive_only:
        topics.append(("drive encoder", encoder_topic, True))

    for label, topic, sample in topics:
        pubs, subs = _topic_counts(topic)
        typer.echo(f"{label:24} {topic:28} publishers={pubs} subscribers={subs}")
        if sample:
            typer.echo(f"{'':24} sample: {_topic_sample(topic)}")

    if _topic_counts("/cmd/velocity")[1] <= 0:
        typer.secho("WARNING: /cmd/velocity has no subscribers; wheels cannot move.", fg=typer.colors.RED, bold=True)
    if _topic_counts("/sensor/ir")[0] <= 0:
        typer.secho("WARNING: /sensor/ir has no publishers; dig setup may never finish.", fg=typer.colors.RED, bold=True)
    if not timed_drive_only and _topic_counts(encoder_topic)[0] <= 0:
        typer.secho(f"WARNING: {encoder_topic} has no publishers; encoder drive legs cannot complete.", fg=typer.colors.RED, bold=True)
    typer.secho("=== end dig preflight ===", fg=typer.colors.CYAN, bold=True)


def _on_jetson() -> bool:
    return Path("/etc/nv_tegra_release").exists()


def _detect_arduino_port() -> Path | None:
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


def _arduino_cli_path() -> str | None:
    candidates = [
        shutil.which("arduino-cli"),
        str(Path.home() / ".local" / "bin" / "arduino-cli"),
    ]
    for candidate in candidates:
        if candidate and Path(candidate).exists():
            return candidate
    return None


def _ensure_arduino_cli(cli_path: str) -> None:
    core_list = subprocess.run(
        [cli_path, "core", "list"],
        capture_output=True,
        text=True,
        check=True,
    )
    if "arduino:avr" not in core_list.stdout:
        raise RuntimeError(
            "arduino-cli is installed, but the 'arduino:avr' core is missing. "
            "Install it once with internet access using "
            "'arduino-cli core install arduino:avr'."
        )

    lib_list = subprocess.run(
        [cli_path, "lib", "list"],
        capture_output=True,
        text=True,
        check=True,
    )
    if "Servo" not in lib_list.stdout:
        raise RuntimeError(
            "arduino-cli is installed, but the 'Servo' library is missing. "
            "Install it once with internet access using "
            "'arduino-cli lib install Servo'."
        )


def _flash_servo_firmware(root: Path) -> None:
    if not _on_jetson():
        typer.secho("Skipping Arduino firmware flash: not running on a Jetson host.", fg=typer.colors.BRIGHT_BLACK)
        return

    cli_path = _arduino_cli_path()
    if cli_path is None:
        typer.secho("Skipping Arduino firmware flash: arduino-cli not found.", fg=typer.colors.YELLOW)
        return

    sketch = root / "firmware" / "arduino" / "arduinoLuna" / "arduinoLuna.ino"
    if not sketch.exists():
        typer.secho(f"Skipping Arduino firmware flash: sketch not found at {sketch}", fg=typer.colors.YELLOW)
        return

    port = _detect_arduino_port()
    if port is None:
        typer.secho("Skipping Arduino firmware flash: Arduino port not detected.", fg=typer.colors.YELLOW)
        return

    typer.echo("Validating Arduino firmware toolchain...")
    _ensure_arduino_cli(cli_path)

    typer.echo(f"Compiling arduinoLuna firmware for {port}...")
    build_dir = root / "firmware" / "arduino" / "arduinoLuna" / "build" / "arduino.avr.uno"
    build_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [cli_path, "compile", "--fqbn", "arduino:avr:uno", "--output-dir", str(build_dir), str(sketch)],
        check=True,
    )

    typer.echo(f"Flashing arduinoLuna firmware to {port}...")
    subprocess.run(["pkill", "-f", "arduino_driver"], stderr=subprocess.DEVNULL)
    subprocess.run(
        [cli_path, "upload", "-p", str(port), "--fqbn", "arduino:avr:uno", str(sketch)],
        check=True,
    )
    typer.secho("Arduino firmware flashed successfully.", fg=typer.colors.GREEN)


def resolve_control_topic(target: ControlTarget) -> str:
    if target == ControlTarget.SIM:
        return "/cmd_vel"
    if target == ControlTarget.ROBOT:
        return "cmd/velocity"

    robot_subs = _subscriber_count("cmd/velocity")
    sim_subs = _subscriber_count("/cmd_vel")

    if robot_subs > 0 and sim_subs == 0:
        return "cmd/velocity"
    if sim_subs > 0 and robot_subs == 0:
        return "/cmd_vel"
    if robot_subs > 0 and sim_subs > 0:
        return "cmd/velocity"
    return "cmd/velocity"


def _run_ros_cmd(cmd: list[str]) -> None:
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass


def _force_cleanup_runtime_processes() -> list[str]:
    msgs = kill_all()

    subprocess.run(["pkill", "-9", "Xvfb"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "gzserver"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "streamlit"], stderr=subprocess.DEVNULL)

    # Do not use broad patterns like "ros2", "backend", or "frontend" here:
    # the checkout lives under ~/ros2/... and pkill -f would kill this lunar process.
    targeted_patterns = [
        "[r]os2 launch frontend comp_launch.py",
        "[r]os2 run backend joystick_driver",
        "[r]os2 run backend rgb_transport",
        "[r]os2 run backend main_controller",
        "[r]os2 bag record",
        "[r]viz2",
        "[a]rduino_driver",
        "[d]rive_motors",
        "[b]ucket_spin",
        "[r]gb_driver",
        "[d]epth_driver",
        "[t]265_driver",
        "[c]onveyor",
        "[m]ining_controller",
        "[r]os2 run backend dig_sequence",
        "[t]ag_detector",
        "[f]oxglove_bridge",
        "[s]treamlit run .*lunar/src/lunar/dashboard/app.py",
        "[c]amera_ws.py",
        "[l]unar_camera_ws_bridge",
        "[l]unar_dashboard_bridge",
        "[l]unar_mission_control_bridge",
        "[l]unar.mission_bridge",
        "[l]unar.mission_control_serve",
        "[m]ission-control/dist &&",
        "[m]ission-control.*http.server",
        "[p]npm dev --host .* --port",
        "[p]npm exec vite preview",
        "[v]ite preview",
        "[v]ite .*--host .*--port",
        "[m]ission_bridge",
        "[p]erception_health",
        "[l]ocal_terrain_grid",
        "[f]lag_detector",
        "[a]utonomy_supervisor",
    ]
    for pattern in targeted_patterns:
        subprocess.run(["pkill", "-9", "-f", pattern], stderr=subprocess.DEVNULL)

    msgs.extend(_kill_dashboard_port_listeners([8501, 8767, 8770]))

    subprocess.run(
        ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 daemon stop"],
        stderr=subprocess.DEVNULL,
    )

    if Path("/tmp/.X99-lock").exists():
        try:
            Path("/tmp/.X99-lock").unlink()
        except Exception:
            pass

    return msgs


def _kill_dashboard_port_listeners(ports: list[int]) -> list[str]:
    """Kill orphan dashboard/bridge listeners on the well-known dashboard ports."""
    msgs: list[str] = []
    try:
        import psutil
    except Exception:
        return msgs

    current_pgid = os.getpgrp()
    killed_pgids: set[int] = set()
    wanted = {int(p) for p in ports}
    try:
        conns = psutil.net_connections(kind="inet")
    except Exception as exc:
        return [f"could not inspect dashboard ports: {exc}"]

    for conn in conns:
        if conn.pid is None or conn.status != psutil.CONN_LISTEN:
            continue
        if not conn.laddr or int(conn.laddr.port) not in wanted:
            continue
        try:
            pgid = os.getpgid(int(conn.pid))
        except (ProcessLookupError, PermissionError):
            continue
        if pgid == current_pgid or pgid in killed_pgids:
            continue
        try:
            os.killpg(pgid, signal.SIGKILL)
            killed_pgids.add(pgid)
            msgs.append(f"killed dashboard listener on :{int(conn.laddr.port)} (pid {conn.pid}, pgid {pgid})")
        except (ProcessLookupError, PermissionError) as exc:
            msgs.append(f"failed to kill dashboard listener on :{int(conn.laddr.port)} (pid {conn.pid}): {exc}")
    return msgs


def _run_shell_foreground_tee(shell_cmd: str, log_path: Path) -> int:
    """Run a shell pipeline in the foreground; mirror stdout+stderr to ``log_path`` and the terminal."""
    proc = subprocess.Popen(
        shell_cmd,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=os.environ.copy(),
    )
    assert proc.stdout is not None
    try:
        with log_path.open("a", encoding="utf-8") as logf:
            for line in iter(proc.stdout.readline, ""):
                logf.write(line)
                sys.stdout.write(line)
    finally:
        try:
            proc.stdout.close()
        except OSError:
            pass
    return int(proc.wait() or 0)


def _ros_publish_message(topic: str, msg, count: int, rate: int = 10, wait_for_subscribers: float = 0.75) -> None:
    import rclpy
    from rclpy.node import Node

    created_context = False
    if not rclpy.ok():
        rclpy.init()
        created_context = True

    node = Node("lunar_act_publisher")
    publisher = node.create_publisher(type(msg), topic, 10)

    try:
        deadline = time.time() + max(0.0, wait_for_subscribers)
        while time.time() < deadline and publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.05)

        interval = 1.0 / max(1, rate)
        for _ in range(max(1, count)):
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.01)
            if count > 1:
                time.sleep(interval)

        rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if created_context and rclpy.ok():
            rclpy.shutdown()


def _publish_twist(
    topic: str,
    linear: float,
    angular: float,
    duration: float,
    rate: int = 10,
    *,
    send_stop: bool = True,
) -> None:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    created_context = False
    if not rclpy.ok():
        rclpy.init()
        created_context = True

    node = Node("lunar_drive_publisher")
    publisher = node.create_publisher(Twist, topic, 10)

    try:
        deadline = time.time() + 0.75
        while time.time() < deadline and publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.05)

        msg = Twist()
        msg.linear.x = float(linear)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular)

        interval = 1.0 / max(1, rate)
        count = max(1, int(duration * max(1, rate)))
        for _ in range(count):
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.01)
            if count > 1:
                time.sleep(interval)

        if send_stop and (linear != 0.0 or angular != 0.0):
            stop_msg = Twist()
            for _ in range(10):
                publisher.publish(stop_msg)
                rclpy.spin_once(node, timeout_sec=0.01)
                time.sleep(0.05)
    finally:
        node.destroy_node()
        if created_context and rclpy.ok():
            rclpy.shutdown()


def _publish_once(topic: str, msg_type: str, value: int) -> None:
    if msg_type == "std_msgs/msg/Int8":
        from std_msgs.msg import Int8

        msg = Int8()
    elif msg_type == "std_msgs/msg/Int16":
        from std_msgs.msg import Int16

        msg = Int16()
    else:
        raise ValueError(f"Unsupported one-shot message type: {msg_type}")

    msg.data = int(value)
    _ros_publish_message(topic, msg, count=5, rate=20)


def _publish_repeated(topic: str, msg_type: str, value: int, duration: float, rate: int = 10) -> None:
    if msg_type == "std_msgs/msg/Int8":
        from std_msgs.msg import Int8

        msg = Int8()
    elif msg_type == "std_msgs/msg/Int16":
        from std_msgs.msg import Int16

        msg = Int16()
    else:
        raise ValueError(f"Unsupported repeated message type: {msg_type}")

    msg.data = int(value)
    count = max(1, int(duration * max(1, rate)))
    _ros_publish_message(topic, msg, count=count, rate=rate)


def _write_arduino_value(pin: int, value: int) -> bool:
    port = _detect_arduino_port()
    if port is None:
        return False

    try:
        import serial

        with serial.Serial(str(port), 115200, timeout=1, write_timeout=1) as ser:
            # Opening the port can reset the Arduino on Uno-class boards.
            time.sleep(2.0)
            payload = f"{int(pin)}:{int(value)}\n".encode("utf-8")
            ser.reset_input_buffer()
            ser.write(payload)
            ser.flush()
            time.sleep(0.05)
        return True
    except Exception:
        return False


class _TerminalMode:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


class _PersistentArduinoWriter:
    def __init__(self):
        self.port = _detect_arduino_port()
        self.ser = None

    def open(self) -> bool:
        if self.port is None:
            return False

        try:
            import serial

            self.ser = serial.Serial(str(self.port), 115200, timeout=0, write_timeout=0.05)
            # Uno-class boards reset when the port opens. Pay that cost once at startup,
            # then keep writes low-latency for each keypress.
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            return True
        except Exception:
            self.ser = None
            return False

    def write(self, pin: int, value: int) -> bool:
        if self.ser is None:
            return False

        try:
            self.ser.write(f"{int(pin)}:{int(value)}\n".encode("utf-8"))
            self.ser.flush()
            return True
        except Exception:
            return False

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


def _parse_subsystems(value: str) -> set[str]:
    aliases = {
        "cam": "camera-height",
        "camera": "camera-height",
        "height": "camera-height",
        "camera_height": "camera-height",
        "bucket": "bucket-pos",
        "bucket-position": "bucket-pos",
        "bucket_pos": "bucket-pos",
        "bucket-velocity": "bucket-vel",
        "bucket_vel": "bucket-vel",
    }
    allowed = {"drive", "camera-height", "pan", "bucket-pos", "bucket-vel", "conveyor"}

    requested = {part.strip().lower() for part in value.split(",") if part.strip()}
    if not requested:
        return allowed
    if "all" in requested:
        return allowed
    return {aliases.get(item, item) for item in requested if aliases.get(item, item) in allowed}


def _run_terminal_subsystem_keyboard(
    *,
    subsystems: set[str],
    step: int,
    direct_serial: bool,
    status_hz: float,
):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int16

    if not sys.stdin.isatty():
        typer.secho("lunar keyboard requires an interactive TTY.", fg=typer.colors.RED, err=True)
        raise typer.Exit(code=2)

    created_context = False
    if not rclpy.ok():
        rclpy.init()
        created_context = True

    node = Node("lunar_terminal_keyboard")
    _kt = KEYBOARD_PUBLISHER_TOPICS
    pub_cam_height = node.create_publisher(Int16, _kt["camera-height"], 10)
    pub_pan = node.create_publisher(Int16, _kt["pan"], 10)
    pub_bucket_pos = node.create_publisher(Int16, _kt["bucket-pos"], 10)
    pub_bucket_vel = node.create_publisher(Int16, _kt["bucket-vel"], 10)
    pub_conveyor = node.create_publisher(Int16, _kt["conveyor"], 10)

    arduino = _PersistentArduinoWriter() if direct_serial else None
    direct_ready = bool(arduino and arduino.open())

    camera_height = 0
    pan = 90
    bucket_pos = 0
    conveyor = 0
    bucket_vel = 0
    bucket_pos_coarse_step = step
    bucket_pos_fine_step = 1
    last_status = 0.0
    last_bucket_ts = 0.0
    hold_timeout = 0.25

    def publish(pub, value: int):
        msg = Int16()
        msg.data = int(value)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)

    def set_camera_height(value: int):
        nonlocal camera_height
        camera_height = max(0, min(100, int(value)))
        if direct_ready and arduino is not None and arduino.write(ARDUINO_CAM_HEIGHT_PIN, camera_height):
            return
        publish(pub_cam_height, camera_height)

    def set_pan(value: int):
        nonlocal pan
        pan = clamp_pan_angle(value)
        if direct_ready and arduino is not None and arduino.write(ARDUINO_PAN_PIN, pan):
            return
        publish(pub_pan, pan)

    def set_bucket_pos(value: int):
        nonlocal bucket_pos
        bucket_pos = max(BUCKET_POS_MIN, min(BUCKET_POS_MAX, int(value)))
        if direct_ready and arduino is not None and arduino.write(ARDUINO_BUCKET_PIN, bucket_pos):
            return
        publish(pub_bucket_pos, bucket_pos)

    def set_conveyor(value: int):
        nonlocal conveyor
        conveyor = 1 if int(value) else 0
        if direct_ready and arduino is not None and arduino.write(ARDUINO_CONVEYOR_PIN, conveyor):
            return
        publish(pub_conveyor, conveyor)

    def set_bucket_vel(value: int):
        nonlocal bucket_vel, last_bucket_ts
        bucket_vel = int(value)
        last_bucket_ts = time.monotonic()
        publish(pub_bucket_vel, bucket_vel)

    def stop_transient():
        nonlocal bucket_vel, conveyor
        if "bucket-vel" in subsystems and bucket_vel != 0:
            bucket_vel = 0
            publish(pub_bucket_vel, 0)
        if "conveyor" in subsystems and conveyor != 0:
            conveyor = 0
            set_conveyor(0)
        if "pan" in subsystems:
            publish(pub_pan, 0)

    def render_status(force: bool = False):
        nonlocal last_status
        now = time.monotonic()
        if not force and now - last_status < (1.0 / max(0.1, status_hz)):
            return
        last_status = now
        mode = "direct-serial" if direct_ready else "ros-topics"
        parts = [f"mode={mode}", f"enabled={','.join(sorted(subsystems))}"]
        if "camera-height" in subsystems:
            parts.append(f"camera-height={camera_height}")
        if "pan" in subsystems:
            parts.append(f"pan={pan}")
        if "bucket-pos" in subsystems:
            parts.append(f"bucket-pos={bucket_pos}")
        if "bucket-vel" in subsystems:
            parts.append(f"bucket-vel={bucket_vel}")
        if "conveyor" in subsystems:
            parts.append(f"conveyor={conveyor}")
        sys.stdout.write("\r" + " | ".join(parts) + " " * 16)
        sys.stdout.flush()

    typer.echo("")
    typer.echo("Lunar subsystem keyboard (drive disabled)")
    typer.echo("q/Ctrl-C exit, space stop transient actuators, ? help")
    typer.echo("camera height: u/j up/down, 0 min, 1 max")
    if "pan" in subsystems:
        typer.echo("pan: h/l right/left, m center")
    if "bucket-pos" in subsystems:
        typer.echo("bucket position: i/k coarse (±step) | I/K fine (±1)")
    if "bucket-vel" in subsystems:
        typer.echo("bucket chain: r/f forward/reverse while key repeats")
    if "conveyor" in subsystems:
        typer.echo("conveyor: c toggle")
    typer.echo("")

    try:
        render_status(force=True)
        with _TerminalMode():
            while True:
                readable, _, _ = select.select([sys.stdin], [], [], 0.02)
                now = time.monotonic()

                if readable:
                    key = sys.stdin.read(1)
                    if key in ("q", "\x03"):
                        break
                    if key == "?":
                        sys.stdout.write("\n")
                        typer.echo("Keys: u/j camera height, 0/1 min/max, space stop, q quit")
                    elif key == " ":
                        stop_transient()
                    elif "camera-height" in subsystems and key == "u":
                        set_camera_height(camera_height + step)
                    elif "camera-height" in subsystems and key == "j":
                        set_camera_height(camera_height - step)
                    elif "camera-height" in subsystems and key == "0":
                        set_camera_height(0)
                    elif "camera-height" in subsystems and key == "1":
                        set_camera_height(100)
                    elif "pan" in subsystems and key == "h":
                        set_pan(pan + step)
                    elif "pan" in subsystems and key == "l":
                        set_pan(pan - step)
                    elif "pan" in subsystems and key == "m":
                        set_pan(90)
                    elif "bucket-pos" in subsystems and key == "i":
                        set_bucket_pos(bucket_pos + bucket_pos_coarse_step)
                    elif "bucket-pos" in subsystems and key == "k":
                        set_bucket_pos(bucket_pos - bucket_pos_coarse_step)
                    elif "bucket-pos" in subsystems and key == "I":
                        set_bucket_pos(bucket_pos + bucket_pos_fine_step)
                    elif "bucket-pos" in subsystems and key == "K":
                        set_bucket_pos(bucket_pos - bucket_pos_fine_step)
                    elif "bucket-vel" in subsystems and key == "r":
                        set_bucket_vel(40)
                    elif "bucket-vel" in subsystems and key == "f":
                        set_bucket_vel(-40)
                    elif "conveyor" in subsystems and key == "c":
                        set_conveyor(0 if conveyor else 1)

                if "bucket-vel" in subsystems and bucket_vel != 0 and now - last_bucket_ts > hold_timeout:
                    bucket_vel = 0
                    publish(pub_bucket_vel, 0)

                render_status()
    except KeyboardInterrupt:
        pass
    finally:
        stop_transient()
        if arduino:
            arduino.close()
        node.destroy_node()
        if created_context and rclpy.ok():
            rclpy.shutdown()
        sys.stdout.write("\n")
        sys.stdout.flush()


@app.callback()
def main():
    """Unified CLI for upmoon25-auto."""
    ensure_env()


@app.command()
def sim(
    world: str = typer.Option(None, "--world", help="The .world file to load. If omitted, defaults to the 'world_path' in lunar config. If the file is not found, you will be prompted to select from gz_worlds/."),
    port: int = typer.Option(None, "--port", help="Custom port for the Foxglove Bridge. Defaults to 8765."),
    headless: bool = typer.Option(False, "--headless", help="Run Gazebo without its heavy Qt GUI. Highly recommended for remote SSH sessions. Uses Xvfb for virtual rendering."),
    autonomy: bool = typer.Option(False, "--autonomy", help="Automatically start the 'main_controller' node immediately after spawning the robot."),
    explain: bool = typer.Option(False, "--explain", help="Display the exact shell commands that would be executed without actually starting them."),
    json_out: bool = typer.Option(False, "--json", help="Output machine-readable status after initialization."),
    no_prompt: bool = typer.Option(False, "--no-prompt", help="Disable interactive prompts. If a world is missing, the command will exit with an error instead of asking for input."),
    dry_run: bool = typer.Option(False, "--dry-run", help="Similar to --explain, but formatted specifically for diagnostic logging."),
    foreground: bool = typer.Option(False, "--foreground", help="Run in the foreground. Streams all process logs (Gazebo, ROS, Spawner) directly to your terminal. Press CTRL-C to stop everything."),
):
    """
    Launch the full Gazebo Simulation Environment.

    This is the primary entry point for simulation. It orchestrates:
    1. Initializing the physics engine (gzserver).
    2. Starting the ROS 2 launch system (robot_state_publisher, transforms).
    3. Spawning the physical robot entity ('my_bot') into the world.
    4. Opening the Foxglove Bridge for 3D visualization.
    """
    cfg = Config.load()
    root = find_repo_root()
    world_path = world or cfg.world_path
    bridge_port = port or cfg.port
    use_headless = headless or cfg.headless
    log_path = root / ".lunar" / "last_sim.log"

    # Auto-cleanup existing processes to ensure a clean start
    typer.echo("Cleaning up existing simulation processes...")
    kill_all()
    subprocess.run(["pkill", "-9", "gzserver"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "Xvfb"], stderr=subprocess.DEVNULL)
    if Path("/tmp/.X99-lock").exists():
        try:
            Path("/tmp/.X99-lock").unlink()
        except OSError:
            pass

    # Handle world file selection
    world_file = Path(world_path).expanduser()
    if not world_file.is_absolute():
        world_file = root / world_path

    if not world_file.exists():
        if no_prompt:
            typer.secho(f"Error: World file not found at {world_file}", fg=typer.colors.RED, err=True)
            raise typer.Exit(code=2)
        
        typer.secho(f"World file not found: {world_path}", fg=typer.colors.YELLOW)
        
        worlds_dir = root / "gz_worlds"
        available_worlds = list(worlds_dir.glob("*.world"))
        
        if available_worlds:
            typer.echo("\nAvailable worlds in gz_worlds/:")
            for i, w in enumerate(available_worlds, 1):
                typer.echo(f" {i}) {w.name}")
            
            choice = typer.prompt("\nSelect a world number or enter a custom path", default="1")
            
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(available_worlds):
                    world_file = available_worlds[idx]
                else:
                    world_file = Path(choice).expanduser()
            except ValueError:
                world_file = Path(choice).expanduser()
        else:
            world_file = Path(typer.prompt("No worlds found in gz_worlds/. Enter path to .world")).expanduser()

    if not world_file.exists():
        typer.secho(f"Error: Final world path does not exist: {world_file}", fg=typer.colors.RED, err=True)
        raise typer.Exit(code=2)

    world_path = str(world_file)
    
    gz_cmd = f"gzserver {world_path} -slibgazebo_ros_init.so -slibgazebo_ros_factory.so -slibgazebo_ros_force_system.so -slibgazebo_ros_state.so"
    launch_cmd = "ros2 launch backend sim_launch.py"
    spawn_cmd = "ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_bot -z 0.5 -timeout 60"
    
    if use_headless:
        # Start virtual display if needed
        if not Path("/tmp/.X99-lock").exists():
            subprocess.Popen(["Xvfb", ":99", "-screen", "0", "1280x1024x24"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)
        os.environ["DISPLAY"] = ":99"

    bridge_cmd = f"{cfg.bridge_cmd} --port {bridge_port}"
    auto_cmd = "ros2 run backend main_controller"

    if dry_run:
        typer.echo(f"gz cmd: {gz_cmd}")
        typer.echo(f"launch cmd: {launch_cmd}")
        return

    # Clear old log
    log_path.parent.mkdir(exist_ok=True)
    log_path.write_text(f"--- Sim started at {time.ctime()} ---\n")

    procs = []
    # 1. Start Gazebo
    procs.append(spawn("gazebo", gz_cmd, log_file=log_path))
    time.sleep(5.0)
    
    # 2. Start Robot State & Autonomy Nodes
    procs.append(spawn("ros_nodes", launch_cmd, log_file=log_path))
    time.sleep(5.0)
    
    # 3. Spawn the actual Robot Entity
    procs.append(spawn("spawner", spawn_cmd, log_file=log_path))
    time.sleep(5.0)

    # 4. Start Bridge
    procs.append(spawn("bridge", bridge_cmd, log_file=log_path))
    
    if autonomy:
        time.sleep(0.5)
        procs.append(spawn("autonomy", auto_cmd, log_file=log_path))

    save_state(procs)

    if not json_out:
        typer.echo("Simulation active.")
        typer.echo(f"Foxglove: http://localhost:{bridge_port}")
        typer.echo("Run 'lunar check' to verify status.")

    if not foreground:
        return

    try:
        import selectors
        sel = selectors.DefaultSelector()
        for p in procs:
            if p.proc and p.proc.stdout:
                os.set_blocking(p.proc.stdout.fileno(), False)
                sel.register(p.proc.stdout, selectors.EVENT_READ, p.name)

        while True:
            events = sel.select(timeout=1.0)
            for key, mask in events:
                line = cast(TextIO, key.fileobj).readline()
                if line:
                    typer.secho(f"[{key.data}] {line.strip()}", dim=True)
    except KeyboardInterrupt:
        typer.echo("\nStopping...")
        kill_all()


@app.command()
def run(
    profile: RunProfile = typer.Argument(
        ...,
        help=(
            "Primary profiles (bring-up story):\n\n"
            "- robot: Manual stack — frontend drivers (teleop / sensors).\n"
            "- nav: Navigation autonomy — terrain + supervisor + corridor controller + "
            "start→dig mission executor (publish /autonomy/nav_mission/command start after marking dig zone).\n"
            "- nav-dig: Same as nav, plus dig_sequence in the background waiting on /autonomy/dig_arm "
            "(nav profile transitions into dig when the mission reaches dig handoff). "
            "Requires --calibrated-rotary or --dig-timing-ms.\n"
            "- dig: Dig autonomy alone — foreground ``dig_sequence``. This preserves an already-running "
            "robot stack so `/sensor/ir`, encoders, and `cmd/velocity` subscribers stay alive. "
            "For nav→dig prefer nav-dig. Writes ``.lunar/runs/...`` logs and optional rosbag. "
            "Drive phases: --calibrated-rotary (encoder) or --dig-timing-ms (timed forward/back).\n\n"
            "- dig-backup: Same as dig, plus an extra end-of-cycle conveyor ON window "
            "(~5s) before each repeat.\n\n"
            "Other profiles:\n"
            "- rc: Joystick / RViz (optional --record).\n"
            "- autonomy: Planner / transport / command stack.\n"
            "- test-encoder: Drive + Arduino encoder exercise, then exit."
        ),
    ),
    record: bool = typer.Option(False, "--record", help="Start a rosbag recording of odom and camera data (RC profile only)."),
    session_bag: bool = typer.Option(
        True,
        "--session-bag/--no-session-bag",
        help="Write a curated ros2 bag under `.lunar/runs/<session>/rosbag/` (robot / nav / nav-dig / dig / …). "
        "Disable for long robot-only sessions or when disk space is tight.",
    ),
    session_bag_depth: bool = typer.Option(
        False,
        "--session-bag-depth/--no-session-bag-depth",
        help="Include `/camera/depth/points` in the session bag (large). Default bag omits raw depth.",
    ),
    dry_run: bool = typer.Option(False, "--dry-run", help="Print the commands that would be run for this profile."),
    calibrated_rotary: Optional[int] = typer.Option(
        None,
        "--calibrated-rotary",
        help="dig / nav-dig: Wheel encoder ticks for forward phase (omit if using --dig-timing-ms).",
    ),
    dig_timing_ms: Optional[int] = typer.Option(
        None,
        "--dig-timing-ms",
        help=(
            "dig / nav-dig: Milliseconds for each timed drive leg (forward and backward; encoders unused). "
            "Alternative to --calibrated-rotary."
        ),
    ),
    encoder_side: str = typer.Option(
        "left",
        "--encoder-side",
        help="dig / nav-dig: Subscribe to /sensor/encoder/left or right.",
    ),
    dig_cycles: int = typer.Option(
        8,
        "--dig-cycles",
        min=1,
        max=100,
        help="dig / nav-dig: Number of forward/back dig cycles to run.",
    ),
    bucket_settle_sec: float = typer.Option(
        2.0,
        "--bucket-settle-sec",
        min=0.0,
        help="dig / nav-dig: Seconds to wait after commanding the initial bucket position before wheel motion.",
    ),
    dig_preflight: bool = typer.Option(
        False,
        "--dig-preflight/--no-dig-preflight",
        help="dig / dig-backup: Print slow ROS topic diagnostics before starting.",
    ),
    grid_preset: str = typer.Option(
        "standard",
        "--grid-preset",
        help="nav profile only: Terrain grid preset — coarse, standard, or fine.",
    ),
    skip_cleanup: bool = typer.Option(
        False,
        "--skip-cleanup",
        help=(
            "Leave existing lunar / ROS processes running before this profile starts. "
            "Use when the robot stack is already up (e.g. keep arduino_driver and /sensor/ir), "
            "then add nav or another stack. Dig and dig-backup preserve the robot stack automatically. "
            "For `dig` without --session-bag, exiting dig does not run "
            "automatic teardown of tracked processes so the robot session stays alive."
        ),
    ),
):
    """
    Execute high-level system profiles.

    **robot**, **nav**, and **nav-dig** (plus rc / autonomy / test-encoder) run ROS nodes in the
    background until `lunar kill`. **dig** runs ``dig_sequence`` in the foreground and still
    writes a session folder (``combined.log``, ``RUN_META.txt``, optional ``rosbag/``) like
    other profiles. Use **nav-dig** when you want the navigation autonomy profile to hand off
    into the dig profile via ``/autonomy/dig_arm`` at the dig zone.

    Each ``lunar run`` (including **dig**) creates a timestamped folder under ``.lunar/runs/`` with
    ``combined.log`` (all spawned process output), ``RUN_META.txt``, optional ``rosbag/``
    (``--session-bag``), and ``README.txt``. ``.lunar/runs/latest`` symlinks to the newest session.

    Pass ``--skip-cleanup`` to avoid killing processes already running (e.g. keep ``lunar run robot``
    and ``/sensor/ir`` while starting ``nav`` from another terminal). ``dig`` and ``dig-backup``
    preserve the robot stack automatically because they require the frontend drivers.
    """
    root = find_repo_root()
    
    cmds = []
    if profile == RunProfile.ROBOT:
        cmds.append(("frontend", "ros2 launch frontend comp_launch.py"))
    
    elif profile == RunProfile.RC:
        cmds.append(("rviz", "rviz2"))
        cmds.append(("joystick", "ros2 run backend joystick_driver"))
        if record:
            cmds.append(("bag", "ros2 bag record /odom /camera /depth /points"))
            
    elif profile == RunProfile.AUTONOMY:
        cmds.append(("rviz", "rviz2"))
        cmds.append(("transport", "ros2 run backend rgb_transport"))
        cmds.append(("controller", "ros2 run backend main_controller"))

    elif profile == RunProfile.NAV:
        cmds.extend(
            _short_segment_nav_stack_commands(
                grid_preset=grid_preset,
                include_navigation_controller=True,
                include_nav_mission=True,
            )
        )

    elif profile == RunProfile.NAV_DIG:
        use_ms, fwd_val = _resolve_dig_drive_params(
            profile_label="nav-dig",
            calibrated_rotary=calibrated_rotary,
            dig_timing_ms=dig_timing_ms,
        )
        side = encoder_side.strip().lower()
        if not use_ms and side not in ("left", "right"):
            typer.secho("--encoder-side must be 'left' or 'right'.", fg=typer.colors.RED, err=True)
            raise typer.Exit(code=2)
        cmds.extend(
            _short_segment_nav_stack_commands(
                grid_preset=grid_preset,
                include_navigation_controller=True,
                include_nav_mission=True,
            )
        )
        if use_ms:
            dig_cmd_suffix = f"-p timed_drive_ms:={fwd_val}"
        else:
            dig_cmd_suffix = (
                f"-p calibrated_rotary:={fwd_val} -p encoder_side:={shlex.quote(side)}"
            )
        dig_cmd_suffix = (
            f"{dig_cmd_suffix} -p max_cycles_le:={int(dig_cycles)} "
            f"-p bucket_start_settle_sec:={float(bucket_settle_sec)}"
        )
        cmds.append(
            (
                "dig_sequence",
                "ros2 run backend dig_sequence --ros-args "
                "-p wait_for_nav_dig_arm:=true "
                f"{dig_cmd_suffix}",
            )
        )

    elif profile == RunProfile.TEST_ENCODER:
        cmds.append(("encoder_test", "ros2 launch frontend encoder_test_launch.py"))

    elif profile in (RunProfile.DIG, RunProfile.DIG_BACKUP):
        is_backup_dig = profile == RunProfile.DIG_BACKUP
        effective_skip_cleanup = True
        use_ms, fwd_val = _resolve_dig_drive_params(
            profile_label="dig-backup" if is_backup_dig else "dig",
            calibrated_rotary=calibrated_rotary,
            dig_timing_ms=dig_timing_ms,
        )
        side = encoder_side.strip().lower()
        if not use_ms and side not in ("left", "right"):
            typer.secho("--encoder-side must be 'left' or 'right'.", fg=typer.colors.RED, err=True)
            raise typer.Exit(code=2)
        dig_cmd_list = [
            "ros2",
            "run",
            "backend",
            "dig_sequence",
            "--ros-args",
        ]
        if use_ms:
            dig_cmd_list.extend(["-p", f"timed_drive_ms:={fwd_val}"])
        else:
            dig_cmd_list.extend(["-p", f"calibrated_rotary:={fwd_val}", "-p", f"encoder_side:={side}"])
        dig_cmd_list.extend(["-p", f"max_cycles_le:={int(dig_cycles)}"])
        dig_cmd_list.extend(["-p", f"bucket_start_settle_sec:={float(bucket_settle_sec)}"])
        if is_backup_dig:
            dig_cmd_list.extend(["-p", "end_cycle_conveyor_seconds:=5.0"])
        dig_shell_cmd = " ".join(shlex.quote(x) for x in dig_cmd_list)

        if dry_run:
            typer.echo(dig_shell_cmd)
            if session_bag:
                bag_preview = bag_record_command(
                    root,
                    root / ".lunar" / "runs" / "(dry-run)",
                    _ros_source_env_chain(root),
                    "dig",
                    include_depth=session_bag_depth,
                )
                typer.echo(f"[session_rosbag] {bag_preview}")
            return

        if effective_skip_cleanup:
            typer.secho(
                f"Preserving existing ROS nodes for {profile.value}; robot/frontend stack must stay up for "
                "IR, encoders, and drive subscribers.",
                fg=typer.colors.YELLOW,
            )
        else:
            typer.echo("Stopping existing lunar/ROS processes...")
            cleanup_msgs = _force_cleanup_runtime_processes()
            for msg in cleanup_msgs:
                typer.echo(msg)
            time.sleep(0.5)

        ensure_env()

        if dig_preflight:
            _print_dig_preflight(
                encoder_topic=f"/sensor/encoder/{side}",
                timed_drive_only=use_ms,
                dig_shell_cmd=dig_shell_cmd,
            )
        else:
            typer.echo(f"dig command: {dig_shell_cmd}")

        typer.secho(
            f"Starting {profile.value} sequence in the foreground... "
            "(Ctrl-C to abort; ensure `lunar run robot` is already up.)",
            fg=typer.colors.YELLOW,
        )

        session_dir, combined_log = create_run_session(root, profile.value)
        ros_chain = _ros_source_env_chain(root)
        meta_cmds: list[tuple[str, str]] = []
        bag_cmd_str = ""
        if session_bag:
            bag_cmd_str = bag_record_command(
                root,
                session_dir,
                ros_chain,
                "dig",
                include_depth=session_bag_depth,
            )
            meta_cmds.append(("session_rosbag", bag_cmd_str))
        meta_cmds.append(("dig_sequence", dig_shell_cmd))

        write_run_meta(
            session_dir,
            root,
            profile=profile.value,
            commands=meta_cmds,
            session_bag=bool(session_bag),
            session_bag_depth=session_bag_depth,
        )

        try:
            if session_bag:
                typer.echo(f"[session] Starting ros2 bag → {session_dir / 'rosbag'}")
                append_process_banner(combined_log, "session_rosbag", bag_cmd_str)
                save_state([spawn("session_rosbag", bag_cmd_str, log_file=combined_log)])

            append_process_banner(combined_log, "dig_sequence", dig_shell_cmd)
            typer.echo(f"Session folder: {session_dir}")
            typer.echo(f"Combined log: {combined_log}")
            if session_bag:
                typer.echo(
                    f"Rosbag: {session_dir / 'rosbag'} "
                    f"(curated dig topics; add --session-bag-depth for /camera/depth/points)"
                )

            _run_shell_foreground_tee(dig_shell_cmd, combined_log)
        except KeyboardInterrupt:
            typer.echo(f"\nStopped {profile.value} (KeyboardInterrupt).")
        finally:
            # With --skip-cleanup and no session bag, state.json may still track `lunar run robot`;
            # avoid kill_all() so the robot stack survives dig exit. Rosbag spawns replace state
            # with only the bag writer, so kill_all() still stops the bag when session_bag is on.
            if (not effective_skip_cleanup) or session_bag:
                kill_all()

        typer.echo(f"{profile.value} session finished. Logs under {session_dir}")
        return

    if dry_run:
        bag_preview = ""
        if session_bag and cmds:
            bag_preview = bag_record_command(
                root,
                root / ".lunar" / "runs" / "(dry-run)",
                _ros_source_env_chain(root),
                profile.value,
                include_depth=session_bag_depth,
            )
        for name, cmd in cmds:
            typer.echo(f"[{name}] {cmd}")
        if bag_preview:
            typer.echo(f"[session_rosbag] {bag_preview}")
        return

    if skip_cleanup:
        typer.secho(
            "Skipping pre-launch cleanup (--skip-cleanup); existing ROS nodes are left running.",
            fg=typer.colors.YELLOW,
        )
    else:
        typer.echo("Stopping existing lunar/ROS processes...")
        cleanup_msgs = _force_cleanup_runtime_processes()
        for msg in cleanup_msgs:
            typer.echo(msg)
        time.sleep(0.5)

    ensure_env()
    session_dir, combined_log = create_run_session(root, profile.value)
    launch_cmds: list[tuple[str, str]] = list(cmds)
    if session_bag and launch_cmds:
        launch_cmds.append(
            (
                "session_rosbag",
                bag_record_command(
                    root,
                    session_dir,
                    _ros_source_env_chain(root),
                    profile.value,
                    include_depth=session_bag_depth,
                ),
            )
        )
    write_run_meta(
        session_dir,
        root,
        profile=profile.value,
        commands=launch_cmds,
        session_bag=bool(session_bag and launch_cmds),
        session_bag_depth=session_bag_depth,
    )

    procs = []
    for name, cmd in launch_cmds:
        typer.echo(f"Starting {name}...")
        append_process_banner(combined_log, name, cmd)
        procs.append(spawn(name, cmd, log_file=combined_log))
        time.sleep(0.5)

    save_state(procs)
    typer.echo(f"Profile {profile.value} running in background.")
    typer.echo(f"Session folder: {session_dir}")
    typer.echo(f"Combined log: {combined_log}")
    if session_bag and cmds:
        typer.echo(f"Rosbag output: {session_dir / 'rosbag'} (curated topics; add --session-bag-depth for point cloud)")
    typer.echo("Run 'lunar kill' to stop.")


@app.command()
def keyboard(
    target: ControlTarget = typer.Option(ControlTarget.ROBOT, "--target", help="Velocity target: robot (cmd/velocity), sim (/cmd_vel), or auto-detect."),
    subsystems: str = typer.Option(
        "all",
        "--subsystems",
        help="Comma-separated subsystems to enable: drive,camera-height,pan,bucket-pos,bucket-vel,conveyor,all. Defaults to all.",
    ),
    step: int = typer.Option(5, "--step", min=1, max=25, help="Increment for camera height, pan, and coarse bucket position (i/k). Fine bucket moves use ±1 (Shift+I / Shift+K in TUI, I/K in --raw)."),
    ros_only: bool = typer.Option(True, "--ros-only/--direct-serial", help="Publish ROS topics only by default; use --direct-serial only for standalone camera-height or pan tests."),
    raw: bool = typer.Option(False, "--raw", help="Use the legacy raw terminal loop instead of the Textual TUI."),
    drive_speed: float = typer.Option(35.0, "--drive-speed", help="Drive command magnitude for W/S in robot units."),
    turn_speed: float = typer.Option(35.0, "--turn-speed", help="Turn command magnitude for A/D in robot units."),
    bucket_speed: int = typer.Option(40, "--bucket-speed", help="Bucket chain speed for R/F."),
    drive_timeout: float = typer.Option(0.35, "--drive-timeout", help="Seconds without repeated drive keypresses before publishing stop."),
    bucket_timeout: float = typer.Option(0.35, "--bucket-timeout", help="Seconds without repeated bucket-chain keypresses before publishing stop."),
    publish_hz: float = typer.Option(30.0, "--publish-hz", help="Continuous drive publish rate while a drive key is active."),
    drive_mode: str = typer.Option("latch", "--drive-mode", help="Drive key behavior: latch or hold."),
):
    """
    Control robot subsystems from an SSH-terminal TUI.

    On robot targets this opens a Textual interface for robot subsystems.
    By default this is equivalent to `lunar keyboard --subsystems all --ros-only`.

    Keymap:
    - U/J : camera height up/down
    - 0/1 : camera height min/max
    - I/K : bucket position up/down (coarse; same step as --step)
    - Shift+I / Shift+K : bucket position ±1 (fine)
    - Space : stop transient actuators
    - Q : quit

    On simulation targets this falls back to teleop_twist_keyboard.
    """
    ensure_env()
    topic = resolve_control_topic(target)
    typer.secho("\nKEYBOARD CONTROL", bold=True, fg=typer.colors.CYAN)
    typer.echo(f"Topic: {topic}")

    if topic == "cmd/velocity":
        enabled = _parse_subsystems(subsystems)
        if not enabled:
            typer.secho("No valid subsystem selected.", fg=typer.colors.RED, err=True)
            raise typer.Exit(code=2)

        if not ros_only and enabled not in ({"camera-height"}, {"pan"}):
            typer.secho(
                "Full robot control must use the ROS robot stack. Run `lunar run robot` first, "
                "then `lunar keyboard --subsystems all --ros-only`. Direct serial is only for "
                "standalone camera-height or pan testing.",
                fg=typer.colors.RED,
                err=True,
            )
            raise typer.Exit(code=2)

        if not raw:
            try:
                from .keyboard_tui import run_keyboard_tui
            except RuntimeError as exc:
                typer.secho(str(exc), fg=typer.colors.RED, err=True)
                raise typer.Exit(code=2)

            try:
                run_keyboard_tui(
                    step=step,
                    ros_only=ros_only,
                    subsystems=enabled,
                    drive_speed=drive_speed,
                    turn_speed=turn_speed,
                    bucket_speed=bucket_speed,
                    drive_timeout=drive_timeout,
                    bucket_timeout=bucket_timeout,
                    publish_hz=publish_hz,
                    drive_mode=drive_mode,
                )
            except RuntimeError as exc:
                typer.secho(str(exc), fg=typer.colors.RED, err=True)
                raise typer.Exit(code=2)
            return

        if "drive" in enabled:
            typer.secho(
                "Drive control is available in the Textual TUI only. "
                "Run without --raw, for example: lunar keyboard --subsystems all --ros-only",
                fg=typer.colors.RED,
                err=True,
            )
            raise typer.Exit(code=2)

        _run_terminal_subsystem_keyboard(
            subsystems=enabled,
            step=step,
            direct_serial=not ros_only,
            status_hz=12.0,
        )
        return
    else:
        typer.echo("-" * 20)
        typer.secho("  i : FORWARD", fg=typer.colors.GREEN)
        typer.secho("  k : STOP", fg=typer.colors.RED)
        typer.echo("  j/l : LEFT / RIGHT")
        typer.echo("  u/o : CURVE LEFT / RIGHT")
        typer.echo("  w/x : INCREASE / DECREASE speed limit")
        typer.echo("-" * 20)
        typer.echo("Press 'q' or CTRL-C to exit.\n")
        cmd = ["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard", "--ros-args", "--remap", f"cmd_vel:={topic}"]

    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass


@app.command()
def drive(
    linear: float = typer.Option(1.0, help="Linear velocity in meters per second (m/s). Positive moves forward, negative moves backward."),
    angular: float = typer.Option(0.0, help="Angular velocity in radians per second (rad/s). Positive rotates left, negative rotates right."),
    duration: float = typer.Option(2.0, help="Total time in seconds to continue publishing the velocity command."),
    target: ControlTarget = typer.Option(ControlTarget.ROBOT, "--target", help="Velocity target: robot (cmd/velocity), sim (/cmd_vel), or auto-detect."),
):
    """
    Programmatically drive the robot for a fixed duration.

    This command is designed for non-interactive testing and debugging.
    It publishes a steady stream of geometry_msgs/Twist messages at 10Hz
    for the specified duration. 
    \n
    Example: 'lunar drive --linear 0.5 --duration 5.0' moves the robot 2.5 meters.
    """
    ensure_env()
    topic = resolve_control_topic(target)
    
    typer.echo(f"Driving: linear={linear}, angular={angular} for {duration}s on {topic}...")
    _publish_twist(topic, linear, angular, duration, rate=10)


@app.command()
def act(
    actuator: Actuator = typer.Argument(..., help="Actuator target to command."),
    value: Optional[int] = typer.Option(None, "--value", help="Integer value for the selected actuator."),
    linear: Optional[float] = typer.Option(None, "--linear", help="Drive linear velocity. Falls back to --value for drive."),
    angular: float = typer.Option(0.0, "--angular", help="Drive angular velocity."),
    duration: float = typer.Option(1.0, "--duration", help="Drive publish duration in seconds."),
    rate: int = typer.Option(10, "--rate", help="Drive publish rate in Hz."),
    control_target: ControlTarget = typer.Option(ControlTarget.ROBOT, "--target", help="Drive target: robot (cmd/velocity), sim (/cmd_vel), or auto-detect."),
):
    """
    Send a focused actuator command without typing full ros2 topic pub invocations.

    Examples:
    - lunar act drive --value 50 --duration 1
    - lunar act bucket-vel --value 30
    - lunar act bucket-pos --value 50
    - lunar act conveyor --value 1
    - lunar act camera-height --value 50
    - lunar act pan --value 120
    - lunar act stop-all
    """
    ensure_env()

    if actuator == Actuator.DRIVE:
        drive_linear = linear if linear is not None else float(value if value is not None else 0.0)
        topic = resolve_control_topic(control_target)
        typer.echo(
            f"Acting on {actuator.value}: linear={drive_linear}, angular={angular}, "
            f"duration={duration}s, rate={rate}Hz on {topic}"
        )
        _publish_twist(topic, drive_linear, angular, duration, rate=max(1, rate))
        return

    if actuator == Actuator.STOP_ALL:
        drive_topic = resolve_control_topic(control_target)
        typer.echo(f"Stopping drive, bucket, conveyor, and pan on {drive_topic} and robot actuator topics...")
        _publish_twist(drive_topic, 0.0, 0.0, duration=max(duration, 0.2), rate=max(1, rate))
        kt = KEYBOARD_PUBLISHER_TOPICS
        _publish_once(kt["bucket-vel"], "std_msgs/msg/Int16", 0)
        _publish_once(kt["conveyor"], "std_msgs/msg/Int16", 0)
        _publish_once(kt["pan"], "std_msgs/msg/Int16", 0)
        return

    if value is None:
        typer.secho("--value is required for this actuator.", fg=typer.colors.RED, err=True)
        raise typer.Exit(code=2)

    kt = KEYBOARD_PUBLISHER_TOPICS
    topic_map = {
        Actuator.BUCKET_VEL: (kt["bucket-vel"], "std_msgs/msg/Int16"),
        Actuator.BUCKET_POS: (kt["bucket-pos"], "std_msgs/msg/Int16"),
        Actuator.CONVEYOR: (kt["conveyor"], "std_msgs/msg/Int16"),
        Actuator.CAMERA_HEIGHT: (kt["camera-height"], "std_msgs/msg/Int16"),
        Actuator.PAN: (kt["pan"], "std_msgs/msg/Int16"),
    }
    topic, msg_type = topic_map[actuator]

    if actuator == Actuator.PAN:
        int_value = int(value)
        if int_value in (-1, 0, 1):
            typer.secho(
                "pan now expects an absolute angle, e.g. '--value 90' or '--value 120'.",
                fg=typer.colors.RED,
                err=True,
            )
            raise typer.Exit(code=2)

        int_value = clamp_pan_angle(int_value)
        if _write_arduino_value(ARDUINO_PAN_PIN, int_value):
            typer.echo(f"Acting on {actuator.value}: direct-serial angle={int_value} -> Arduino pin {ARDUINO_PAN_PIN}")
            return

        typer.echo(f"Acting on {actuator.value}: ROS angle={int_value} -> {topic}")
        _publish_once(topic, msg_type, int_value)
        return

    if actuator == Actuator.BUCKET_POS:
        int_value = max(BUCKET_POS_MIN, min(BUCKET_POS_MAX, int(value)))
        if _write_arduino_value(ARDUINO_BUCKET_PIN, int_value):
            typer.echo(
                f"Acting on {actuator.value}: direct-serial value={int_value} -> Arduino pin {ARDUINO_BUCKET_PIN}"
            )
            return

        typer.echo(f"Acting on {actuator.value}: ROS value={int_value} -> {topic}")
        _publish_once(topic, msg_type, int_value)
        return

    if actuator == Actuator.CAMERA_HEIGHT:
        int_value = max(0, min(100, int(value)))
        if _write_arduino_value(ARDUINO_CAM_HEIGHT_PIN, int_value):
            typer.echo(
                f"Acting on {actuator.value}: direct-serial value={int_value} -> Arduino pin {ARDUINO_CAM_HEIGHT_PIN}"
            )
            return

        typer.echo(f"Acting on {actuator.value}: ROS value={int_value} -> {topic}")
        _publish_once(topic, msg_type, int_value)
        return

    if actuator == Actuator.CONVEYOR:
        int_value = 1 if int(value) else 0
        if _write_arduino_value(ARDUINO_CONVEYOR_PIN, int_value):
            typer.echo(
                f"Acting on {actuator.value}: direct-serial value={int_value} -> Arduino pin {ARDUINO_CONVEYOR_PIN}"
            )
            return

        typer.echo(f"Acting on {actuator.value}: ROS value={int_value} -> {topic}")
        _publish_once(topic, msg_type, int_value)
        return

    typer.echo(f"Acting on {actuator.value}: value={value} -> {topic}")
    _publish_once(topic, msg_type, int(value))


@app.command()
def lint():
    """
    Run static analysis and linting across the codebase.

    This command uses:
    1. **ty check**: Validates Python type hints and basic syntax.
    2. **ruff check**: A fast, comprehensive linter for PEP8 compliance and bug detection.
    \n
    It scans `src/backend` and `lunar/src` (see repo-root `pyproject.toml` for excludes).
    """
    root = find_repo_root()
    ty_python = root / "lunar" / ".venv" / "bin" / "python"
    ty_cmd = ["uvx", "ty", "check", "src/backend", "lunar/src"]
    if ty_python.exists():
        ty_cmd.extend(["--python", str(ty_python)])

    typer.secho("--- Running 'ty check' ---", fg=typer.colors.CYAN, bold=True)
    res_ty = subprocess.run(ty_cmd, cwd=str(root))

    typer.echo("")

    typer.secho("--- Running 'ruff check' ---", fg=typer.colors.CYAN, bold=True)
    res_ruff = subprocess.run(["uvx", "ruff", "check", "src/backend", "lunar/src"], cwd=str(root))

    if res_ty.returncode == 0 and res_ruff.returncode == 0:
        typer.secho("\n✨ All lint checks passed!", fg=typer.colors.GREEN, bold=True)
    else:
        typer.secho("\n❌ Linting failed.", fg=typer.colors.RED, bold=True)
        raise typer.Exit(code=1)


@app.command()
def build(
    clean: bool = typer.Option(False, "--clean", help="Remove build, install, and log dirs first"),
):
    root = find_repo_root()
    ensure_env(force=True)

    typer.echo("Stopping running robot/sim processes...")
    kill_all()
    subprocess.run(["pkill", "-f", "ros2 launch frontend comp_launch.py"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "arduino_driver"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "drive_motors"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "bucket_spin"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "rgb_driver"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "depth_driver"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "mining_controller"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "dig_sequence"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "tag_detector"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "conveyor"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "gzserver"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "Xvfb"], stderr=subprocess.DEVNULL)
    subprocess.run(["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL)

    if clean:
        if Path("/tmp/.X99-lock").exists():
            try:
                Path("/tmp/.X99-lock").unlink()
            except OSError:
                pass

        typer.echo("Cleaning stale build...")
        for d in ["build", "install", "log"]:
            path = root / d
            if path.exists():
                shutil.rmtree(path)
    
    typer.echo("Building upmoon25-auto packages...")
    for pkgs in [["interfaces"], ["frontend", "backend"]]:
        cmd = ["colcon", "build", "--packages-select"] + pkgs
        res = subprocess.run(cmd, cwd=str(root))
        if res.returncode != 0:
            typer.echo(f"Error building packages: {pkgs}")
            raise typer.Exit(code=1)

    typer.echo("Checking for Gazebo (Simulation) components...")
    has_gazebo = bool(shutil.which("gz") or shutil.which("gazebo"))
    
    if not has_gazebo:
        typer.secho("Gazebo not found. Skipping plugin build (Normal for physical robot operation).", fg=typer.colors.YELLOW)
    else:
        typer.echo("Building gazebo plugins...")
        sensor_dir = root / "src" / "sensor"
        build_dir = sensor_dir / "build"
        if build_dir.exists():
            shutil.rmtree(build_dir)
        build_dir.mkdir()
        try:
            subprocess.run(["cmake", ".."], cwd=str(build_dir), check=True)
            subprocess.run(["make"], cwd=str(build_dir), check=True)
        except subprocess.CalledProcessError as e:
            typer.echo(f"Error building gazebo plugins: {e}")
            raise typer.Exit(code=1)

    typer.echo("Updating environment...")
    ensure_env(force=True)
    typer.echo(f"ROS_DISTRO={os.environ.get('ROS_DISTRO', 'unknown')}")
    typer.echo(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'unset')}")

    app_dir = root / "lunar" / "mission-control"
    if shutil.which("pnpm") is not None and (app_dir / "package.json").exists():
        if not (app_dir / "node_modules").exists():
            typer.echo("Installing mission-control dependencies (pnpm install)...")
            subprocess.run(["pnpm", "install"], cwd=str(app_dir))
        typer.echo("Building mission-control (pnpm build) for `lunar dashboard` static mode...")
        res_mc = subprocess.run(["pnpm", "build"], cwd=str(app_dir))
        if res_mc.returncode != 0:
            typer.secho(
                "mission-control pnpm build failed (dashboard may need `pnpm build` on a dev machine).",
                fg=typer.colors.YELLOW,
            )

    try:
        _flash_servo_firmware(root)
    except (subprocess.CalledProcessError, RuntimeError) as e:
        typer.secho(f"Arduino firmware flash failed: {e}", fg=typer.colors.RED, err=True)
        raise typer.Exit(code=1)

    typer.echo("Build complete!")


@app.command()
def check(
    live: bool = typer.Option(False, "--live", help="Enable live monitoring mode. Instead of a one-time audit, this stays open and streams high-priority logs (WARN and above) from /rosout directly to your terminal."),
    json_out: bool = typer.Option(False, "--json", help="Emit the full audit results as a machine-readable JSON object. Useful for integration with other tools or automated health monitoring."),
):
    """
    Perform a Deep System Audit of the entire Robot & Simulation stack.

    This command executes a comprehensive diagnostic check across multiple layers:
    \n
    1. **Environment:** Verifies ROS 2 Humble installation, Gazebo Classic availability, and Xvfb (virtual display) status.
    \n
    2. **Process Management:** Checks if core background processes (gzserver, foxglove_bridge, Xvfb) are physically running.
    \n
    3. **Gazebo Physics:** Queries the simulation engine to list all active models. Confirms if the robot entity ('my_bot') has successfully spawned.
    \n
    4. **Telemetry Heartbeat:** Samples the ROS 2 topic graph to ensure active data flow for Odometry (/odom) and Joint States (/joint_states).
    \n
    5. **Pose Verification:** Extracts the robot's real-time X, Y, Z coordinates from the physics engine.
    \n
    6. **Coordinate Tree (TF):** Validates the connectivity of the transformation tree (map -> odom -> base_link).
    \n
    7. **Log Analysis:** Scans the last simulation log for critical ERRORS or process deaths to help identify silent failures.
    """
    ensure_env()
    root = find_repo_root()
    log_path = root / ".lunar" / "last_sim.log"
    from .dashboard.components.hardware import (
        DRIVE_PORTS,
        MINING_SPIN_PORTS,
        _detect_arduino,
        _detect_realsense_devices,
        _first_existing_path,
        _on_jetson,
    )

    def probe_python_module(import_name: str) -> tuple[bool, str]:
        try:
            import importlib.util

            if importlib.util.find_spec(import_name) is not None:
                return True, "lunar env"
        except Exception:
            pass

        if _on_jetson():
            script = (
                "import importlib.util; "
                f"print('1' if importlib.util.find_spec({import_name!r}) is not None else '0')"
            )
            try:
                proc = subprocess.run(
                    ["/usr/bin/python3", "-c", script],
                    capture_output=True,
                    text=True,
                    timeout=3,
                    check=False,
                )
                if proc.stdout.strip() == "1":
                    return True, "system python"
            except Exception:
                pass

        return False, "not found"

    results = {
        "env": {"ros2": bool(shutil.which("ros2")), "gazebo": bool(shutil.which("gz") or shutil.which("gazebo")), "xvfb": bool(shutil.which("Xvfb"))},
        "system_health": {
            "cpu_temp": "N/A",
            "mem_usage": "N/A",
            "disk_free": "N/A",
            "throttled": "Unknown"
        },
        "processes": {},
        "nodes": [],
        "models": [],
        "telemetry": {"odom": False, "joint_states": False, "pos": {"x": 0.0, "y": 0.0, "z": 0.0}},
        "tf_tree": {"map_found": False, "odom_found": False, "base_link_found": False, "wheels_found": False},
        "recent_errors": []
    }

    # --- Real-time System Probes (EE Focus) ---
    import psutil
    results["system_health"]["mem_usage"] = f"{psutil.virtual_memory().percent}%"
    results["system_health"]["disk_free"] = f"{psutil.disk_usage('/').percent}% usage"
    
    # Try to get SoC temperature (Jetson/Linux specific)
    try:
        temp_files = [
            "/sys/class/thermal/thermal_zone0/temp", # CPU
            "/sys/class/thermal/thermal_zone1/temp"  # GPU/SoC
        ]
        temps = []
        for tf in temp_files:
            if Path(tf).exists():
                t = int(Path(tf).read_text().strip()) / 1000.0
                temps.append(f"{t:.1f}C")
        if temps:
            results["system_health"]["cpu_temp"] = " / ".join(temps)
    except Exception:
        pass

    import concurrent.futures
    tomli_loader: Any
    try:
        import tomli as _tomli_mod

        tomli_loader = _tomli_mod
    except ImportError:
        tomli_loader = None

    def run_cmd(cmd, timeout=2.0):
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
        except (OSError, subprocess.TimeoutExpired, ValueError):
            return ""

    # --- Hardware Checks ---
    hw_report = []
    if tomli_loader is None:
         hw_report.append({"name": "TOML Support", "path": "tomli", "status": "MISSING (Pip install failed?)"})
    else:
        hw_config_path = root / "lunar" / "hardware.toml"
        if not hw_config_path.exists():
             hw_report.append({"name": "Config File", "path": str(hw_config_path), "status": "MISSING"})
        else:
            with open(hw_config_path, "rb") as f:
                hw_data = tomli_loader.load(f)
            
            on_jetson = _on_jetson()
            realsense = _detect_realsense_devices()

            # --- New Data-Driven Peripheral Check ---
            # Vision Sensors
            if "vision" in hw_data:
                for v_name, v_cfg in hw_data["vision"].items():
                    model = v_cfg.get("model", v_name)
                    if v_name == "rgb_camera":
                        found = realsense["front_connected"]
                        if found:
                            detail = {"detected_serial": "018322071465"}
                        elif realsense["driver_ok"]:
                            detail = {"detected_device": "Front D435 RGB not detected"}
                        else:
                            detail = {"detected_device": "RealSense checker unavailable"}
                        status = "CONNECTED" if found else ("NO DEVICE" if realsense["driver_ok"] else "CHECKER MISSING")
                    elif v_name == "rear_camera":
                        found = realsense["rear_connected"]
                        if found:
                            detail = {"detected_serial": "018322071045"}
                        elif realsense["driver_ok"]:
                            detail = {"detected_device": "Rear D435 RGB not detected"}
                        else:
                            detail = {"detected_device": "RealSense checker unavailable"}
                        status = "CONNECTED" if found else ("NO DEVICE" if realsense["driver_ok"] else "CHECKER MISSING")
                    else:
                        lib = v_cfg.get("driver_lib")
                        found, scope = probe_python_module(lib) if lib else (False, "not found")
                        detail = {"driver_scope": scope}
                        status = "READY" if found else "MISSING"

                    # Format: [Category] Logical: Physical
                    display_name = f"[Vision] {v_name}: {model}"
                    details = dict(v_cfg)
                    details.update(detail)
                    hw_report.append({"name": display_name, "status": status, "details": details})

            # Actuators & Mechanisms
            if "actuation" in hw_data:
                # Drive Train (Nested)
                if "drive_train" in hw_data["actuation"]:
                    dt = hw_data["actuation"]["drive_train"]
                    ctrl = dt.get("controller", "Sabertooth")
                    
                    # Extract parent config (shared props)
                    parent_cfg = {k: v for k, v in dt.items() if not isinstance(v, dict)}

                    for side in ["left", "right"]:
                        if side in dt:
                            live_paths = DRIVE_PORTS.get(side, [])
                            dev = _first_existing_path(live_paths) or dt[side].get("primary_device_path")
                            exists = _first_existing_path(live_paths) is not None
                            status = "CONNECTED" if exists else ("NO DEVICE" if on_jetson else "MISSING")
                            
                            # Merge details
                            details = parent_cfg.copy()
                            details.update(dt[side])
                            details["runtime_paths"] = ", ".join(live_paths)

                            display_name = f"[Actuation] drive_train ({side}): {ctrl}"
                            hw_report.append({"name": display_name, "status": status, "details": details})
                
                # Mining Mechanism
                if "mining_mechanism" in hw_data["actuation"]:
                    mm = hw_data["actuation"]["mining_mechanism"]
                    on_jetson = Path("/etc/nv_tegra_release").exists()
                    for part in ["spin", "linear"]:
                        if part in mm:
                            cfg = mm[part]
                            ctrl = cfg.get("controller", f"Mining {part}")
                            if part == "linear":
                                status = "GPIO CAPABLE" if on_jetson else "OFF JETSON"
                                dev = "Jetson GPIO path"
                            else:
                                dev = _first_existing_path(MINING_SPIN_PORTS) or cfg.get("device_path")
                                exists = _first_existing_path(MINING_SPIN_PORTS) is not None
                                status = "CONNECTED" if exists else ("NO DEVICE" if on_jetson else "MISSING")
                            display_name = f"[Actuation] mining ({part}): {ctrl}"
                            details = dict(cfg)
                            if part == "spin":
                                details["runtime_paths"] = ", ".join(MINING_SPIN_PORTS)
                            else:
                                details["runtime_note"] = "Host capability only; actuator is not physically probeable here."
                            hw_report.append({"name": display_name, "status": status, "details": details})

            # Microcontrollers
            if "microcontrollers" in hw_data:
                for mc_name, mc_cfg in hw_data["microcontrollers"].items():
                    model = mc_cfg.get("model", mc_name)
                    if mc_name == "arduino":
                        detected = _detect_arduino()
                        dev = detected["path"] or mc_cfg.get("device_path")
                        if detected["connected"]:
                            status = "CONNECTED"
                        elif detected["driver_ok"]:
                            status = "NO DEVICE" if on_jetson else "MISSING"
                        else:
                            status = "CHECKER MISSING"
                    else:
                        dev = mc_cfg.get("device_path")
                        exists = Path(dev).exists() if dev else False
                        status = "CONNECTED" if exists else ("NO DEVICE" if on_jetson else "MISSING")
                    display_name = f"[MCU] {mc_name}: {model}"
                    details = dict(mc_cfg)
                    if mc_name == "arduino":
                        details["runtime_path"] = dev or "not detected"
                    hw_report.append({"name": display_name, "status": status, "details": details})
            
            # Check Python Dependencies
            if "system" in hw_data and "python_dependencies" in hw_data["system"]:
                for lib, desc in hw_data["system"]["python_dependencies"].items():
                    # Handle mapping from toml key to actual import name if needed (e.g. opencv_python -> cv2)
                    import_name = lib
                    if lib == "opencv_python":
                        import_name = "cv2"
                    if lib == "Jetson_GPIO":
                        import_name = "Jetson.GPIO"

                    if lib == "pyrealsense2":
                        found = realsense["driver_ok"]
                        scope = "system python" if found and not probe_python_module(import_name)[0] else "lunar env"
                    else:
                        found, scope = probe_python_module(import_name)
                        
                    status = "INSTALLED" if found else "MISSING"
                    hw_report.append({
                        "name": f"Lib: {lib}", 
                        "status": status, 
                        "details": {"description": desc, "import_path": import_name, "scope": scope}
                    })

    # Map of diagnostic functions to run in parallel
    diag_cmds = {
        "nodes": (["ros2", "node", "list"], 2.0),
        "models": (["ros2", "service", "call", "/get_model_list", "gazebo_msgs/srv/GetModelList", "{}"], 3.0),
        "odom": (["ros2", "topic", "echo", "/odom", "--once"], 2.0),
        "js": (["ros2", "topic", "echo", "/joint_states", "--once"], 2.0),
        "tf": (["ros2", "topic", "echo", "/tf", "--once"], 2.0),
    }

    sim_running = subprocess.run(["pgrep", "gzserver"], capture_output=True).returncode == 0

    if sim_running:
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future_to_diag = {executor.submit(run_cmd, cmd, timeout): key for key, (cmd, timeout) in diag_cmds.items()}
            
            # Collect process statuses while ROS calls are pending
            for proc_name in ["gzserver", "foxglove_bridge", "Xvfb"]:
                is_running = subprocess.run(["pgrep", proc_name], capture_output=True).returncode == 0
                results["processes"][proc_name] = is_running

            # Process results as they come in
            for future in concurrent.futures.as_completed(future_to_diag):
                key = future_to_diag[future]
                out = future.result()
                
                if key == "nodes":
                    results["nodes"] = [n for n in out.splitlines() if n.strip()]
                elif key == "models":
                    if "model_names" in out:
                        import re
                        models = re.findall(r"'(.*?)'", out)
                        results["models"] = [m for m in models if m not in ('', ' ')]
                elif key == "odom":
                    if "position:" in out:
                        results["telemetry"]["odom"] = True
                        import re
                        x = re.search(r"x:\s*([-+]?\d*\.\d+|\d+)", out)
                        y = re.search(r"y:\s*([-+]?\d*\.\d+|\d+)", out)
                        z = re.search(r"z:\s*([-+]?\d*\.\d+|\d+)", out)
                        if x and y and z:
                            results["telemetry"]["pos"] = {"x": float(x.group(1)), "y": float(y.group(1)), "z": float(z.group(1))}
                elif key == "js":
                    results["telemetry"]["joint_states"] = "name:" in out
                elif key == "tf":
                    # Check for both dynamic and static frames in the output
                    results["tf_tree"]["odom_found"] = "frame_id: odom" in out or "child_frame_id: odom" in out
                    results["tf_tree"]["base_link_found"] = "base_link" in out
                    # Check if 'wheel_' is in TF or if any node name contains 'static_wheel'
                    wheels_in_nodes = any("static_wheel" in n for n in results["nodes"])
                    results["tf_tree"]["wheels_found"] = "wheel_" in out or wheels_in_nodes
    else:
        for proc_name in ["gzserver", "foxglove_bridge", "Xvfb"]:
            is_running = subprocess.run(["pgrep", proc_name], capture_output=True).returncode == 0
            results["processes"][proc_name] = is_running

    # Final check: cross-reference node list for TF publishers if topic was empty
    node_str = " ".join(results["nodes"])
    if not results["tf_tree"]["map_found"]:
        results["tf_tree"]["map_found"] = "static_transform_publisher" in node_str or "/robot_state_publisher" in results["nodes"]
    
    # If odom pos is non-zero, odom->base_link must be active
    if results["telemetry"]["odom"] and results["telemetry"]["pos"]["z"] != 0:
        results["tf_tree"]["odom_found"] = True
        results["tf_tree"]["base_link_found"] = True

    if log_path.exists():
        with open(log_path, "r") as f:
            lines = f.readlines()
            results["recent_errors"] = [
                ln.strip()
                for ln in lines
                if "ERROR" in ln.upper() or "process has died" in ln.upper()
            ][-8:]

    if json_out:
        typer.echo(json.dumps(results, indent=2))
        return

    typer.secho("\n🔍 SYSTEM AUDIT", bold=True, underline=True)
    
    typer.echo("\n[Environment]")
    physical_robot_mode = _on_jetson() and not sim_running
    for k, v in results["env"].items():
        if physical_robot_mode and k in {"gazebo", "xvfb"} and not v:
            status = typer.style("OPTIONAL (Robot Mode)", fg=typer.colors.BRIGHT_BLACK)
        else:
            status = typer.style("OK", fg=typer.colors.GREEN) if v else typer.style("MISSING", fg=typer.colors.RED)
        typer.echo(f"  {k.upper():<8}: {status}")

    typer.echo("\n[System Health]")
    sh = results["system_health"]
    typer.echo(f"  SoC TEMP: {sh['cpu_temp']:<16} MEMORY: {sh['mem_usage']:<16} DISK: {sh['disk_free']}")

    typer.echo("\n[Processes]")
    for k, v in results["processes"].items():
        if physical_robot_mode and k in {"gzserver", "Xvfb"} and not v:
            status = typer.style("OPTIONAL (Robot Mode)", fg=typer.colors.BRIGHT_BLACK)
        elif physical_robot_mode and k == "foxglove_bridge" and not v:
            status = typer.style("STOPPED (Optional)", fg=typer.colors.YELLOW)
        else:
            status = typer.style("RUNNING", fg=typer.colors.GREEN) if v else typer.style("STOPPED", fg=typer.colors.RED)
        typer.echo(f"  {k:<16}: {status}")

    if hw_report:
        typer.echo("\n[Hardware]")
        on_jetson = Path("/etc/nv_tegra_release").exists()
        for item in hw_report:
            s = item["status"]
            color = typer.colors.GREEN if s in ["CONNECTED", "INSTALLED", "READY", "GPIO", "GPIO CAPABLE"] else typer.colors.YELLOW
            
            # Label as missing ONLY if we are missing hardware AND not on a Jetson
            if s == "MISSING" and not on_jetson:
                 s = typer.style("MISSING", fg=typer.colors.BRIGHT_BLACK) + " (Not a Jetson)"
            elif s == "MISSING" and on_jetson:
                 s = typer.style("MISSING", fg=typer.colors.RED, bold=True)
            else:
                s = typer.style(s, fg=color)
            
            typer.echo(f"  {item['name']:<60}: {s}")
            
            if "details" in item and item["details"]:
                for k, v in item["details"].items():
                    # Handle nested dicts (like identification in arduino) simple flatten for display
                    if isinstance(v, dict):
                         typer.secho(f"    - {k:<25}:", fg=typer.colors.BRIGHT_BLACK)
                         for sub_k, sub_v in v.items():
                             typer.secho(f"      . {sub_k:<23}: {sub_v}", fg=typer.colors.BRIGHT_BLACK)
                    else:
                        typer.secho(f"    - {k:<25}: {v}", fg=typer.colors.BRIGHT_BLACK)

    if not sim_running:
        typer.secho("\n[Simulation internal checks skipped: gzserver is not running]", fg=typer.colors.BRIGHT_BLACK)
        return

    typer.echo("\n[Gazebo Physics]")
    if results["models"]:
        typer.echo(f"  Models: {', '.join(results['models'])}")
        if "my_bot" in results["models"]:
            typer.secho("  ENTITY 'my_bot': SPAWNED", fg=typer.colors.GREEN)
        else:
            typer.secho("  ENTITY 'my_bot': MISSING", fg=typer.colors.RED)
    else:
        typer.secho("  No models found (Check Gazebo/Bridge connectivity)", fg=typer.colors.YELLOW)

    typer.echo("\n[Telemetry Heartbeat]")
    for k in ["odom", "joint_states"]:
        v = results["telemetry"][k]
        status = typer.style("ACTIVE", fg=typer.colors.GREEN) if v else typer.style("STALE/EMPTY", fg=typer.colors.RED)
        typer.echo(f"  {k.upper():<12}: {status}")
    
    if results["telemetry"]["odom"]:
        pos = results["telemetry"]["pos"]
        typer.echo(f"  Current Pose: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}")

    typer.echo("\n[ROS 2 Nodes]")
    typer.echo(f"  Active Count: {len(results['nodes'])}")
    if results["nodes"]:
        if len(results["nodes"]) <= 20:
            for node in results["nodes"]:
                typer.echo(f"    - {node}")
        else:
            for node in results["nodes"][:5]:
                typer.echo(f"    - {node}")
            typer.echo(f"    ... and {len(results['nodes'])-5} more")

    typer.echo("\n[Coordinate Tree (TF)]")
    for frame, found in results["tf_tree"].items():
        status = typer.style("CONNECTED", fg=typer.colors.GREEN) if found else typer.style("DISCONNECTED", fg=typer.colors.RED)
        typer.echo(f"  {frame.replace('_found','').upper():<10}: {status}")

    typer.echo("\n[Recent Errors/Warnings]")
    if results["recent_errors"]:
        for err_ln in results["recent_errors"]:
            color = typer.colors.RED if "ERROR" in err_ln.upper() or "died" in err_ln.upper() else typer.colors.YELLOW
            typer.secho(f"  {err_ln}", fg=color)
    else:
        typer.secho("  No critical errors found in log.", fg=typer.colors.GREEN)

    if live:
        typer.secho("\n--- Live Monitor (Level: WARN+) ---", bold=True, fg=typer.colors.CYAN)
        cmd = ["ros2", "topic", "echo", "/rosout"]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            while True:
                out = proc.stdout
                if out is None:
                    break
                line = out.readline()
                if not line:
                    break
                if any(x in line for x in ["level: 30", "level: 40", "level: 50"]):
                    typer.secho(line.strip(), fg=typer.colors.YELLOW if "30" in line else typer.colors.RED)
        except KeyboardInterrupt:
            proc.terminate()


@app.command()
def logs():
    """
    Follow the simulation logs in real-time.

    This is a convenience wrapper around 'tail -f'. it streams the merged output
    of Gazebo, the ROS 2 launch system, and the robot spawner.
    \n
    Use this to debug:
    - Plugin loading failures
    - URDF/Xacro parsing errors
    - Physics engine crashes
    """
    root = find_repo_root()
    log_path = root / ".lunar" / "last_sim.log"
    if not log_path.exists():
        typer.echo("No log file found.")
        return
    subprocess.run(["tail", "-f", str(log_path)])


@app.command()
def env(
    json_out: bool = typer.Option(False, "--json", help="Emit environment data as JSON."),
):
    """
    Display current CLI configuration and ROS 2 environment variables.

    Shows the active ROS_DOMAIN_ID, the path to the internal lunar config file,
    and all current simulator defaults (headless mode, port, default world).
    """
    cfg = Config.load()
    payload = {
        "config_path": str(CONFIG_PATH),
        "config": asdict(cfg),
        "env": {
            "ROS_DISTRO": os.environ.get("ROS_DISTRO"),
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
        },
    }
    if json_out:
        typer.echo(json.dumps(payload, indent=2))
        return

    typer.echo(f"config: {CONFIG_PATH}")
    for k, v in asdict(cfg).items():
        typer.echo(f"{k}={v}")
    typer.echo("")
    for k in ["ROS_DISTRO", "ROS_DOMAIN_ID"]:
        if k in os.environ:
            typer.echo(f"{k}={os.environ[k]}")


@app.command("shell-env")
def shell_env():
    """
    Print shell export commands for the active Lunar-managed ROS environment.

    Intended usage:
        eval "$(uv run lunar shell-env)"
    """
    ensure_env(force=True)
    keys = [
        "ROS_DISTRO",
        "ROS_DOMAIN_ID",
        "AMENT_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "LD_LIBRARY_PATH",
        "PATH",
        "PYTHONPATH",
        "GAZEBO_PLUGIN_PATH",
    ]
    for key in keys:
        value = os.environ.get(key)
        if value is not None:
            typer.echo(f"export {key}={shlex.quote(value)}")


@app.command()
def topics():
    """
    List all active ROS 2 topics.

    A quick shortcut to 'ros2 topic list' to verify that nodes are communicating.
    """
    cmd = ["ros2", "topic", "list"]
    subprocess.run(cmd, check=False)


@app.command()
def kill():
    """
    Hard-stop all simulation and background processes.

    This is the 'panic button'. It performs the following:
    1. Sends SIGTERM to all process groups tracked in .lunar/state.json.
    2. Force-kills any remaining 'gzserver', 'Xvfb', and ROS nodes.
    3. Stops the ROS 2 daemon.
    """
    typer.echo("Force-cleaning remaining processes...")
    msgs = _force_cleanup_runtime_processes()
    
    if not msgs:
        typer.echo("Cleanup complete.")
        return
    for m in msgs:
        typer.echo(m)


@app.command()
def config(
    set_value: str = typer.Option(
        None,
        "--set",
        help="Update a configuration value. Format: key=value (e.g., --set headless=true)",
    ),
    json_out: bool = typer.Option(False, "--json", help="Emit configuration as JSON."),
):
    """
    View or modify the persistent lunar configuration.

    Available Keys:
    - world_path: Default world file used by 'lunar sim'.
    - port: Port for the Foxglove WebSocket bridge (default: 8765).
    - headless: If true, 'lunar sim' runs without a Gazebo GUI.
    - domain_id: The ROS_DOMAIN_ID (must be 0-101).
    """
    cfg = Config.load()
    if set_value:
        if "=" not in set_value:
            typer.echo("Expected key=value")
            raise typer.Exit(code=1)
        key, value = set_value.split("=", 1)
        key = key.strip()
        value = value.strip()
        if hasattr(cfg, key):
            # Type conversion
            current_val = getattr(cfg, key)
            if isinstance(current_val, bool):
                setattr(cfg, key, value.lower() in {"1", "true", "yes"})
            elif isinstance(current_val, int):
                setattr(cfg, key, int(value))
            else:
                setattr(cfg, key, value)
            cfg.save()
            if json_out:
                typer.echo(cfg.to_json())
            else:
                typer.echo(f"Updated {key}.")
        else:
            typer.echo(f"Unknown key: {key}")
            raise typer.Exit(code=1)
    else:
        typer.echo(cfg.to_json())


@app.command()
def autonomy_stack(
    grid_preset: str = typer.Option("standard", "--grid-preset", help="Terrain grid preset: coarse, standard, or fine."),
    foreground: bool = typer.Option(False, "--foreground", help="Print startup status but keep nodes tracked in the background."),
    dry_run: bool = typer.Option(False, "--dry-run", help="Print the commands that would be launched."),
    navigation_controller: bool = typer.Option(
        True,
        "--nav-controller/--no-nav-controller",
        help="Also start navigation_controller (publishes /autonomy/navigation_twist when gated).",
    ),
    nav_mission: bool = typer.Option(
        False,
        "--nav-mission/--no-nav-mission",
        help="Also start nav_mission_executor (start→dig mission; use `lunar run nav` for the full nav profile).",
    ),
):
    """
    Launch the shadow-mode perception and autonomy stack.

    This starts perception health, local terrain grid, flag detection, the
    autonomy supervisor (motion disabled by default), and optionally the
    short-segment navigation_controller and nav_mission_executor.
    """
    root = find_repo_root()
    log_path = root / ".lunar" / "autonomy_stack.log"
    ensure_env(force=True)

    cmds = _short_segment_nav_stack_commands(
        grid_preset=grid_preset,
        include_navigation_controller=navigation_controller,
        include_nav_mission=nav_mission,
    )

    if dry_run:
        for name, cmd in cmds:
            typer.echo(f"[{name}] {cmd}")
        return

    log_path.parent.mkdir(exist_ok=True)
    log_path.write_text(f"--- Shadow autonomy stack started at {time.ctime()} ---\n")

    procs = []
    for name, cmd in cmds:
        typer.echo(f"Starting {name}...")
        procs.append(spawn(name, cmd, log_file=log_path))
        time.sleep(0.3)

    save_state(procs)
    typer.echo("Shadow autonomy stack running with autonomous motion disabled.")
    typer.echo(f"Logs: {log_path}")
    typer.echo("Run 'lunar kill' to stop.")

    if foreground:
        typer.echo("Foreground log streaming is not implemented for this command yet; use 'lunar logs'.")


def _ros_source_env_chain(root: Path) -> str:
    """Shell prefix: lunar on PYTHONPATH, source ROS + colcon workspace, ROS_DOMAIN_ID."""
    package_root = root / "lunar" / "src"
    parts = [f"export PYTHONPATH={shlex.quote(str(package_root))}:$PYTHONPATH"]
    ros_setup = _detect_ros_setup_script()
    workspace_setup = root / "install" / "setup.bash"
    if ros_setup and ros_setup.exists():
        parts.append(f"source {ros_setup}")
    if workspace_setup.exists():
        parts.append(f"source {workspace_setup}")
    parts.append(f"export ROS_DOMAIN_ID={Config.load().domain_id}")
    return " && ".join(parts)


def _camera_ws_command(root: Path) -> str:
    """Bash command to run camera_ws.py (JPEG streams + /sensor/ws)."""
    script = shlex.quote(str(root / "lunar" / "src" / "lunar" / "dashboard" / "camera_ws.py"))
    cmd = f"{_ros_source_env_chain(root)} && {sys.executable} {script}"
    ros_arg_parts: list[str] = []
    cam_hz = os.environ.get("LUNAR_MAX_CAMERA_PUSH_HZ", "").strip()
    if cam_hz:
        ros_arg_parts.append(f"-p max_camera_push_hz:={shlex.quote(cam_hz)}")
    sens_hz = os.environ.get("LUNAR_MAX_SENSOR_PUSH_HZ", "").strip()
    if sens_hz:
        ros_arg_parts.append(f"-p max_sensor_push_hz:={shlex.quote(sens_hz)}")
    queue_frames = os.environ.get("LUNAR_CAMERA_WRITE_QUEUE_FRAMES", "").strip()
    if queue_frames:
        ros_arg_parts.append(f"-p camera_write_queue_frames:={shlex.quote(queue_frames)}")
    if ros_arg_parts:
        cmd += " --ros-args " + " ".join(ros_arg_parts)
    return cmd


def _wait_for_tcp_ports(host: str, ports: list[int], timeout_sec: float = 20.0) -> bool:
    """Return True when every port accepts a TCP connection (bridge processes up)."""
    import socket
    import time

    deadline = time.monotonic() + max(0.5, float(timeout_sec))
    pending = set(int(p) for p in ports)
    while pending and time.monotonic() < deadline:
        ready = set()
        for port in pending:
            try:
                with socket.create_connection((host, port), timeout=0.4):
                    ready.add(port)
            except OSError:
                pass
        pending -= ready
        if not pending:
            return True
        time.sleep(0.25)
    return not pending


def _mission_bridge_run_command(root: Path, port: int, host: str) -> str:
    """Bash command to run mission_bridge.main (MissionControlSnapshot on /mission/ws)."""
    return (
        f"{_ros_source_env_chain(root)} && "
        f"{sys.executable} -c \"from lunar.mission_bridge import main; main(port={int(port)}, host={host!r})\""
    )


def _launch_vite_mission_control(
    *,
    port: int,
    host: str,
    background: bool,
    log_name: str,
    with_robot_stack: bool = False,
    stack_bridge_port: int = 8770,
    stack_bridge_host: str = "0.0.0.0",
) -> None:
    """Run mission-control: Vite dev server when pnpm exists, else static ``dist/`` via stdlib http.server."""
    root = find_repo_root()
    app_dir = root / "lunar" / "mission-control"
    dist_dir = app_dir / "dist"
    dist_index = dist_dir / "index.html"
    log_path = root / ".lunar" / f"{log_name}.log"

    if not app_dir.exists():
        typer.secho(f"Error: mission-control app not found at {app_dir}", fg=typer.colors.RED)
        raise typer.Exit(1)

    same_origin_env = "VITE_USE_SAME_ORIGIN_WS=1 " if with_robot_stack else ""
    has_pnpm_deps = shutil.which("pnpm") is not None and (app_dir / "node_modules").is_dir()
    if with_robot_stack and dist_index.is_file():
        py = shlex.quote(sys.executable)
        dist_q = shlex.quote(str(dist_dir))
        host_q = shlex.quote(host)
        serve_mod = (
            "from lunar.mission_control_serve import main; "
            f"main({dist_q!r}, port={int(port)}, host={host_q!r})"
        )
        cmd = f"{_ros_source_env_chain(root)} && {py} -c {shlex.quote(serve_mod)}"
        mode = "static (dist/ + WebSocket proxy on :8501)"
        typer.secho(
            "Serving mission-control from dist/ with same-origin WebSocket proxy.",
            fg=typer.colors.CYAN,
        )
    elif has_pnpm_deps:
        cmd = (
            f"cd {shlex.quote(str(app_dir))} && {same_origin_env}pnpm dev "
            f"--host {shlex.quote(host)} --port {int(port)}"
        )
        mode = "dev (pnpm + Vite)"
    elif dist_index.is_file():
        py = shlex.quote(sys.executable)
        dist_q = shlex.quote(str(dist_dir))
        host_q = shlex.quote(host)
        if sys.version_info >= (3, 8):
            cmd = f"cd {dist_q} && {py} -m http.server {int(port)} --bind {host_q}"
            mode = "static (dist/ + http.server)"
        else:
            cmd = f"cd {dist_q} && {py} -m http.server {int(port)}"
            mode = "static (dist/ + http.server)"
        typer.secho("pnpm not found; serving pre-built mission-control from dist/.", fg=typer.colors.YELLOW)
    else:
        typer.secho(
            "Error: pnpm is not installed and mission-control is not built.\n"
            f"  Install pnpm for dev mode, or run `pnpm build` in\n  {app_dir}\n"
            "  on a machine with Node (e.g. `make mission-control-build`), then copy dist/ here.",
            fg=typer.colors.RED,
        )
        raise typer.Exit(1)

    typer.secho(f"Launching mission control — {mode} — http://{host}:{port}", fg=typer.colors.GREEN, bold=True)

    camera_proc = None
    bridge_proc = None
    camera_log = root / ".lunar" / "camera_ws.log"
    bridge_log = root / ".lunar" / "mission_bridge.log"

    if with_robot_stack:
        cam_path = root / "lunar" / "src" / "lunar" / "dashboard" / "camera_ws.py"
        if not cam_path.is_file():
            typer.secho(f"Error: camera_ws not found at {cam_path}", fg=typer.colors.RED)
            raise typer.Exit(1)
        ensure_env(force=True)
        typer.secho(
            "Bundling camera_ws (:8767) + mission_bridge (:8770); dashboard proxies them on the HTTP port "
            f"({port}) so the browser only opens one URL.",
            fg=typer.colors.CYAN,
        )
        camera_proc = spawn("camera_ws", _camera_ws_command(root), log_file=camera_log)
        bridge_proc = spawn(
            "mission_bridge",
            _mission_bridge_run_command(root, stack_bridge_port, stack_bridge_host),
            log_file=bridge_log,
        )
        if not _wait_for_tcp_ports("127.0.0.1", [8767, stack_bridge_port], timeout_sec=25.0):
            typer.secho(
                "Warning: camera_ws (:8767) or mission_bridge (:8770) not ready yet; "
                f"check {camera_log} and {bridge_log}. The UI will retry WebSockets.",
                fg=typer.colors.YELLOW,
            )
        else:
            typer.secho("Bridge ports :8767 and :8770 are up.", fg=typer.colors.GREEN)

    if background:
        proc = spawn(log_name, cmd, log_file=log_path)
        procs_to_save = []
        if with_robot_stack:
            procs_to_save.extend([camera_proc, bridge_proc])
        procs_to_save.append(proc)
        save_state(procs_to_save)
        typer.echo(f"Mission control running in background. Logs: {log_path}")
        if with_robot_stack:
            typer.echo(f"Also logging camera_ws to {camera_log} and mission_bridge to {bridge_log}")
        typer.echo("Run 'lunar kill' to stop tracked background processes.")
        return

    try:
        subprocess.run(cmd, shell=True, executable="/bin/bash", check=True)
    except KeyboardInterrupt:
        typer.secho("\nMission control stopped.", fg=typer.colors.YELLOW)
    except subprocess.CalledProcessError as e:
        typer.secho(f"Mission control failed with exit code {e.returncode}", fg=typer.colors.RED)
    finally:
        if with_robot_stack and camera_proc is not None and bridge_proc is not None:
            for p in (camera_proc, bridge_proc):
                try:
                    os.killpg(p.pgid, signal.SIGTERM)
                except ProcessLookupError:
                    pass


@app.command()
def mission_control(
    port: int = typer.Option(8501, help="HTTP port (Vite dev or static dist)."),
    host: str = typer.Option("0.0.0.0", help="Host to bind to."),
    background: bool = typer.Option(False, "--background/--foreground", help="Run in the background or foreground."),
    with_robot_stack: bool = typer.Option(
        True,
        "--with-robot-stack/--no-robot-stack",
        help="Also start camera_ws (:8767) and mission_bridge (:8770) for live feeds (same as lunar dashboard).",
    ),
):
    """
    Launch the React mission-control dashboard.

    Uses Vite dev server when pnpm is available; otherwise serves ``dist/`` with Python's http.server
    (build ``dist`` elsewhere, e.g. ``make mission-control-build``).
    """
    _launch_vite_mission_control(
        port=port,
        host=host,
        background=background,
        log_name="mission_control",
        with_robot_stack=with_robot_stack,
    )


@app.command()
def mission_bridge(
    port: int = typer.Option(8770, help="Port for the mission-control telemetry WebSocket."),
    host: str = typer.Option("0.0.0.0", help="Host to bind to."),
    background: bool = typer.Option(False, "--background/--foreground", help="Run in the background or foreground (default)."),
):
    """
    Launch the safe-command WebSocket bridge for the new mission-control app.

    The bridge streams the typed MissionControlSnapshot contract to
    /mission/ws. It accepts only ESTOP, pause, manual takeover, drive stop,
    and zone marking until the full robot-side watchdog exists.
    """
    root = find_repo_root()
    log_path = root / ".lunar" / "mission_bridge.log"

    ensure_env(force=True)

    cmd = _mission_bridge_run_command(root, port, host)

    typer.secho(f"Launching mission bridge on ws://{host}:{port}/mission/ws", fg=typer.colors.GREEN, bold=True)
    if background:
        proc = spawn("mission_bridge", cmd, log_file=log_path)
        save_state([proc])
        typer.echo(f"Mission bridge running in background. Logs: {log_path}")
        typer.echo("Run 'lunar kill' to stop.")
        return

    try:
        subprocess.run(cmd, shell=True, executable="/bin/bash", check=True)
    except KeyboardInterrupt:
        typer.secho("\nMission bridge stopped.", fg=typer.colors.YELLOW)
    except subprocess.CalledProcessError as e:
        typer.secho(f"Mission bridge failed with exit code {e.returncode}", fg=typer.colors.RED)


@app.command()
def dashboard(
    port: int = typer.Option(8501, help="HTTP port (Vite dev or static dist)."),
    host: str = typer.Option("0.0.0.0", help="Host to bind to."),
    background: bool = typer.Option(True, "--foreground/--background", help="Run in the background (default) or foreground."),
    with_robot_stack: bool = typer.Option(
        True,
        "--with-robot-stack/--no-robot-stack",
        help="Also start camera_ws (:8767) and mission_bridge (:8770) so cameras and mission data work without extra commands.",
    ),
):
    """
    Launch the React mission-control operator dashboard.

    By default also starts **camera_ws** (JPEG + /sensor/ws) and **mission_bridge** (/mission/ws), like the legacy
    Streamlit flow, so ``lunar build`` → ``lunar run robot`` → ``lunar dashboard`` is enough on the robot.

    Uses Vite when pnpm is installed; otherwise serves ``lunar/mission-control/dist/`` with Python (no pnpm on device).
    For the legacy Streamlit UI, use ``lunar streamlit-dashboard``.
    """
    _launch_vite_mission_control(
        port=port,
        host=host,
        background=background,
        log_name="dashboard",
        with_robot_stack=with_robot_stack,
    )


@app.command("streamlit-dashboard")
def streamlit_dashboard(
    port: int = typer.Option(8501, help="Port to run Streamlit on."),
    host: str = typer.Option("0.0.0.0", help="Host to bind to."),
    background: bool = typer.Option(True, "--foreground/--background", help="Run in the background (default) or foreground."),
):
    """
    Launch the legacy Streamlit dashboard plus the camera websocket bridge.

    Prefer `lunar dashboard` for the current React mission-control app.
    """
    root = find_repo_root()
    dashboard_path = root / "lunar" / "src" / "lunar" / "dashboard" / "app.py"
    camera_ws_path = root / "lunar" / "src" / "lunar" / "dashboard" / "camera_ws.py"
    log_path = root / ".lunar" / "streamlit_dashboard.log"
    camera_log_path = root / ".lunar" / "camera_ws.log"

    if not dashboard_path.exists():
        typer.secho(f"Error: Streamlit app not found at {dashboard_path}", fg=typer.colors.RED)
        raise typer.Exit(1)
    if not camera_ws_path.exists():
        typer.secho(f"Error: Camera websocket app not found at {camera_ws_path}", fg=typer.colors.RED)
        raise typer.Exit(1)

    ensure_env(force=True)

    typer.secho(f"Launching legacy Streamlit dashboard on http://{host}:{port}", fg=typer.colors.GREEN, bold=True)

    chain = _ros_source_env_chain(root)
    cmd = (
        f"{chain} && {sys.executable} -m streamlit run {shlex.quote(str(dashboard_path))} "
        f"--server.port {int(port)} --server.address {shlex.quote(host)} --logger.level info"
    )
    camera_cmd = _camera_ws_command(root)

    if background:
        typer.echo("Starting Streamlit dashboard in background...")
        camera_proc = spawn("camera_ws", camera_cmd, log_file=camera_log_path)
        proc = spawn("streamlit_dashboard", cmd, log_file=log_path)
        save_state([camera_proc, proc])
        typer.echo(f"Streamlit dashboard running in background. Logs: {log_path}")
        typer.echo("Run 'lunar kill' to stop.")
    else:
        camera_proc = spawn("camera_ws", camera_cmd, log_file=camera_log_path)
        try:
            subprocess.run(cmd, shell=True, executable="/bin/bash", check=True)
        except KeyboardInterrupt:
            typer.secho("\nStreamlit dashboard stopped.", fg=typer.colors.YELLOW)
        except subprocess.CalledProcessError as e:
            typer.secho(f"Streamlit failed with exit code {e.returncode}", fg=typer.colors.RED)
        finally:
            try:
                os.killpg(camera_proc.pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass


if __name__ == "__main__":
    app()
