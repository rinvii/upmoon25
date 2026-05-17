from __future__ import annotations

import asyncio
import threading
import time
import json
import re
import subprocess
from collections import deque

import psutil
import rclpy
import tornado.ioloop
import tornado.web
import tornado.websocket
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import Log
from std_msgs.msg import Float32, Int16, Int32, Int32MultiArray, String
from geometry_msgs.msg import Twist

from lunar.keyboard_topics import BUCKET_POS_MAX, BUCKET_POS_MIN, KEYBOARD_PUBLISHER_TOPICS, KEYBOARD_SENSOR_TOPICS

CAMERA_WS_PORT = 8767
# Outbound JPEG cap (per camera). 0 means every received JPEG is forwarded.
_DEFAULT_CAMERA_PUSH_HZ = 0.0
_DEFAULT_SENSOR_PUSH_HZ = 20.0
_DEFAULT_CAMERA_WRITE_QUEUE_FRAMES = 32
_camera_push_interval_sec = 0.0 if _DEFAULT_CAMERA_PUSH_HZ <= 0.0 else 1.0 / _DEFAULT_CAMERA_PUSH_HZ
_sensor_push_interval_sec = 1.0 / _DEFAULT_SENSOR_PUSH_HZ
_camera_write_queue_frames = _DEFAULT_CAMERA_WRITE_QUEUE_FRAMES

_clients_lock = threading.Lock()
_clients = {"rgb": set(), "rear": set()}
_sensor_clients = set()
_io_loop = None
_last_push_ts = {"rgb": 0.0, "rear": 0.0}
_latest_frames = {"rgb": b"", "rear": b""}
_camera_broadcast_pending = {"rgb": False, "rear": False}
_last_sensor_push_ts = 0.0
_sensor_state = {
    "time": [],
    "history_vel": [],
    "history_base_vel": [],
    "history_cpu": [],
    "history_temp": [],
    "history_battery": [],
    "history_latency": [],
    "linear_vel": 0.0,
    "angular_vel": 0.0,
    "baseline_vel": 0.0,
    "odom_x": 0.0,
    "odom_y": 0.0,
    "cpu_usage": 0.0,
    "ram_usage": 0.0,
    "cpu_temp": 0.0,
    "battery_voltage": 0.0,
    "network_latency": 0.0,
    "ir_left": 0,
    "ir_right": 0,
    "encoder_left": 0,
    "encoder_right": 0,
    "encoder_pin_right": None,
    "encoder_pin_left": None,
    "encoder_dec_right": None,
    "encoder_dec_left": None,
    "encoder_bad_right": None,
    "encoder_bad_left": None,
    "drive_linear": 0.0,
    "drive_angular": 0.0,
    "camera_height": None,
    "pan_angle": 90,
    "bucket_pos": None,
    "bucket_pos_max": BUCKET_POS_MAX,
    "dig_bucket_pos_commanded": None,
    "dig_cycle_counter": None,
    "dig_max_cycles": None,
    "dig_phase": None,
    "bucket_vel": None,
    "conveyor": None,
    "recent_logs": [],
}


def _recent_logs_snapshot():
    rl = _sensor_state.get("recent_logs", [])
    return list(rl) if isinstance(rl, list) else []


def _sensor_float(key: str, default: float = 0.0) -> float:
    v = _sensor_state.get(key, default)
    return float(v) if isinstance(v, (int, float)) else default


_history = {
    "time": deque(maxlen=100),
    "vel": deque(maxlen=100),
    "base_vel": deque(maxlen=100),
    "cpu": deque(maxlen=100),
    "temp": deque(maxlen=100),
    "battery": deque(maxlen=100),
    "latency": deque(maxlen=100),
}


class CameraWebSocketHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self, camera_name):
        self.camera_name = camera_name or "rgb"
        self._camera_write_pending = False
        self._camera_latest_queued_payload = None
        if self.camera_name not in _clients:
            self.close(code=1008, reason="unknown camera")
            return
        with _clients_lock:
            _clients[self.camera_name].add(self)
            payload = _latest_frames.get(self.camera_name, b"")
        if payload:
            self.send_camera_payload(payload)

    def on_close(self):
        with _clients_lock:
            for group in _clients.values():
                group.discard(self)

    def on_message(self, message):
        return

    def send_camera_payload(self, payload: bytes) -> None:
        if not payload or self.ws_connection is None:
            return
        if self._camera_write_pending:
            # Latest-frame wins: overwrite queued payload so stale frames are dropped.
            self._camera_latest_queued_payload = payload
            return
        self._camera_write_pending = True
        try:
            future = self.write_message(payload, binary=True)
        except Exception:
            self.close()
            return
        future.add_done_callback(self._camera_write_done)

    def _camera_write_done(self, future) -> None:
        self._camera_write_pending = False
        try:
            future.result()
        except Exception:
            self.close()
            return
        payload = self._camera_latest_queued_payload
        self._camera_latest_queued_payload = None
        if payload and self.ws_connection is not None:
            self.send_camera_payload(payload)


class SensorWebSocketHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        with _clients_lock:
            _sensor_clients.add(self)
            payload = json.dumps(_sensor_state)
        try:
            self.write_message(payload)
        except Exception:
            self.close()

    def on_close(self):
        with _clients_lock:
            _sensor_clients.discard(self)

    def on_message(self, message):
        return


def _broadcast(camera_name: str, payload: bytes):
    stale = []
    with _clients_lock:
        clients = list(_clients.get(camera_name, set()))

    for client in clients:
        try:
            client.send_camera_payload(payload)
        except Exception:
            stale.append(client)

    if stale:
        with _clients_lock:
            for client in stale:
                _clients.get(camera_name, set()).discard(client)


def _schedule_broadcast(camera_name: str, payload: bytes):
    if not payload or _io_loop is None:
        return
    with _clients_lock:
        _latest_frames[camera_name] = payload

    now = time.monotonic()
    if _camera_push_interval_sec > 0.0 and (now - _last_push_ts[camera_name] < _camera_push_interval_sec):
        return
    _last_push_ts[camera_name] = now
    with _clients_lock:
        if _camera_broadcast_pending.get(camera_name, False):
            return
        _camera_broadcast_pending[camera_name] = True
    _io_loop.add_callback(_drain_camera_broadcast, camera_name)


def _drain_camera_broadcast(camera_name: str):
    try:
        with _clients_lock:
            payload = _latest_frames.get(camera_name, b"")
        if payload:
            _broadcast(camera_name, payload)
    finally:
        with _clients_lock:
            _camera_broadcast_pending[camera_name] = False


def _broadcast_sensors(payload: str):
    stale = []
    with _clients_lock:
        clients = list(_sensor_clients)
    for client in clients:
        try:
            client.write_message(payload)
        except Exception:
            stale.append(client)
    if stale:
        with _clients_lock:
            for client in stale:
                _sensor_clients.discard(client)


def _schedule_sensor_broadcast():
    global _last_sensor_push_ts
    if _io_loop is None:
        return
    now = time.monotonic()
    if _sensor_push_interval_sec > 0.0 and (now - _last_sensor_push_ts < _sensor_push_interval_sec):
        return
    _last_sensor_push_ts = now
    with _clients_lock:
        payload = json.dumps(_sensor_state)
    _io_loop.add_callback(_broadcast_sensors, payload)


def _configure_push_rates(*, max_camera_push_hz: float, max_sensor_push_hz: float, camera_write_queue_frames: int) -> None:
    """Set module-level intervals from ROS parameters (``CameraWsBridge``)."""
    global _camera_push_interval_sec, _sensor_push_interval_sec, _camera_write_queue_frames
    cam = float(max_camera_push_hz)
    if cam <= 0.0:
        _camera_push_interval_sec = 0.0
    else:
        cam = max(1.0, min(cam, 60.0))
        _camera_push_interval_sec = 1.0 / cam
    sens = float(max_sensor_push_hz)
    if sens <= 0.0:
        _sensor_push_interval_sec = 0.0
    else:
        sens = max(1.0, min(sens, 50.0))
        _sensor_push_interval_sec = 1.0 / sens
    _camera_write_queue_frames = max(1, min(int(camera_write_queue_frames), 240))


class CameraWsBridge(Node):
    def __init__(self):
        super().__init__("lunar_camera_ws_bridge")

        self.declare_parameter("max_camera_push_hz", _DEFAULT_CAMERA_PUSH_HZ)
        self.declare_parameter("max_sensor_push_hz", _DEFAULT_SENSOR_PUSH_HZ)
        self.declare_parameter("camera_write_queue_frames", _DEFAULT_CAMERA_WRITE_QUEUE_FRAMES)
        _configure_push_rates(
            max_camera_push_hz=float(self.get_parameter("max_camera_push_hz").value),
            max_sensor_push_hz=float(self.get_parameter("max_sensor_push_hz").value),
            camera_write_queue_frames=int(self.get_parameter("camera_write_queue_frames").value),
        )
        cam_label = "unlimited" if _camera_push_interval_sec <= 0.0 else f"{round(1.0 / _camera_push_interval_sec, 2)} Hz"
        sens_label = "unlimited" if _sensor_push_interval_sec <= 0.0 else f"{round(1.0 / _sensor_push_interval_sec, 2)} Hz"
        self.get_logger().info(
            f"camera_ws push limits: JPEG {cam_label} per camera, /sensor/ws {sens_label} "
            f"write_queue={_camera_write_queue_frames} frames "
            f"(set max_camera_push_hz / max_sensor_push_hz; 0 = no cap)"
        )

        sensor_qos = QoSProfile(
            depth=30,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            CompressedImage,
            "/camera/rgb/image_compressed",
            lambda msg: self.image_compressed_cb("rgb", msg),
            sensor_qos,
        )
        self.create_subscription(
            CompressedImage,
            "/camera/rear/image_compressed",
            lambda msg: self.image_compressed_cb("rear", msg),
            sensor_qos,
        )
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Twist, "cmd/velocity", self.cmd_vel_cb, 10)
        self.create_subscription(Float32, "/sensor/battery", self.battery_cb, 10)
        self.create_subscription(Int16, "/sensor/ir/left", self.ir_left_cb, 10)
        self.create_subscription(Int16, "/sensor/ir/right", self.ir_right_cb, 10)
        self.create_subscription(Int32, "/sensor/encoder/left", self.encoder_left_cb, 10)
        self.create_subscription(Int32, "/sensor/encoder/right", self.encoder_right_cb, 10)
        self.create_subscription(Int32MultiArray, KEYBOARD_SENSOR_TOPICS["encoder_telemetry"], self.encoder_telemetry_cb, 10)
        self.create_subscription(Int16, KEYBOARD_PUBLISHER_TOPICS["camera-height"], self.camera_height_cmd_cb, 10)
        self.create_subscription(Int16, KEYBOARD_PUBLISHER_TOPICS["pan"], self.pan_cmd_cb, 10)
        self.create_subscription(Int16, KEYBOARD_PUBLISHER_TOPICS["bucket-pos"], self.bucket_pos_cmd_cb, 10)
        self.create_subscription(Int16, KEYBOARD_PUBLISHER_TOPICS["bucket-vel"], self.bucket_vel_cmd_cb, 10)
        self.create_subscription(Int16, KEYBOARD_PUBLISHER_TOPICS["conveyor"], self.conveyor_cmd_cb, 10)
        self.create_subscription(String, "/autonomy/dig_sequence/state", self.dig_sequence_state_cb, 10)
        self.create_subscription(Log, "/rosout", self.log_cb, 10)
        self.create_timer(1.0, self.system_tick)
        self.get_logger().info("Camera websocket bridge initialized")

    def image_compressed_cb(self, camera_name: str, msg):
        _schedule_broadcast(camera_name, bytes(msg.data))

    def odom_cb(self, msg):
        with _clients_lock:
            _sensor_state["linear_vel"] = float(msg.twist.twist.linear.x)
            _sensor_state["angular_vel"] = float(msg.twist.twist.angular.z)
            _sensor_state["odom_x"] = float(msg.pose.pose.position.x)
            _sensor_state["odom_y"] = float(msg.pose.pose.position.y)
        _schedule_sensor_broadcast()

    def cmd_vel_cb(self, msg):
        with _clients_lock:
            _sensor_state["baseline_vel"] = float(msg.linear.x)
            _sensor_state["drive_linear"] = float(msg.linear.x)
            _sensor_state["drive_angular"] = float(msg.angular.z)
        _schedule_sensor_broadcast()

    def battery_cb(self, msg):
        with _clients_lock:
            _sensor_state["battery_voltage"] = float(msg.data)
        _schedule_sensor_broadcast()

    def ir_left_cb(self, msg):
        with _clients_lock:
            _sensor_state["ir_left"] = int(msg.data)
        _schedule_sensor_broadcast()

    def ir_right_cb(self, msg):
        with _clients_lock:
            _sensor_state["ir_right"] = int(msg.data)
        _schedule_sensor_broadcast()

    def encoder_left_cb(self, msg):
        with _clients_lock:
            _sensor_state["encoder_left"] = int(msg.data)
        _schedule_sensor_broadcast()

    def encoder_right_cb(self, msg):
        with _clients_lock:
            _sensor_state["encoder_right"] = int(msg.data)
        _schedule_sensor_broadcast()

    def encoder_telemetry_cb(self, msg):
        data = [int(v) for v in msg.data]
        if len(data) < 6:
            return
        with _clients_lock:
            _sensor_state["encoder_pin_right"] = data[0]
            _sensor_state["encoder_pin_left"] = data[1]
            _sensor_state["encoder_dec_right"] = data[2]
            _sensor_state["encoder_dec_left"] = data[3]
            _sensor_state["encoder_bad_right"] = data[4]
            _sensor_state["encoder_bad_left"] = data[5]
        _schedule_sensor_broadcast()

    def camera_height_cmd_cb(self, msg):
        with _clients_lock:
            _sensor_state["camera_height"] = int(msg.data)
        _schedule_sensor_broadcast()

    def pan_cmd_cb(self, msg):
        with _clients_lock:
            _sensor_state["pan_angle"] = int(msg.data)
        _schedule_sensor_broadcast()

    def bucket_pos_cmd_cb(self, msg):
        with _clients_lock:
            _sensor_state["bucket_pos"] = max(BUCKET_POS_MIN, min(BUCKET_POS_MAX, int(msg.data)))
            _sensor_state["bucket_pos_max"] = BUCKET_POS_MAX
        _schedule_sensor_broadcast()

    def bucket_vel_cmd_cb(self, msg):
        with _clients_lock:
            _sensor_state["bucket_vel"] = int(msg.data)
        _schedule_sensor_broadcast()

    def conveyor_cmd_cb(self, msg):
        with _clients_lock:
            _sensor_state["conveyor"] = int(msg.data)
        _schedule_sensor_broadcast()

    def dig_sequence_state_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        with _clients_lock:
            _sensor_state["dig_bucket_pos_commanded"] = payload.get("bucket_pos_commanded")
            _sensor_state["dig_cycle_counter"] = payload.get("cycle_counter")
            _sensor_state["dig_max_cycles"] = payload.get("max_cycles_le")
            _sensor_state["dig_phase"] = payload.get("phase")
            _sensor_state["bucket_pos_max"] = BUCKET_POS_MAX
        _schedule_sensor_broadcast()

    def log_cb(self, msg):
        if msg.level < 30:
            return
        line = f"[{msg.name}] {msg.msg}"
        with _clients_lock:
            logs = _recent_logs_snapshot()
            logs.append(line)
            _sensor_state["recent_logs"] = logs[-20:]
        _schedule_sensor_broadcast()

    def _get_cpu_temp(self) -> float:
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as f:
                return float(f.read().strip()) / 1000.0
        except Exception:
            return 0.0

    def _get_latency(self) -> float:
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", "8.8.8.8"],
                capture_output=True,
                text=True,
                timeout=1.5,
                check=False,
            )
            match = re.search(r"time=([\\d.]+)", result.stdout)
            return float(match.group(1)) if match else 0.0
        except Exception:
            return 0.0

    def system_tick(self):
        now = time.time()
        cpu = float(psutil.cpu_percent())
        ram = float(psutil.virtual_memory().percent)
        temp = float(self._get_cpu_temp())
        latency = float(self._get_latency())
        with _clients_lock:
            _sensor_state["cpu_usage"] = cpu
            _sensor_state["ram_usage"] = ram
            _sensor_state["cpu_temp"] = temp
            _sensor_state["network_latency"] = latency

            _history["time"].append(now)
            _history["vel"].append(_sensor_float("linear_vel"))
            _history["base_vel"].append(_sensor_float("baseline_vel"))
            _history["cpu"].append(cpu)
            _history["temp"].append(temp)
            _history["battery"].append(_sensor_float("battery_voltage"))
            _history["latency"].append(latency)

            t0 = _history["time"][0] if _history["time"] else now
            _sensor_state["time"] = [round(t - t0, 3) for t in _history["time"]]
            _sensor_state["history_vel"] = list(_history["vel"])
            _sensor_state["history_base_vel"] = list(_history["base_vel"])
            _sensor_state["history_cpu"] = list(_history["cpu"])
            _sensor_state["history_temp"] = list(_history["temp"])
            _sensor_state["history_battery"] = list(_history["battery"])
            _sensor_state["history_latency"] = list(_history["latency"])
        _schedule_sensor_broadcast()


def main():
    global _io_loop

    rclpy.init()
    node = CameraWsBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True, name="camera-ws-ros")
    spin_thread.start()

    try:
        asyncio.set_event_loop(asyncio.new_event_loop())
        _io_loop = tornado.ioloop.IOLoop.current()
        app = tornado.web.Application([
            (r"/camera/ws/(?P<camera_name>rgb|rear)", CameraWebSocketHandler),
            (r"/sensor/ws", SensorWebSocketHandler),
        ])
        app.listen(CAMERA_WS_PORT, address="0.0.0.0")
        node.get_logger().info(f"Camera websocket server listening on :{CAMERA_WS_PORT}")
        _io_loop.start()
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
