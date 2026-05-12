import asyncio
import os
import re
import signal
import subprocess
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import psutil
import rclpy
import tornado.ioloop
import tornado.web
import tornado.websocket
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Log
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from std_msgs.msg import Float32, Int16, Int32, String

from lunar.camera_pan_limits import clamp_pan_degrees
from lunar.dashboard.state import store

HOLD_TIMEOUT_SEC = 0.25
WATCHDOG_PERIOD_SEC = 0.05
CAMERA_STREAM_PORT = 8766
CAMERA_WS_PORT = 8767
CAMERA_STREAM_BOUNDARY = b"frame"
CAMERA_PUSH_INTERVAL_SEC = 1.0 / 20.0


def _latest_camera_jpeg() -> bytes:
    snapshot = store.get_snapshot()
    if snapshot.camera_jpeg:
        return snapshot.camera_jpeg

    if snapshot.camera_image is None:
        return b""

    frame = snapshot.camera_image.astype(np.uint8)
    success, jpeg = cv2.imencode(
        ".jpg",
        frame[:, :, ::-1],
        [int(cv2.IMWRITE_JPEG_QUALITY), 60],
    )
    if not success:
        return b""
    return jpeg.tobytes()


class _CameraStreamHandler(BaseHTTPRequestHandler):
    server_version = "LunarCamera/1.0"

    def log_message(self, format, *args):
        return

    def _write_headers(self, code: int, content_type: str, extra_headers=None):
        self.send_response(code)
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Type", content_type)
        if extra_headers:
            for key, value in extra_headers.items():
                self.send_header(key, value)
        self.end_headers()

    def do_GET(self):
        if self.path.startswith("/healthz"):
            self._write_headers(200, "text/plain; charset=utf-8")
            self.wfile.write(b"ok\n")
            return

        if self.path.startswith("/camera/latest.jpg"):
            payload = _latest_camera_jpeg()
            if not payload:
                self._write_headers(503, "text/plain; charset=utf-8")
                self.wfile.write(b"camera unavailable\n")
                return
            self._write_headers(200, "image/jpeg", {"Content-Length": str(len(payload))})
            self.wfile.write(payload)
            return

        if self.path.startswith("/camera/stream.mjpg"):
            self._write_headers(
                200,
                f"multipart/x-mixed-replace; boundary={CAMERA_STREAM_BOUNDARY.decode('ascii')}",
            )
            last_payload = None
            try:
                while True:
                    payload = _latest_camera_jpeg()
                    if not payload:
                        time.sleep(0.1)
                        continue

                    if payload == last_payload:
                        time.sleep(0.03)
                        continue

                    self.wfile.write(b"--" + CAMERA_STREAM_BOUNDARY + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    last_payload = payload
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                return

        self._write_headers(404, "text/plain; charset=utf-8")
        self.wfile.write(b"not found\n")


class _CameraWebSocketHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        _register_camera_ws_client(self)
        payload = _latest_camera_jpeg()
        if payload:
            try:
                self.write_message(payload, binary=True)
            except Exception:
                _unregister_camera_ws_client(self)

    def on_close(self):
        _unregister_camera_ws_client(self)

    def on_message(self, message):
        return


_camera_ws_lock = threading.Lock()
_camera_ws_clients = set()
_camera_ws_loop = None
_camera_ws_thread = None
_last_camera_push_ts = 0.0


def _register_camera_ws_client(client):
    with _camera_ws_lock:
        _camera_ws_clients.add(client)


def _unregister_camera_ws_client(client):
    with _camera_ws_lock:
        _camera_ws_clients.discard(client)


def _broadcast_camera_frame(payload: bytes):
    stale = []
    with _camera_ws_lock:
        clients = list(_camera_ws_clients)
    for client in clients:
        try:
            client.write_message(payload, binary=True)
        except Exception:
            stale.append(client)
    if stale:
        with _camera_ws_lock:
            for client in stale:
                _camera_ws_clients.discard(client)


def _schedule_camera_frame(payload: bytes):
    global _last_camera_push_ts
    if not payload:
        return
    now = time.monotonic()
    if now - _last_camera_push_ts < CAMERA_PUSH_INTERVAL_SEC:
        return
    _last_camera_push_ts = now

    loop = _camera_ws_loop
    if loop is None:
        return
    loop.add_callback(_broadcast_camera_frame, payload)


class DashboardBridge(Node):
    def __init__(self):
        super().__init__("lunar_dashboard_bridge")

        sensor_qos = QoSProfile(
            depth=3,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._logged_image_frame = False

        # Subscribers
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.create_subscription(Twist, "cmd/velocity", self.cmd_vel_cb, 10)
        self.create_subscription(Float32, "/sensor/battery", self.battery_cb, 10)
        self.create_subscription(Log, "/rosout", self.log_cb, 10)
        self.create_subscription(Image, "/camera/rgb/image_raw", self.image_cb, sensor_qos)
        self.create_subscription(
            CompressedImage,
            "/camera/rgb/image_compressed",
            self.image_compressed_cb,
            sensor_qos,
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, 1)
        self.create_subscription(PointCloud2, "/camera/depth/points", self.pc_cb, sensor_qos)
        self.create_subscription(Int16, "/camera/rgb/pan", self.pan_feedback_cb, 10)

        # Hardware Topics
        self.create_subscription(Int16, "/sensor/ir/left", self.ir_left_cb, 10)
        self.create_subscription(Int16, "/sensor/ir/right", self.ir_right_cb, 10)
        self.create_subscription(Int32, "/sensor/encoder/left", self.encoder_left_cb, 10)
        self.create_subscription(Int32, "/sensor/encoder/right", self.encoder_right_cb, 10)

        # Publishers
        self.pub_velocity = self.create_publisher(Twist, "cmd/velocity", 10)
        self.pub_conveyor = self.create_publisher(Int16, "cmd/conveyor", 10)
        self.pub_bucket_vel = self.create_publisher(Int16, "cmd/bucket_vel", 10)
        self.pub_pan = self.create_publisher(Int16, "cmd/pan", 10)
        self.pub_cam_height = self.create_publisher(Int16, "/cmd/camera_height", 10)
        self.pub_bucket = self.create_publisher(Int16, "/cmd/bucket_pos", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.pub_macro = self.create_publisher(String, "/cmd/sequence", 10)
        self.pub_pid = self.create_publisher(Vector3, "/cmd/pid_tuning", 10)

        self._bag_proc = None
        self._last_drive_heartbeat = 0.0
        self._last_pan_heartbeat = 0.0
        self._last_bucket_heartbeat = 0.0
        self._active_drive_cmd = ""
        self._active_pan_cmd = ""
        self._active_bucket_cmd = ""

        # Timer for System Stats (1Hz)
        self.create_timer(1.0, self.system_stats_cb)
        self.create_timer(WATCHDOG_PERIOD_SEC, self._hold_watchdog_cb)

        self.get_logger().info("Dashboard Bridge Initialized (V3 - Teleop)")

    def odom_cb(self, msg):
        store.update(
            odom_x=msg.pose.pose.position.x,
            odom_y=msg.pose.pose.position.y,
            odom_z=msg.pose.pose.position.z,
            linear_vel=msg.twist.twist.linear.x,
            angular_vel=msg.twist.twist.angular.z,
        )

    def cmd_vel_cb(self, msg):
        store.update(baseline_vel=msg.linear.x)

    def battery_cb(self, msg):
        store.update(battery_voltage=msg.data)

    def pc_cb(self, msg):
        store.update(point_cloud_density=msg.width * msg.height)

    def image_cb(self, msg):
        try:
            h, w = msg.height, msg.width
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            if msg.encoding == "bgr8":
                img = img[..., ::-1]
            store.update(camera_image=img, camera_jpeg=None)
            success, jpeg = cv2.imencode(
                ".jpg",
                img[:, :, ::-1],
                [int(cv2.IMWRITE_JPEG_QUALITY), 60],
            )
            if success:
                _schedule_camera_frame(jpeg.tobytes())
            if not self._logged_image_frame:
                self.get_logger().info(f"Received raw RGB frame {w}x{h}")
                self._logged_image_frame = True
        except Exception:
            pass

    def image_compressed_cb(self, msg):
        try:
            payload = bytes(msg.data)
            store.update(camera_jpeg=payload)
            _schedule_camera_frame(payload)
            if not self._logged_image_frame:
                self.get_logger().info(
                    f"Received compressed RGB frame format={msg.format} bytes={len(msg.data)}"
                )
                self._logged_image_frame = True
        except Exception:
            pass

    def map_cb(self, msg):
        try:
            width, height = msg.info.width, msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            store.update(
                map_data=data,
                map_info={
                    "resolution": msg.info.resolution,
                    "origin": (
                        msg.info.origin.position.x,
                        msg.info.origin.position.y,
                    ),
                },
            )
        except Exception:
            pass

    def ir_left_cb(self, msg):
        store.update(ir_left=msg.data)

    def ir_right_cb(self, msg):
        store.update(ir_right=msg.data)

    def encoder_left_cb(self, msg):
        store.update(encoder_left=int(msg.data))

    def encoder_right_cb(self, msg):
        store.update(encoder_right=int(msg.data))

    def pan_feedback_cb(self, msg):
        store.update(camera_pan=int(msg.data))

    def log_cb(self, msg):
        if msg.level >= 30:
            log_str = f"[{msg.name}] {msg.msg}"
            current_logs = store.get_snapshot().recent_logs
            store.update(recent_logs=(current_logs[-14:] + [log_str]))

    def _set_active_hold(self, kind: str, command: str) -> None:
        now = time.monotonic()
        if kind == "drive":
            self._last_drive_heartbeat = now
            self._active_drive_cmd = command
            store.update(active_drive_cmd=command)
        elif kind == "pan":
            self._last_pan_heartbeat = now
            self._active_pan_cmd = command
            store.update(active_pan_cmd=command)
        elif kind == "bucket_vel":
            self._last_bucket_heartbeat = now
            self._active_bucket_cmd = command
            store.update(active_bucket_cmd=command)

    def _clear_hold(self, kind: str) -> None:
        if kind == "drive":
            self._last_drive_heartbeat = 0.0
            self._active_drive_cmd = ""
            store.update(active_drive_cmd="")
        elif kind == "pan":
            self._last_pan_heartbeat = 0.0
            self._active_pan_cmd = ""
            store.update(active_pan_cmd="")
        elif kind == "bucket_vel":
            self._last_bucket_heartbeat = 0.0
            self._active_bucket_cmd = ""
            store.update(active_bucket_cmd="")

    def _hold_watchdog_cb(self) -> None:
        now = time.monotonic()
        if self._active_drive_cmd and (now - self._last_drive_heartbeat) > HOLD_TIMEOUT_SEC:
            self.stop_velocity()
        if self._active_pan_cmd and (now - self._last_pan_heartbeat) > HOLD_TIMEOUT_SEC:
            self.stop_pan()
        if self._active_bucket_cmd and (now - self._last_bucket_heartbeat) > HOLD_TIMEOUT_SEC:
            self.stop_bucket_vel()

    def get_cpu_temp(self):
        try:
            if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
                with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as f:
                    return float(f.read()) / 1000.0
            return 0.0
        except Exception:
            return 0.0

    def system_stats_cb(self):
        cpu = psutil.cpu_percent()
        ram = psutil.virtual_memory().percent
        temp = self.get_cpu_temp()

        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", "8.8.8.8"],
                capture_output=True,
                text=True,
            )
            match = re.search(r"time=([\d.]+)", result.stdout)
            latency = float(match.group(1)) if result.returncode == 0 and match else 0.0
        except Exception:
            latency = 0.0

        store.update(cpu_usage=cpu, ram_usage=ram, cpu_temp=temp, network_latency=latency)

        current = store.get_snapshot()
        with store._lock:
            store._state.history_time.append(time.time())
            store._state.history_cpu.append(cpu)
            store._state.history_vel.append(current.linear_vel)
            store._state.history_base_vel.append(current.baseline_vel)
            store._state.history_latency.append(latency)
            store._state.history_battery.append(current.battery_voltage)
            store._state.history_temp.append(temp)

    def publish_goal(self, x: float, y: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)

    def trigger_macro(self, name: str):
        self.pub_macro.publish(String(data=name))

    def publish_pid(self, p: float, i: float, d: float):
        msg = Vector3()
        msg.x, msg.y, msg.z = p, i, d
        self.pub_pid.publish(msg)
        store.update(kp=p, ki=i, kd=d)
        self.get_logger().info(f"Published new PID: P={p}, I={i}, D={d}")

    def save_map(self, filename: str):
        subprocess.Popen(
            f"ros2 run nav2_map_server map_saver_cli -f /workspace/maps/{filename}",
            shell=True,
            executable="/bin/bash",
        )

    def toggle_recording(self, filename: str):
        state = store.get_snapshot()
        if not state.is_recording:
            proc = subprocess.Popen(
                f"ros2 bag record -o /workspace/bags/{filename} /odom /tf /camera/rgb/image_raw /map",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
            )
            self._bag_proc = proc
            store.update(is_recording=True, bag_filename=filename)
            return

        if self._bag_proc is not None:
            os.killpg(os.getpgid(self._bag_proc.pid), signal.SIGINT)
            self._bag_proc = None
        store.update(is_recording=False, bag_filename="")

    def publish_velocity(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_velocity.publish(msg)

    def stop_velocity(self) -> None:
        self.publish_velocity(0.0, 0.0)
        self._clear_hold("drive")

    def publish_conveyor(self, enabled: bool) -> None:
        self.pub_conveyor.publish(Int16(data=1 if enabled else 0))
        store.update(conveyor_enabled=bool(enabled))

    def publish_bucket_vel(self, value: int) -> None:
        self.pub_bucket_vel.publish(Int16(data=int(value)))

    def stop_bucket_vel(self) -> None:
        self.publish_bucket_vel(0)
        self._clear_hold("bucket_vel")

    def publish_pan(self, angle: int) -> None:
        clamped = clamp_pan_degrees(angle)
        self.pub_pan.publish(Int16(data=clamped))
        store.update(camera_pan=clamped, active_pan_cmd="")

    def publish_pan_jog(self, cmd: int) -> None:
        """Send velocity-style pan: +1 left, -1 right (matches arduino_driver /cmd/pan semantics)."""
        cmd = max(-1, min(1, int(cmd)))
        self.pub_pan.publish(Int16(data=cmd))

    def stop_pan(self) -> None:
        self.pub_pan.publish(Int16(data=0))
        self._clear_hold("pan")

    def stop_all_actuators(self) -> None:
        self.stop_velocity()
        self.stop_bucket_vel()
        self.stop_pan()
        self.publish_conveyor(False)

    def handle_hold_event(self, kind: str, command: str, active: bool) -> None:
        if kind not in {"drive", "pan", "bucket_vel"}:
            return

        if (not active) or command == "stop":
            if kind == "drive":
                self.stop_velocity()
            elif kind == "pan":
                self.stop_pan()
            else:
                self.stop_bucket_vel()
            return

        state = store.get_snapshot()
        self._set_active_hold(kind, command)

        if kind == "drive":
            throttle = float(state.teleop_throttle)
            if command == "forward":
                self.publish_velocity(throttle, 0.0)
            elif command == "back":
                self.publish_velocity(-throttle, 0.0)
            elif command == "left_arc":
                self.publish_velocity(throttle, -1.0)
            elif command == "right_arc":
                self.publish_velocity(throttle, 1.0)
            else:
                self.stop_velocity()
        elif kind == "pan":
            if command == "left":
                self.publish_pan_jog(1)
            elif command == "right":
                self.publish_pan_jog(-1)
            else:
                self.stop_pan()
        elif kind == "bucket_vel":
            speed = int(state.bucket_chain_speed)
            if command == "forward":
                self.publish_bucket_vel(speed)
            elif command == "reverse":
                self.publish_bucket_vel(-speed)
            else:
                self.stop_bucket_vel()

    def publish_cam_height(self, value: int):
        self.pub_cam_height.publish(Int16(data=value))
        store.update(camera_height=value)

    def publish_bucket_pos(self, value: int):
        self.pub_bucket.publish(Int16(data=value))
        store.update(bucket_pos=value)


_node = None
_thread = None
_thread_lock = threading.RLock()
_camera_http_server = None
_camera_http_thread = None


def _start_camera_http_server():
    global _camera_http_server, _camera_http_thread
    with _thread_lock:
        if _camera_http_server is not None and _camera_http_thread is not None and _camera_http_thread.is_alive():
            return

        _camera_http_server = ThreadingHTTPServer(("0.0.0.0", CAMERA_STREAM_PORT), _CameraStreamHandler)

        def serve():
            try:
                _camera_http_server.serve_forever(poll_interval=0.2)
            finally:
                try:
                    _camera_http_server.server_close()
                except Exception:
                    pass

        _camera_http_thread = threading.Thread(target=serve, daemon=True, name="lunar-camera-http")
        _camera_http_thread.start()


def _start_camera_ws_server():
    global _camera_ws_loop, _camera_ws_thread
    with _thread_lock:
        if _camera_ws_loop is not None and _camera_ws_thread is not None and _camera_ws_thread.is_alive():
            return

        def serve():
            global _camera_ws_loop
            try:
                asyncio.set_event_loop(asyncio.new_event_loop())
                loop = tornado.ioloop.IOLoop.current()
                _camera_ws_loop = loop
                app = tornado.web.Application([
                    (r"/camera/ws", _CameraWebSocketHandler),
                ])
                app.listen(CAMERA_WS_PORT, address="0.0.0.0")
                loop.start()
            except Exception:
                _camera_ws_loop = None
                traceback.print_exc()

        _camera_ws_thread = threading.Thread(target=serve, daemon=True, name="lunar-camera-ws")
        _camera_ws_thread.start()


def start_ros_thread():
    global _node, _thread
    with _thread_lock:
        if _node is not None and _thread is not None and _thread.is_alive():
            _start_camera_http_server()
            return
        if not rclpy.ok():
            rclpy.init()
        _node = DashboardBridge()
        _start_camera_http_server()

        def spin():
            executor = SingleThreadedExecutor()
            executor.add_node(_node)
            try:
                executor.spin()
            finally:
                try:
                    executor.remove_node(_node)
                except Exception:
                    pass
                executor.shutdown()

        _thread = threading.Thread(target=spin, daemon=True)
        _thread.start()


def get_node():
    return _node
