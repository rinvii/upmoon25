import rclpy
import serial
import serial.tools.list_ports
import subprocess
import time
import os
import numpy as np
from pathlib import Path

from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Int16, Int32
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

'''
    This node handles all interactions with the Arduino. This includes force sensors,
    IR sensors, camera servo, camera linear servo, and bucket linear servo.

    Subscriptions:
    /cmd/pan:           Int16 - +1 for pan left, -1 for pan right, 0 for stop,
                        or absolute angle when outside [-1, 1]
    /cmd/camera_height: Int16, 0 - 100 height percentage for the camera
    /cmd/bucket_pos:    Int16, 0 - 100 extension percentage for the bucket
    /cmd/conveyor:      Int16, 1 to turn on the conveyor, 0 to turn it off

    Publishes:
    /sensor/ir/right - Raw IR measurement from sensor
    /sensor/ir/left

    TF transforms:
    actuator_base_link -> actuator_link - describes height of camera
    actuator_link -> servo_link         - describes camera rotation
'''

DEBUG = False
FLASH_ON_START = os.environ.get("LUNAR_FLASH_ARDUINO_ON_START", "0") == "1"
PAN_MAX = 180
PAN_MIN = 0
ARDUINO_PAN_PIN = 3
ARDUINO_CAM_HEIGHT_PIN = 9
ARDUINO_BUCKET_PIN = 10
ARDUINO_CONVEYOR_PIN = 7

PAN_ANG_0 = 0
PAN_ANG_180 = 280

LIN_SERVO_SPEED = 0.012 # Meters per second
LIN_SERVO_MAX = 0.2 # Meters

class Smoother():

    def __init__(self, n=5):
        self.n=n
        self.idx = 0
        self.num_elems = 0

        self.arr = [0] * self.n

    def add(self, value):
        self.arr[self.idx] = value

        if (self.num_elems < self.n):
            self.num_elems += 1

        self.idx = (self.idx + 1) % self.num_elems

    def getValue(self):
        idx = self.idx
        val = self.arr[idx]
        for i in range(self.num_elems):
            idx = (idx - 1) % self.n
            val += self.arr[idx]

        return round(val / self.num_elems)


class PanState(Enum):
    STOP = 0
    LEFT = 1
    RIGHT = 2

class ArduinoDriver(Node):

    def __init__(self):
        super().__init__('arduino_driver')
        self.get_logger().info('Arduino driver initialized')
        self.get_logger().info("Locating arduino port...")

        # Find and flash to Arduino first
        
        self.port = self.findArduinoPort()
        if self.port:
            if FLASH_ON_START:
                self.flashHexFile(self.port, 'arduinoLuna.ino.hex')
            else:
                self.get_logger().info("Skipping startup firmware flash; use 'lunar build' to flash arduinoLuna.")
        else:
            self.get_logger().error("Could not find arduino port!")
            self.destroy_node()
            rclpy.shutdown()
            return

        self.ser = serial.Serial(self.port, 115200, timeout=0.02, write_timeout=0.05)
        time.sleep(2)

        self.get_logger().info('Found arduino')
        # We have two different callback groups here, so that one thread
        # can listen for command inputs, while the other handles the
        # serial to the Arduino without blocking
        pub_cb = MutuallyExclusiveCallbackGroup()
        sub_cb = MutuallyExclusiveCallbackGroup()

        # Flash succesful: now initialize publishers
        self.ir_pub = self.create_publisher(Int16, '/sensor/ir', 1, callback_group=pub_cb)
        self.ir_right_pub = self.create_publisher(Int16, '/sensor/ir/right', 1, callback_group=pub_cb)
        self.ir_left_pub = self.create_publisher(Int16, '/sensor/ir/left', 1, callback_group=pub_cb)
        self.encoder_left_pub = self.create_publisher(Int32, '/sensor/encoder/left', 1, callback_group=pub_cb)
        self.encoder_right_pub = self.create_publisher(Int32, '/sensor/encoder/right', 1, callback_group=pub_cb)

        self.PUB_campan = self.create_publisher(Int16, '/camera/rgb/pan', 1, callback_group=pub_cb)

        # Initialize subscribers
        self.create_subscription(Int16,  '/cmd/pan', self.onPan, 3, callback_group=sub_cb)
        self.create_subscription(Int16, '/cmd/bucket_pos', self.onBucket, 10, callback_group=sub_cb)
        self.create_subscription(Int16, '/cmd/camera_height', self.onCam, 10, callback_group=sub_cb)
        self.create_subscription(Int16, '/cmd/conveyor', self.onConveyor, 10, callback_group=sub_cb)

        # Transform broadcasters
        self.PUB_joint = self.create_publisher(JointState, '/joint_states', 3, callback_group=sub_cb)

        self.create_timer(0.02, self.camTick, sub_cb)
        self.create_timer(0.02, self.handleSerial, pub_cb)

        self.write = True # Set to true whenever cam pan is updated, false when written to Arduino
        self.pan_state = PanState.STOP
        self.cam_pan = 90

        self.cam_height = 0
        self.cam_height_tf = 0.0
        self.bucket_height = 0
        self.conveyor_enabled = 0
        self.dirty_until = time.monotonic() + 1.0
        self.last_serial_write = 0.0
        self.last_serial_heartbeat = 0.0
        self.serial_write_period = 0.05
        self.serial_heartbeat_period = 0.75

        self.ir_smoother = Smoother(n=20)
        self.encoder_left = 0
        self.encoder_right = 0
        self.ir_left = 0
        self.ir_right = 0

        # Publish initial transforms 
        self.publishActTF(0.0)
        self.publishServoTF(0.0)

    def markDirty(self, duration=0.75):
        self.write = True
        self.dirty_until = max(self.dirty_until, time.monotonic() + duration)

    def onBucket(self, msg):
        self.bucket_height = max(0, min(100, int(msg.data)))
        self.markDirty()

    def onCam(self, msg):
        self.cam_height = max(0, min(100, int(msg.data)))
        self.markDirty()

    def onConveyor(self, msg):
        self.conveyor_enabled = 1 if int(msg.data) else 0
        self.markDirty()

    def writeActuatorState(self):
        commands = [
            f"{ARDUINO_PAN_PIN}:{int(round(self.cam_pan))}\n",
            f"{ARDUINO_CAM_HEIGHT_PIN}:{int(self.cam_height)}\n",
            f"{ARDUINO_BUCKET_PIN}:{int(self.bucket_height)}\n",
            f"{ARDUINO_CONVEYOR_PIN}:{int(self.conveyor_enabled)}\n",
        ]
        for command in commands:
            self.ser.write(command.encode())
        self.ser.flush()
        self.last_serial_write = time.monotonic()

    def handleSerial(self):
        try:
            for _ in range(10):
                if self.ser.in_waiting <= 0:
                    break
                # Read raw bytes first to avoid decode errors
                raw_line = self.ser.readline()
                if not raw_line:
                    break

                try:
                    # Decode with 'ignore' to handle garbage bytes from port contention
                    datum = raw_line.decode('utf-8', errors='ignore').strip()
                except Exception:
                    continue

                if not datum:
                    continue

                # Only process if it starts with the expected header
                if datum.startswith('#'):
                    self.ir_send(datum)
                
                if DEBUG:
                    self.get_logger().info(f'ARDUINO: {datum}')

            now = time.monotonic()
            should_burst = self.write or now < self.dirty_until
            should_heartbeat = now - self.last_serial_heartbeat >= self.serial_heartbeat_period

            if (should_burst and now - self.last_serial_write >= self.serial_write_period) or should_heartbeat:
                self.write = False
                if should_heartbeat:
                    self.last_serial_heartbeat = now
                self.writeActuatorState()

        except Exception as e:
            # Catch 'device disconnected' or 'readiness' errors to prevent node crash
            if "readiness" in str(e) or "disconnected" in str(e):
                self.get_logger().error(f"Serial Port Error (Contention?): {e}")
            pass

    def ir_send(self, data):
        try:
            # Robust cleaning: Keep only digits, colons, and the header
            clean_data = "".join(c for c in data if c.isdigit() or c in ":#")
            if not clean_data.startswith('#'):
                return

            tokens = clean_data.replace('#', '').split(':')
            if len(tokens) < 2:
                return 

            right_raw = int(tokens[0])
            left_raw = int(tokens[1])
            encoder_left_raw = int(tokens[2]) if len(tokens) >= 3 else self.encoder_left
            encoder_right_raw = int(tokens[3]) if len(tokens) >= 4 else self.encoder_right

            self.ir_smoother.add(right_raw)
            self.ir_right = right_raw
            self.ir_left = left_raw
            self.encoder_left = encoder_left_raw
            self.encoder_right = encoder_right_raw

            ir_msg = Int16()
            ir_msg.data = self.ir_smoother.getValue()
            self.ir_pub.publish(ir_msg)

            ir_right_msg = Int16()
            ir_right_msg.data = int(self.ir_right)
            self.ir_right_pub.publish(ir_right_msg)

            ir_left_msg = Int16()
            ir_left_msg.data = int(self.ir_left)
            self.ir_left_pub.publish(ir_left_msg)

            enc_left_msg = Int32()
            enc_left_msg.data = int(self.encoder_left)
            self.encoder_left_pub.publish(enc_left_msg)

            enc_right_msg = Int32()
            enc_right_msg.data = int(self.encoder_right)
            self.encoder_right_pub.publish(enc_right_msg)
        except Exception:
            pass

    # Updates the cam pan
    def camTick(self):
        if (self.pan_state == PanState.LEFT):
            self.cam_pan -= 0.5
            self.markDirty(duration=0.15)
        elif (self.pan_state == PanState.RIGHT):
            self.cam_pan += 0.5
            self.markDirty(duration=0.15)

        if (self.cam_pan < PAN_MIN):
            self.cam_pan = PAN_MIN
        elif(self.cam_pan > PAN_MAX):
            self.cam_pan = PAN_MAX
        
        # Publish the current camera pan
        msg = Int16()
        msg.data = int(self.cam_pan)
        self.PUB_campan.publish(msg)

        # Logic for approximating position of linear servo
        converted_cam_height = (float(self.cam_height) / 100.0) * LIN_SERVO_MAX
        if (self.cam_height_tf < converted_cam_height):
            self.cam_height_tf += (LIN_SERVO_SPEED * 0.04) # Timer runs every 0.04 seconds
        elif (self.cam_height_tf > converted_cam_height):
            self.cam_height_tf -= (LIN_SERVO_SPEED * 0.04)

        # Stops the jittering
        if (abs(self.cam_height_tf - converted_cam_height) <= LIN_SERVO_SPEED * 0.04):
            self.cam_height_tf = converted_cam_height


        # Calculate actual transform of camera
        percentage = float(self.cam_pan) / PAN_MAX
        ang = ((PAN_ANG_180 - PAN_ANG_0) * percentage) + PAN_ANG_0

        self.publishServoTF(ang)
        self.publishActTF(self.cam_height_tf)

    def publishActTF(self, height):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = ['actuator_joint']
        msg.position = np.array([height], dtype=np.float64).tolist()
        msg.velocity = np.array([0], dtype=np.float64).tolist()
        msg.effort = np.array([0], dtype=np.float64).tolist()

        self.PUB_joint.publish(msg)


    def publishServoTF(self, ang):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        radians_ratio = np.pi / 180.0

        msg.name = ['servo_joint']
        msg.position = np.array([-ang * radians_ratio], dtype=np.float64).tolist()
        msg.velocity = np.array([0], dtype=np.float64).tolist()
        msg.effort =   np.array([0], dtype=np.float64).tolist()

        self.PUB_joint.publish(msg)
        
    def onPan(self, msg):
        if msg.data > 1 or msg.data < -1:
            self.pan_state = PanState.STOP
            self.cam_pan = max(PAN_MIN, min(PAN_MAX, int(msg.data)))
            self.markDirty()
        elif (msg.data > 0):
            self.pan_state = PanState.LEFT
            self.markDirty(duration=0.15)
        elif (msg.data < 0):
            self.pan_state = PanState.RIGHT
            self.markDirty(duration=0.15)
        else:
            self.pan_state = PanState.STOP
            self.markDirty(duration=0.15)

    def flashHexFile(self, port, hex_file):
        repo_root = self.findRepoRoot()
        hex_candidates = [
            repo_root / 'firmware' / 'arduino' / 'arduinoLuna' / 'build' / 'arduino.avr.uno' / hex_file,
            repo_root / 'src' / 'frontend' / 'frontend' / 'arduino' / 'hex' / hex_file,
        ]
        hex_path = next((path for path in hex_candidates if path.exists()), None)
        if hex_path is None:
            self.get_logger().warn(f"Hex file {hex_file} not found in firmware build or legacy hex paths. Skipping flash.")
            return

        avrdude_cmd = [
            'avrdude',
            '-v',
            '-V',
            '-c',
            'arduino',
            '-patmega328p',
            'carduino',
            f'-P{port}',
            '-b115200',
            '-D',
            f'-Uflash:w:{hex_path}:i'
        ]

        result = subprocess.run(avrdude_cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info("Flash successful")
        else:
            self.get_logger().warn("Flash failed (firmware might already be present or port busy). Continuing...")

    def findRepoRoot(self):
        current = Path(__file__).resolve()
        for parent in current.parents:
            if (parent / 'src' / 'frontend' / 'frontend').exists():
                return parent
            if (parent / 'firmware' / 'arduino' / 'arduinoLuna').exists():
                return parent
        return current.parents[-1]

    def findArduinoPort(self, arduino_vid='2341', arduino_pid='0043'):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            vid = None if port.vid is None else f"{port.vid:04x}"
            pid = None if port.pid is None else f"{port.pid:04x}"
            if vid == arduino_vid and pid == arduino_pid:
                return port.device

        for fallback in ['/dev/ttyACM2', '/dev/ttyACM1', '/dev/ttyACM0', '/dev/ttyUSB0']:
            if Path(fallback).exists():
                return fallback

        return None




def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = ArduinoDriver()
        if not rclpy.ok():
            return

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Shutting down...')
    finally:
        if executor is not None and node is not None:
            executor.remove_node(node)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
