import rclpy
import numpy as np
import rclpy.timer
import os
from std_msgs.msg import Header
import sensor_msgs.msg as sensor_msgs
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import Vector3, Pose, PoseWithCovariance, Point, Quaternion, Twist, TwistWithCovariance, TransformStamped
from nav_msgs.msg import Odometry
import time as pytime
import cv2

try:
    import pyrealsense2 as rs
    if not hasattr(rs, 'pipeline'):
        raise ImportError("pyrealsense2 namespace package without bindings")
except (ImportError, AttributeError):
    from pyrealsense2 import pyrealsense2 as rs

from rclpy.node import Node, QoSProfile

T265_SN = os.environ.get("UPMOON25_T265_SN", "").strip()
COV = 0.01
TRACKING_WIDTH = 848
TRACKING_HEIGHT = 800

X_OFFSET = 0.35      # How far forward the camera is from the base
Z_OFFSET = 0.0     # How far up the camera is from the base
ROBOT_HEIGHT = 0.22 # How far the base_link is from the ground

'''
    Publishes the T265 pose data as an Odometry message and TF transform. 

    Publishes:
    /odom - Odometry, the pose and velocity of the robot in the odom frame
    odom -> base_link transform - the transform from the odom frame to the base_link frame
'''
class T265Driver(Node):

    def __init__(self):
        super().__init__('t265_driver')

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.PUB_odom = self.create_publisher(Odometry, '/odom', self.QOS)
        self.PUB_tracking = self.create_publisher(sensor_msgs.CompressedImage, '/camera/tracking/image_compressed', self.QOS)

        self.TF_odom = TransformBroadcaster(self)

        self.init_pose = None
        self.pipe = None

        while rclpy.ok():
            try:
                self._ensure_pipeline()
                self._stream_pose()
            except Exception as exc:
                self.get_logger().warn(f"T265 stream dropped: {exc}. Retrying...")
                self._reset_pipeline()
                pytime.sleep(1.0)

    def _ensure_pipeline(self):
        if self.pipe is not None:
            return
        device_sn = self._device_present()
        if not device_sn:
            raise RuntimeError("T265 device not detected")
        self.pipe = rs.pipeline()
        
        cfg = rs.config()
        cfg.enable_device(device_sn)
        cfg.enable_stream(rs.stream.pose)
        cfg.enable_stream(rs.stream.fisheye, 1, TRACKING_WIDTH, TRACKING_HEIGHT, rs.format.y8, 30)

        self.pipe.start(cfg)
        self.get_logger().info("T265 Initialized")

    def _device_present(self):
        try:
            ctx = rs.context()
            for dev in ctx.devices:
                try:
                    serial = dev.get_info(rs.camera_info.serial_number)
                    name = dev.get_info(rs.camera_info.name)
                except Exception:
                    continue
                upper_name = str(name).upper()
                if T265_SN:
                    if serial == T265_SN:
                        return serial
                    continue
                if "T265" in upper_name or "TRACKING" in upper_name:
                    return serial
        except Exception:
            pass
        return None

    def _reset_pipeline(self):
        if self.pipe is None:
            return
        try:
            self.pipe.stop()
        except Exception:
            pass
        self.pipe = None

    def _stream_pose(self):
        while rclpy.ok():

            frames = self.pipe.wait_for_frames()

            pose = frames.get_pose_frame()
            fisheye = frames.get_fisheye_frame(1)

            if not pose:
                continue

            time = self.get_clock().now().to_msg()
            self.publishOdom(pose, time)
            if fisheye:
                self.publishTrackingImage(fisheye, time)

    def publishOdom(self, pose, time):
        odom = Odometry()

        odom.header = Header(frame_id='odom', stamp=time)
        odom.child_frame_id = 'base_link'

        data = pose.get_pose_data()

        # Pose estimate
        # TODO: Fix covariance - no internal cov provided by T265 :(
        covariance = [
                COV, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, COV, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, COV, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, COV, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, COV, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, COV]
        
        # I know this is horrible, but for some reason I can't use the Point()
        # constructor to initialize these
        #
        # We also perform the transform from the optical frame to the actual camera frame here
        point = Point()
        point.x = 0.0 # -data.translation.z 
        point.y = 0.0 # -data.translation.x 
        point.z = 0.0 # data.translation.y 

        quat = Quaternion()
        quat.x = -data.rotation.z
        quat.y = -data.rotation.x
        quat.z = data.rotation.y
        quat.w = data.rotation.w

        # We now perform the offset calculation
        #np_quat = quaternion.from_float_array([quat.w, quat.x, quat.y, quat.z])
        #forward_vec = quaternion.rotate_vectors(np_quat, [1, 0, 0])
        #up_vec = quaternion.rotate_vectors(np_quat, [0, 0, 1])

        #forward_vec *= X_OFFSET
        #up_vec *= Z_OFFSET
        #point.x = point.x - forward_vec[0] - up_vec[0]
        #point.y = point.y - forward_vec[1] - up_vec[1]
        #point.z = point.z - forward_vec[2] - up_vec[2] + ROBOT_HEIGHT

        #if self.init_pose is None:
        #    self.init_pose = point
        #    return

        #point.x = point.x - self.init_pose.x
        #point.y = point.y - self.init_pose.y
        #point.z = point.z - self.init_pose.z

        #self.get_logger().info(f'ORIENTATION: {np_quat.w}, {np_quat.x}, {np_quat.y}, {np_quat.z}')

        odom_p = Pose()
        odom_p.position = point
        odom_p.orientation = quat

        odom.pose = PoseWithCovariance()
        odom.pose.pose = odom_p
        odom.pose.covariance = covariance

        odom.twist = TwistWithCovariance()
        odom_t = Twist()

        odom_t_vel = Vector3()
        odom_t_vel.x = -data.velocity.z #data.velocity.x
        odom_t_vel.y = -data.velocity.x #data.velocity.y
        odom_t_vel.z = data.velocity.y #data.velocity.z

        odom_t_ang = Vector3()
        odom_t_ang.x = -data.angular_velocity.z #data.angular_velocity.x
        odom_t_ang.y = -data.angular_velocity.x #data.angular_velocity.y
        odom_t_ang.z = data.angular_velocity.y #data.angular_velocity.z
       
        odom_t.linear = odom_t_vel
        odom_t.angular = odom_t_ang

        odom.twist.twist = odom_t
        odom.twist.covariance = covariance

        self.PUB_odom.publish(odom)

        t = TransformStamped()

        t.header.stamp = time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = 0.0 #point.x
        t.transform.translation.y = 0.0 #point.y
        t.transform.translation.z = 0.0 #point.z

        t.transform.rotation.x = quat.x
        t.transform.rotation.y = quat.y
        t.transform.rotation.z = quat.z
        t.transform.rotation.w = quat.w

        self.TF_odom.sendTransform(t)

    def publishTrackingImage(self, frame, time):
        try:
            img = np.asanyarray(frame.get_data())
            if img.size == 0:
                return
            success, jpeg = cv2.imencode('.jpg', img)
            if not success:
                return

            msg = sensor_msgs.CompressedImage()
            msg.header = Header(frame_id='tracking_camera_optical', stamp=time)
            msg.format = 'jpeg'
            msg.data = jpeg.tobytes()
            self.PUB_tracking.publish(msg)
        except Exception:
            pass



def main(args=None):
    rclpy.init()

    t265 = T265Driver()

    rclpy.spin(t265)

    t265.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
