import rclpy
import numpy as np
import rclpy.timer
import sensor_msgs.msg as sensor_msgs
import time as pytime
import cv2

try:
    import pyrealsense2 as rs
    if not hasattr(rs, 'pipeline'):
        raise ImportError("pyrealsense2 namespace package without bindings")
except (ImportError, AttributeError):
    from pyrealsense2 import pyrealsense2 as rs

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2


from rclpy.node import Node, QoSProfile
from std_msgs.msg import Int8

DEPTH_SN = '018322071045'
RGB_SN = '018322071465'
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
REAR_WIDTH = 640
REAR_HEIGHT = 480

'''
    This node handles the second D435 on the robot. It keeps the point-cloud path
    alive while also publishing a rear RGB operator feed from the same device.

    Parameters:
    demand_publish: bool, if True, the node will only publish when it receives a command on /cmd/pointcloud

    Subscriptions:
    /cmd/pointcloud - Int8, if 1, the node will publish the point cloud data, if 0, it will not publish. Will
                            only be used if demand_publish is True. 

    Publishes:
    /camera/depth/points - PointCloud2, the point cloud data from the second D435
    /camera/rear/image_raw - Image, the rear RGB image
    /camera/rear/image_compressed - CompressedImage, the rear RGB image
'''
class DepthDriver(Node):

    def __init__(self):
        super().__init__('depth_driver')

        self.declare_parameter('demand_publish', False)
        self.demand_publish = self.get_parameter('demand_publish').value

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.PUB_pc = self.create_publisher(PointCloud2, '/camera/depth/points', self.QOS)
        self.PUB_rear = self.create_publisher(sensor_msgs.Image, '/camera/rear/image_raw', self.QOS)
        self.PUB_rear_comp = self.create_publisher(sensor_msgs.CompressedImage, '/camera/rear/image_compressed', self.QOS)
        self.PUB_rear_info = self.create_publisher(sensor_msgs.CameraInfo, '/camera/rear/camera_info', self.QOS)

        # Filters for pointcloud data
        self.dec_filter = rs.decimation_filter()
        self.dec_filter.set_option(rs.option.filter_magnitude, 3)

        self.spat_filter = rs.spatial_filter()

        self.temp_filter = rs.temporal_filter()

        self.pc = rs.pointcloud()
        self.cam_info_msg = None

        self.pipe = None

        if self.demand_publish:
            self.create_subscription(Int8, '/cmd/pointcloud', self.onCmd, 1)
            return

        while rclpy.ok():
            try:
                self._ensure_pipeline()
                self.getFrame()
            except Exception as exc:
                self.get_logger().warn(f"Depth camera stream dropped: {exc}. Retrying...")
                self._reset_pipeline()
                pytime.sleep(1.0)

    def _ensure_pipeline(self):
        if self.pipe is not None:
            return

        depth_serial = self._resolve_depth_serial()
        if not depth_serial:
            raise RuntimeError("No dedicated D435 depth camera detected")

        self.pipe = rs.pipeline()

        cfg = rs.config()
        cfg.enable_device(depth_serial)
        cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, REAR_WIDTH, REAR_HEIGHT, rs.format.bgr8, 30)

        self.pipe.start(cfg)
        self.get_logger().info(f"Depth camera initialized ({depth_serial})")
        pipe_profile = self.pipe.get_active_profile()
        stamp = self.get_clock().now().to_msg()
        for stream_profile in pipe_profile.get_streams():
            if stream_profile.is_video_stream_profile():
                vs_p = stream_profile.as_video_stream_profile()
                if vs_p.stream_type() == rs.stream.color:
                    self.publishRearCamInfo(vs_p.intrinsics, stamp, 'rear_link_optical')

    def _resolve_depth_serial(self):
        try:
            ctx = rs.context()
        except Exception:
            return DEPTH_SN

        candidates = []
        for dev in ctx.devices:
            try:
                name = dev.get_info(rs.camera_info.name)
                serial = dev.get_info(rs.camera_info.serial_number)
            except Exception:
                continue
            if "D435" not in name:
                continue
            if serial == RGB_SN:
                continue
            candidates.append(serial)

        if DEPTH_SN in candidates:
            return DEPTH_SN
        if candidates:
            return candidates[0]
        return None

    def _reset_pipeline(self):
        if self.pipe is None:
            return
        try:
            self.pipe.stop()
        except Exception:
            pass
        self.pipe = None

    def onTimer(self):
        self.getFrame()

    def onCmd(self, msg):
        self.get_logger().info('Publishing')
        try:
            self._ensure_pipeline()
            self.getFrame()
        except Exception as exc:
            self.get_logger().warn(f"Depth camera command capture failed: {exc}. Retrying on next request...")
            self._reset_pipeline()

    def getFrame(self):
            frame = self.pipe.wait_for_frames()

            depth = frame.get_depth_frame()
            color = frame.get_color_frame()

            time = self.get_clock().now().to_msg()

            if color:
                self.publishRearImageCompressed(color, time)
                self.publishRearImageRaw(color, time)
                self.publishRearCamInfo(None, time, None)
            if not depth:
                return

            # Apply post-processing filters
            filtered = depth
            filtered = self.dec_filter.process(filtered)
            filtered = self.spat_filter.process(filtered)
            filtered = self.temp_filter.process(filtered)

            points = self.pc.calculate(filtered)
            vertices = np.array(points.get_vertices())
            
            x = vertices['f0']
            y = vertices['f1']
            z = vertices['f2']

            vertices = np.vstack((x, y, z)).T
            
            self.publishPC(vertices, time)    

    def publishPC(self, vertices, time):
        # Credit: https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo/blob/master/pcd_demo/pcd_publisher/pcd_publisher_node.py
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = vertices.astype(dtype).tobytes()

        fields = [sensor_msgs.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]
        
        header = Header(frame_id='depth_link_optical', stamp=time)

        msg = sensor_msgs.PointCloud2(
            header=header,
            height=1, 
            width=vertices.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * vertices.shape[0]),
            data=data
        )

        self.PUB_pc.publish(msg)

    def publishRearImageCompressed(self, frame, time):
        data = np.asanyarray(frame.get_data())
        success, jpeg_data = cv2.imencode('.jpg', data)
        if not success:
            return

        msg = sensor_msgs.CompressedImage()
        msg.header = Header(frame_id='rear_link_optical', stamp=time)
        msg.format = 'jpeg'
        msg.data = jpeg_data.tobytes()
        self.PUB_rear_comp.publish(msg)

    def publishRearImageRaw(self, frame, time):
        msg = sensor_msgs.Image()
        msg.header = Header(frame_id='rear_link_optical', stamp=time)
        msg.height = frame.get_height()
        msg.width = frame.get_width()
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = frame.get_stride_in_bytes()
        msg.data = np.asanyarray(frame.get_data()).tobytes()
        self.PUB_rear.publish(msg)

    def publishRearCamInfo(self, intrinsics, time, frame_id):
        if self.cam_info_msg is not None:
            self.cam_info_msg.header.stamp = time
            self.PUB_rear_info.publish(self.cam_info_msg)
            return

        if intrinsics is None:
            return

        msg = sensor_msgs.CameraInfo()
        msg.height = intrinsics.height
        msg.width = intrinsics.width
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.header = Header(frame_id=frame_id, stamp=time)

        self.cam_info_msg = msg
        self.PUB_rear_info.publish(msg)


def main(args=None):
    rclpy.init()
    
    dd = DepthDriver()

    rclpy.spin(dd)

    dd.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
