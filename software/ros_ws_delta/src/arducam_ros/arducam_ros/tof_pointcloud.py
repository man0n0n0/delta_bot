from argparse import ArgumentParser
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
import numpy as np
from threading import Thread
import sys
import math

from ArducamDepthCamera import (
    ArducamCamera,
    Connection,
    DeviceType,
    FrameType,
    Control,
    DepthData,
)

class Option:
    cfg: Optional[str]

class TOFPublisher(Node):
    """
    ArduCam Time-of-Flight Camera ROS2 Publisher
    
    Publishes point cloud data from an ArduCam ToF camera with the ability
    to rotate the point cloud to match the physical camera orientation.
    
    Author: Manoah Camporini <camporini@protonmail.com>
    """
    def __init__(self, options: Option):
        super().__init__("arducam")

        # Declare rotation parameter (in degrees)
        self.declare_parameter('rotation_angle', 0.0)
        
        # Get rotation parameter and convert to radians
        self.rotation_angle = self.get_parameter('rotation_angle').get_parameter_value().double_value
        self.rotation_rad = math.radians(self.rotation_angle)
        
        self.get_logger().info(f"Point cloud rotation set to {self.rotation_angle} degrees")

        tof = self.__init_camera(options)
        if tof is None:
            raise Exception("Failed to initialize camera")

        self.tof_ = tof
        self.pointsize_ = self.width_ * self.height_
        self.frame_id = "sensor_frame"
        self.depth_msg_ = Float32MultiArray()
        self.publisher_ = self.create_publisher(PointCloud2, "point_cloud", 10)
        self.publisher_depth_ = self.create_publisher(
            Float32MultiArray, "depth_frame", 10
        )
        self.fx = tof.getControl(Control.INTRINSIC_FX) / 100
        self.fy = tof.getControl(Control.INTRINSIC_FY) / 100
        self.header = Header()
        self.header.frame_id = "map"
        self.points = None
        self.running_ = True
        self.timer_ = self.create_timer(1 / 30, self.update)
        self.process_point_cloud_thr = Thread(
            target=self.__generateSensorPointCloud, daemon=True
        )
        self.process_point_cloud_thr.start()

    def __init_camera(self, options: Option):
        print("pointcloud publisher init")
        tof = ArducamCamera()
        ret = 0
        
        if options.cfg is not None:
            ret = tof.openWithFile(options.cfg, 0)
        else:
            ret = tof.open(Connection.CSI, 0)
        if ret != 0:
            print("Failed to open camera. Error code:", ret)
            return

        ret = tof.start(FrameType.DEPTH)
        if ret != 0:
            print("Failed to start camera. Error code:", ret)
            tof.close()
            return

        info = tof.getCameraInfo()
        if info.device_type == DeviceType.HQVGA:
            self.width_ = info.width
            self.height_ = info.height
            tof.setControl(Control.RANGE, 4)
        elif info.device_type == DeviceType.VGA:
            self.width_ = info.width
            self.height_ = info.height // 10 - 1
        print(f"Open camera success, width: {self.width_}, height: {self.height_}")

        print("Pointcloud publisher start")
        return tof

    def rotate_points(self, points):
        """
        Rotate the points around the Z axis by the specified angle in radians.
        
        Args:
            points: Numpy array of shape (N, 3) containing point cloud coordinates
            
        Returns:
            Rotated points
        """
        # Create rotation matrix around Z axis
        cos_theta = np.cos(self.rotation_rad)
        sin_theta = np.sin(self.rotation_rad)
        
        # Extract coordinates
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        
        # Apply rotation
        x_rotated = x * cos_theta - y * sin_theta
        y_rotated = x * sin_theta + y * cos_theta
        
        # Construct rotated points
        rotated_points = np.column_stack((x_rotated, y_rotated, z))
        
        return rotated_points

    def __generateSensorPointCloud(self):
        while self.running_:
            frame = self.tof_.requestFrame(200)
            if frame is not None and isinstance(frame, DepthData):
                self.fx = self.tof_.getControl(Control.INTRINSIC_FX) / 100
                self.fy = self.tof_.getControl(Control.INTRINSIC_FY) / 100
                depth_buf = frame.depth_data
                confidence_buf = frame.confidence_data

                depth_buf[confidence_buf < 30] = 0

                self.depth_msg_.data = depth_buf.flatten() / 1000

                # Convert depth values from millimeters to meters
                z = depth_buf / 1000.0
                z[z <= 0] = np.nan  # Handling invalid depth values

                # Calculate x and y coordinates
                u = np.arange(self.width_)
                v = np.arange(self.height_)
                u, v = np.meshgrid(u, v)

                # Calculate point cloud coordinates
                x = (u - self.width_ / 2) * z / self.fx
                y = (v - self.height_ / 2) * z / self.fy

                # Combined point cloud
                points = np.stack((x, y, z), axis=-1)
                valid_points = points[~np.isnan(points).any(axis=-1)]  # Filter invalid points
                
                # Apply rotation to valid points
                if len(valid_points) > 0:
                    self.points = self.rotate_points(valid_points)
                else:
                    self.points = valid_points

                self.tof_.releaseFrame(frame)

    def update(self):
        if self.points is None:
            return
        self.header.stamp = self.get_clock().now().to_msg()

        pc2_msg_ = point_cloud2.create_cloud_xyz32(self.header, self.points)

        self.publisher_.publish(pc2_msg_)
        self.publisher_depth_.publish(self.depth_msg_)

    def stop(self):
        self.running_ = False
        self.process_point_cloud_thr.join()
        self.tof_.stop()
        self.tof_.close()

def main(args=None):
    # Initialize rclpy with all arguments
    rclpy.init(args=args)
    
    # Create argument parser for non-ROS arguments
    parser = ArgumentParser()
    parser.add_argument("--cfg", type=str, help="Path to camera configuration file")
    
    # Parse only known arguments and ignore the rest (including ROS args)
    ns, remaining = parser.parse_known_args(args=sys.argv[1:] if args is None else args[1:])
    
    # Create options
    options = Option()
    options.cfg = ns.cfg
    
    # Create and spin the node
    tof_publisher = TOFPublisher(options)

    try:
        rclpy.spin(tof_publisher)
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down...")
    finally:
        # Clean up
        tof_publisher.stop()
        rclpy.shutdown()

if __name__ == "__main__":
    main()