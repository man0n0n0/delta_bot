#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import sensor_msgs.msg
import subprocess
import struct
import time
import sensor_msgs_py.point_cloud2 as pc2

class EffectorToClosestPointCloudNode(Node):
    def __init__(self):
        super().__init__('effector_to_closest_pointcloud')
        
        # Declare parameters
        self.declare_parameter('pointcloud_topic', '/point_cloud')
        self.declare_parameter('grbl_action_name', '/delta_marlin/send_gcode_cmd')
        self.declare_parameter('min_distance_threshold', 0.005)  # Minimum distance in meters to trigger action
        self.declare_parameter('processing_rate', 1.0)  # How often to process point clouds (Hz)
        self.declare_parameter('num_closest_points', 300)  # Number of closest points to average
        self.declare_parameter('x_limit', )  # Number of closest points to average

        
        # Get parameters
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.grbl_action_name = self.get_parameter('grbl_action_name').value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').value
        self.processing_rate = self.get_parameter('processing_rate').value
        self.num_closest_points = self.get_parameter('num_closest_points').value

        # Subscribe to point cloud
        self.pointcloud_sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10)
       
        # Last processing time to control the rate
        self.last_process_time = self.get_clock().now()

        # Send homing command
        self.send_grbl_command("$G28")

        self.prev_average_point = [0,0,0]
        
        self.get_logger().info("Effector to closest point cloud node initialized!")

    def send_grbl_command(self, command):
        """Helper method to send GRBL commands via ros2 action"""
        ros2_command = [
            "ros2", "action", "send_goal",
            self.grbl_action_name,
            "grbl_msgs/action/SendGcodeCmd",
            f"{{command: {command}}}"
        ]
        
        process = subprocess.Popen(
            ros2_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        

    def pointcloud_callback(self, pointcloud_msg):
        # Convert point cloud to numpy array with explicit coordinate extraction
        points_list = []
        for point in pc2.read_points(pointcloud_msg, skip_nans=True):
            # Mirror the x-coordinate (change the sign)
            # This effectively flips the point cloud to adapt 
            # y x order inverted  
            points_list.append((-1 * point[1], -1 * point[0], point[2]))
        
        # Log point cloud information
        total_points = len(points_list)
                    
        # Check if point cloud is empty
        if total_points == 0:
            self.get_logger().warn("Empty point cloud received")
            return
        
        # Convert to numpy array of floats
        points_array = np.array(points_list, dtype=np.float64)
        
        # Calculate distances from origin
        distances = np.linalg.norm(points_array, axis=1)

        # Sort points by distance
        sorted_indices = np.argsort(distances)
        
        # Take the n closest points
        closest_points_indices = sorted_indices[:self.num_closest_points]
        closest_points = points_array[closest_points_indices]
        
        # Calculate the average of the closest points
        average_point = np.mean(closest_points, axis=0)
        
        # Calculate average distance of the closest points
        # average_distance = np.mean(distances[closest_points_indices])

        self.get_logger().info(f"{average_point}")

        # Convert point coordinates to machine coordinates (mm)
        x_machine = (self.prev_average_point[0] + average_point[0])* 1000.0 # Convert to mm 
        y_machine = (self.prev_average_point[1] + average_point[1]) * 1000.0 # Convert to mm 
        z_machine = 110

        self.prev_average_point = average_point 

        # Print the closest points information
        self.get_logger().info(f"Average point coordinates: X={x_machine:.0f}, Y={y_machine:.0f}, Z={z_machine:.0f}")
        
        # Send movement command
        self.send_grbl_command(f"$G1X{x_machine:.0f}Y{y_machine:.0f}Z{z_machine:.0f}F6000")


def main(args=None):
    rclpy.init(args=args)
    node = EffectorToClosestPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()