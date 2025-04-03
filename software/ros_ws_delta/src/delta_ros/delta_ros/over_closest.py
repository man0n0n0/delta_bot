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
        self.declare_parameter('num_closest_points', 1000)  # Max number of points to consider
        self.declare_parameter('height_tolerance', 0.005)  # Tolerance for grouping points by height (in meters)
        self.declare_parameter('min_cluster_size', 50)  # Minimum number of points in a cluster
        self.declare_parameter('x_limit', (-150,150))  
        self.declare_parameter('y_limit', (-150,150))  
        self.declare_parameter('z_limit', (0,110))  

        # Get parameters
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.grbl_action_name = self.get_parameter('grbl_action_name').value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').value
        self.num_closest_points = self.get_parameter('num_closest_points').value
        self.height_tolerance = self.get_parameter('height_tolerance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.x_limit = self.get_parameter('x_limit').value
        self.y_limit = self.get_parameter('y_limit').value
        self.z_limit = self.get_parameter('z_limit').value

        # Subscribe to point cloud
        self.pointcloud_sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10)

        # Send homing command
        self.send_grbl_command("$G28")

        self.prev_target_point = [0, 0, 0]
        
        self.get_logger().info("Effector to closest dense zone node initialized!")

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

    def find_dense_zones(self, points_array, height_tolerance, min_cluster_size):
        """
        Find zones with at least min_cluster_size points at the same height (within tolerance)
        Returns a list of (center_point, num_points, distance_to_origin) for each zone
        """
        # Round z-coordinates to group by height within tolerance
        z_rounded = np.round(points_array[:, 2] / height_tolerance) * height_tolerance
        
        # Find unique heights and count points at each height
        unique_heights, indices, counts = np.unique(z_rounded, return_inverse=True, return_counts=True)
        
        dense_zones = []
        
        # For each height with enough points
        for i, height in enumerate(unique_heights):
            if counts[i] >= min_cluster_size:
                # Get all points at this height
                height_points = points_array[z_rounded == height]
                
                # Calculate centroid of these points
                centroid = np.mean(height_points, axis=0)
                
                # Calculate distance to origin
                distance = np.linalg.norm(centroid)
                
                dense_zones.append((centroid, counts[i], distance))
        
        return dense_zones

    def pointcloud_callback(self, pointcloud_msg):
        '''go home / take snapshot / go 10mm over closest dense zone'''
        
        self.send_grbl_command("$G28")
        time.sleep(5)

        # Convert point cloud to numpy array with explicit coordinate extraction
        points_list = []
        for point in pc2.read_points(pointcloud_msg, skip_nans=True):
            points_list.append((-1 * point[1], -1 * point[0], point[2]))

        total_points = len(points_list)
        
        self.get_logger().info(f"Total points: {total_points}")

        if total_points == 0:
            self.get_logger().warn("Empty point cloud received")
            return
         
        points_array = np.array(points_list, dtype=np.float64)
        
        # Get the closest N points to consider for processing
        distances = np.linalg.norm(points_array, axis=1)
        sorted_indices = np.argsort(distances)
        closest_points_indices = sorted_indices[:self.num_closest_points]
        closest_points = points_array[closest_points_indices]
        
        # Find dense zones in the closest points
        dense_zones = self.find_dense_zones(closest_points, self.height_tolerance, self.min_cluster_size)
        
        if not dense_zones:
            self.get_logger().warn("No dense zones found with at least 50 points at the same height")
            return
            
        # Sort dense zones by distance to origin
        dense_zones.sort(key=lambda x: x[2])
        
        # Get the closest dense zone
        closest_dense_zone = dense_zones[0]
        target_point = closest_dense_zone[0]
        point_count = closest_dense_zone[1]
        
        self.get_logger().info(f"Found closest dense zone with {point_count} points at height {target_point[2]:.4f}m")
        
        x_machine = target_point[0] * 1000.0  # Convert to mm 
        y_machine = target_point[1] * 1000.0  # Convert to mm 
        z_machine = 10 + (target_point[2] * 1000.0)  # Convert to mm + 10mm offset
        
        self.prev_target_point = target_point
        
        if self.x_limit[0] < x_machine < self.x_limit[1] and self.y_limit[0] < y_machine < self.y_limit[1] and self.z_limit[0] < z_machine < self.z_limit[1] :
            self.get_logger().info(f"Moving to dense zone at: X={x_machine:.0f}, Y={y_machine:.0f}, Z={z_machine:.0f}")
            self.send_grbl_command(f"$G1X{x_machine:.0f}Y{y_machine:.0f}Z{z_machine:.0f}F6000")                
            time.sleep(5)
        
        else :
            self.get_logger().info(f"Dected cluster out of the machine physical limits")
            return


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