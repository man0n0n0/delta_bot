#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import sensor_msgs.msg
import subprocess
import struct
import sensor_msgs_py.point_cloud2 as pc2

class EffectorToClosestPointCloudNode(Node):
    def __init__(self):
        super().__init__('effector_to_closest_pointcloud')
        
        # Declare parameters
        self.declare_parameter('pointcloud_topic', '/point_cloud')
        self.declare_parameter('grbl_action_name', '/cnc_001/send_gcode_cmd')
        self.declare_parameter('min_distance_threshold', 0.1)  # Minimum distance in meters to trigger action
        self.declare_parameter('processing_rate', 1.0)  # How often to process point clouds (Hz)
        self.declare_parameter('num_closest_points', 100)  # Number of closest points to average
        
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
        
        self.get_logger().info("Effector to closest point cloud node initialized!")

    def pointcloud_callback(self, pointcloud_msg):
        # Control processing rate
        current_time = self.get_clock().now()
        if (current_time - self.last_process_time).nanoseconds / 1e9 < 1.0 / self.processing_rate:
            return
        self.last_process_time = current_time
        
        try:
            # Convert point cloud to numpy array with explicit coordinate extraction
            points_list = []
            for point in pc2.read_points(pointcloud_msg, skip_nans=True):
                # Explicitly extract x, y, z coordinates
                points_list.append((point[0], point[1], point[2]))
            
            # Log point cloud information
            total_points = len(points_list)
            self.get_logger().info(f"Total point cloud size: {total_points} points")
            
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
            average_distance = np.mean(distances[closest_points_indices])
            
            # Print the closest points information
            self.get_logger().info(f"Average point coordinates: X={average_point[0]:.4f}, Y={average_point[1]:.4f}, Z={average_point[2]:.4f}")
            print(f"Average distance: {average_distance:.4f} meters")
            
            # Only proceed if the average distance is within our threshold
            if average_distance > self.min_distance_threshold:
                print(f"Points are beyond threshold of {self.min_distance_threshold}m - not moving effector")
                return
            
            # Convert point coordinates to machine coordinates (mm)
            x_machine = average_point[0] * 1000.0  # Convert to mm
            y_machine = average_point[1] * 1000.0  # Convert to mm
            z_machine = average_point[2] * 1000.0  # Convert to mm
            
            # Print machine coordinates
            print(f"Machine coordinates: X={x_machine:.2f}mm, Y={y_machine:.2f}mm, Z={z_machine:.2f}mm")
            
            # Generate G-code to point to the average point
            gcode_command = f"G1 X{x_machine:.2f} Y{y_machine:.2f} Z{z_machine:.2f} F1000"
            
            print(f"G-code command: {gcode_command}")
            print("============================\n")
            
            self.get_logger().info(f"Sending GRBL command: {gcode_command}")
            
            # Use subprocess to call ros2 action send_goal command
            command = [
                "ros2", "action", "send_goal",
                self.grbl_action_name,
                "grbl_msgs/action/SendGcodeCmd",
                f"{{command: '{gcode_command}'}}"
            ]
            
            self.get_logger().info(f"Executing: {' '.join(command)}")
            
            # Execute the command asynchronously
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # Optional: Handle the process in a non-blocking way
            def check_process():
                return_code = process.poll()
                if return_code is not None:
                    stdout, stderr = process.communicate()
                    if return_code == 0:
                        self.get_logger().info(f"Command succeeded: {stdout.strip()}")
                    else:
                        self.get_logger().error(f"Command failed with code {return_code}: {stderr.strip()}")
                    return True
                return False
                
            # Add a timer to check the process (non-blocking)
            self.create_timer(0.1, lambda: check_process() and None)
            
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {str(e)}")
            import traceback
            traceback.print_exc()

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