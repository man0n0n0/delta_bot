#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import time

class EffectorToClosestNode(Node):
    def __init__(self):
        super().__init__('effector_to_closest')
        
        # Declare parameters
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('grbl_action_name', '/cnc_001/send_gcode_cmd')
        self.declare_parameter('min_distance_threshold', 0.1)  # Minimum distance in meters to trigger action
        self.declare_parameter('processing_rate', 1.0)  # How often to process depth frames (Hz)
        
        # Get parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        self.grbl_action_name = self.get_parameter('grbl_action_name').value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').value
        self.processing_rate = self.get_parameter('processing_rate').value
        
        # CV Bridge for converting ROS Image messages to OpenCV format
        self.bridge = CvBridge()
        
        # Subscribe to depth image
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10)
        
        # Last processing time to control the rate
        self.last_process_time = self.get_clock().now()
        
        self.get_logger().info("Effector to closest point node initialized!")

    def depth_callback(self, depth_msg):
        # Control processing rate
        current_time = self.get_clock().now()
        if (current_time - self.last_process_time).nanoseconds / 1e9 < 1.0 / self.processing_rate:
            return
        self.last_process_time = current_time
        
        try:
            # Convert ROS Image message to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            
            # Find the closest point in the depth map
            # Ignore NaN and zero values which often represent invalid measurements
            masked_depth = np.ma.masked_array(depth_image, mask=(np.isnan(depth_image) | (depth_image == 0)))
            
            if masked_depth.count() == 0:
                self.get_logger().warn("No valid depth data found")
                return
                
            # Find the minimum depth value and its coordinates
            min_depth = np.min(masked_depth)
            min_coords = np.where(masked_depth == min_depth)
            
            # Only proceed if the minimum depth is within our threshold
            if min_depth > self.min_distance_threshold:
                self.get_logger().info(f"Closest point at {min_depth:.3f}m, beyond threshold of {self.min_distance_threshold}m")
                return
            
            # Get the x, y coordinates of the closest point
            y, x = min_coords[0][0], min_coords[1][0]
            
            # Convert image coordinates to world coordinates
            # This is a simplification - you might need camera calibration parameters
            # to properly convert from image to world coordinates
            height, width = depth_image.shape
            x_normalized = (x - width/2) / (width/2)  # Range: -1 to 1
            y_normalized = (y - height/2) / (height/2)  # Range: -1 to 1
            
            # Scale normalized coordinates to machine coordinates (mm)
            # Adjust these scale factors to match your machine's working area
            x_machine = x_normalized * 100.0  # Scale -1,1 to -100,100 mm
            y_machine = y_normalized * 100.0  # Scale -1,1 to -100,100 mm
            z_machine = min_depth * 1000.0    # Convert meters to mm
            
            # Generate G-code to point to the closest zone
            # Format the command according to GRBL standards
            gcode_command = f"G1 X{x_machine:.2f} Y{y_machine:.2f} F1000"
            
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
            self.get_logger().error(f"Error processing depth image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = EffectorToClosestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()