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
        self.declare_parameter('min_distance_threshold', 0.1)  # Minimum distance in meters to trigger action
        self.declare_parameter('max_distance_threshold', 0.3)  # maximum distance in meters to trigger action
        self.declare_parameter('processing_rate', 1.0)  # How often to process point clouds (Hz)
        self.declare_parameter('num_closest_points', 300)  # Number of closest points to average
        
        # Get parameters
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.grbl_action_name = self.get_parameter('grbl_action_name').value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').value
        self.max_distance_threshold = self.get_parameter('max_distance_threshold').value
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
        
        # Wait for the command to complete
        process.wait()
        stdout, stderr = process.communicate()
        
        if process.returncode != 0:
            self.get_logger().error(f"Error executing command: {stderr}")

    def pointcloud_callback(self, pointcloud_msg):
        # Control processing rate
        current_time = self.get_clock().now()
        if (current_time - self.last_process_time).nanoseconds / 1e9 < 1.0 / self.processing_rate:
            return
        self.last_process_time = current_time

        #Define absolute max value per axis
        X = Y = 60
        Z = [-10,90]

        try:
            Xl = np.random.randint(-X, X + 1)
            Yl = np.random.randint(-Y, Y)
            Zl = np.random.randint(Z[0],Z[1])
            Fl = np.random.randint(F_min,F_max)
            toSend = f"G1 X{Xl} Y{Yl} Z{Zl} F{Fl}\n"
            toSend = toSend.encode()
            s.write(toSend)
            print(toSend)

            # Print the closest points information
            self.get_logger().info(f"Random coordinate: X={Xl}, Y={Yl}, Z=-{Zl}")
            
            # Send movement command
            self.send_grbl_command(f"$G1X{Xl}Y{Yl}Z-{Zl}F6000")


            
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