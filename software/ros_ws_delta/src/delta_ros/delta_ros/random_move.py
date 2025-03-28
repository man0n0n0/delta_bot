#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import subprocess
import time

class EffectorRandomMovementNode(Node):
    def __init__(self):
        super().__init__('effector_random_movement')
        
        # Declare parameters
        self.declare_parameter('grbl_action_name', '/delta_marlin/send_gcode_cmd')
        self.declare_parameter('processing_rate', 4.0)  # How often to move (Hz)
        
        # Get parameters
        self.grbl_action_name = self.get_parameter('grbl_action_name').value
        self.processing_rate = self.get_parameter('processing_rate').value
        
        # Last processing time to control the rate
        self.last_process_time = self.get_clock().now()
        
        # Create a timer to periodically trigger random movements
        self.timer = self.create_timer(1.0 / self.processing_rate, self.perform_random_movement)
        
        # Send homing command
        self.send_grbl_command("$G28", waiting = True)

        self.get_logger().info("Effector random movement node initialized!")

    def send_grbl_command(self, command, waiting):
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

        # get machine info
        stdout, stderr = process.communicate()
        self.get_logger().info(f"{stdout}")

        if waiting :
            process.wait()
            if process.returncode != 0:
                self.get_logger().error(f"Error executing command: {stderr}")

    def perform_random_movement(self):
        # Define absolute max value per axis
        X = Y = 60
        Z = [-10, 90]

        # Generate and send random coordinates
        Xl = np.random.randint(-X, X)
        Yl = np.random.randint(-Y, Y)
        Zl = np.random.randint(Z[0], Z[1])

        # Print the random coordinate
        self.get_logger().info(f"Random coordinate: X={Xl}, Y={Yl}, Z={Zl}")
        
        # Send movement command
        self.send_grbl_command(f"$G1X{Xl}Y{Yl}Z{Zl}F6000", waiting = False)

def main(args=None):
    rclpy.init(args=args)
    node = EffectorRandomMovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()