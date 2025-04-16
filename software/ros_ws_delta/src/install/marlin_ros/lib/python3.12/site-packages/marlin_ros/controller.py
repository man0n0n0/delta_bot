#!/usr/bin/env python3
# Fast and lightweight controller for Marlin-based devices

import serial
import threading
import queue
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger

# We'll use a simple string action for gcode commands
from example_interfaces.action import ExecuteGcode

class MarlinController(Node):
    """
    Lightweight Marlin controller for ROS2 with minimal verification.
    Optimized for high-frequency command sending.
    """
    
    def __init__(self):
        super().__init__('marlin_controller')
        
        # Declare parameters with defaults
        self.declare_parameter('machine_id', 'delta_marlin')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 250000)
        self.declare_parameter('buffer_size', 128)
        
        # Get parameters
        self.machine_id = self.get_parameter('machine_id').value
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.buffer_size = self.get_parameter('buffer_size').value
        
        self.get_logger().info(f"Starting Marlin controller with port: {self.port}")
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,  # Non-blocking reads
                write_timeout=0.1  # Non-blocking writes
            )
            self.get_logger().info(f"Connected to Marlin device on {self.port}")
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Marlin device: {str(e)}")
            self.serial = None
        
        # Command queue - simple FIFO
        self.cmd_queue = queue.Queue()
        
        # Publishers
        self.response_pub = self.create_publisher(
            String, 
            f'/{self.machine_id}/response', 
            10
        )
        
        # Services
        self.emergency_stop_srv = self.create_service(
            Trigger, 
            f'/{self.machine_id}/emergency_stop', 
            self.emergency_stop_callback
        )
        
        # Action server for sending G-code
        self.send_gcode_action = ActionServer(
            self,
            ExecuteGcode,
            f'/{self.machine_id}/send_gcode',
            self.send_gcode_callback
        )
        
        # Background threads
        self.running = True
        self.send_thread = threading.Thread(target=self._send_command_loop)
        self.read_thread = threading.Thread(target=self._read_response_loop)
        
        # Start threads
        self.send_thread.daemon = True
        self.read_thread.daemon = True
        self.send_thread.start()
        self.read_thread.start()
        
        self.get_logger().info("Marlin controller initialized and ready for commands")

    def send_gcode_callback(self, goal_handle):
        """Action callback for sending G-code commands"""
        cmd = goal_handle.request.command
        self.get_logger().debug(f"Received G-code command: {cmd}")
        
        # Simply queue the command without waiting for completion
        self.cmd_queue.put(cmd)
        
        # Immediately return success - we're not waiting for verification
        result = ExecuteGcode.Result()
        result.success = True
        goal_handle.succeed()
        return result

    def emergency_stop_callback(self, request, response):
        """Emergency stop service callback"""
        self.get_logger().info("Emergency stop triggered")
        
        # Clear command queue
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break
        
        # Send stop command
        if self.serial and self.serial.is_open:
            # Send reset command (Ctrl+X)
            self.serial.write(b'\x18')  
        
        response.success = True
        response.message = "Emergency stop executed"
        return response

    def _send_command_loop(self):
        """Background thread for sending commands from queue"""
        while self.running:
            try:
                if not self.cmd_queue.empty() and self.serial and self.serial.is_open:
                    cmd = self.cmd_queue.get_nowait()
                    
                    # Don't block on write - if it fails, that's ok
                    try:
                        cmd_bytes = (cmd + '\n').encode('ascii')
                        self.serial.write(cmd_bytes)
                        self.get_logger().debug(f"Sent: {cmd}")
                    except Exception as e:
                        self.get_logger().error(f"Error sending command {cmd}: {str(e)}")
                    
                    # Small sleep to prevent CPU hogging
                    time.sleep(0.001)
                else:
                    # If queue is empty, sleep a bit longer
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Error in send loop: {str(e)}")
                time.sleep(0.1)

    def _read_response_loop(self):
        """Background thread for reading responses"""
        while self.running:
            try:
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    response = self.serial.readline().decode('ascii', errors='ignore').strip()
                    
                    if response:
                        # Publish raw response
                        msg = String()
                        msg.data = response
                        self.response_pub.publish(msg)
                        
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Error in read loop: {str(e)}")
                time.sleep(0.1)

    def shutdown(self):
        """Clean shutdown of the controller"""
        self.get_logger().info("Shutting down Marlin controller")
        self.running = False
        
        # Wait for threads to finish
        if self.send_thread.is_alive():
            self.send_thread.join(timeout=1.0)
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        # Close serial connection
        if self.serial and self.serial.is_open:
            self.serial.close()

def main(args=None):
    rclpy.init(args=args)
    controller = MarlinController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()