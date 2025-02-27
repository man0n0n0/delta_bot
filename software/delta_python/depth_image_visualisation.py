#!/usr/bin/env python3

import rclpy
import cv2
from std_msgs.msg import Float32MultiArray

def float_array_callback(msg):
    """
    Simple callback function for Float32MultiArray messages.
    
    Args:
        msg (std_msgs.msg.Float32MultiArray): The received float array message
    """
    try:
        # Print basic information about the received array
        data_length = len(msg.data)
        print(f"Received Float32MultiArray on depth_frame topic:")
        print(f"  Array length: {data_length} elements")
        
        # Print the first few elements if there are any
        if data_length > 0:
            preview_size = min(5, data_length)
            preview = [f"{value:.3f}" for value in msg.data[:preview_size]]
            print(f"  First {preview_size} elements: {', '.join(preview)}")
            
            # Calculate basic statistics
            min_val = min(msg.data)
            max_val = max(msg.data)
            avg_val = sum(msg.data) / data_length
            
            print(f"  Min value: {min_val:.3f}")
            print(f"  Max value: {max_val:.3f}")
            print(f"  Average value: {avg_val:.3f}")
        else:
            print("  Array is empty")
            
    except Exception as e:
        print(f"Error processing message: {str(e)}")

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create a simple node
    node = rclpy.create_node('float_array_listener')
    
    # Create the subscription
    node.create_subscription(
        Float32MultiArray,
        'depth_frame',
        float_array_callback,
        10  # QoS profile depth
    )
    
    print("Float32MultiArray listener running on depth_frame topic. Press Ctrl+C to exit.")
    
    try:
        # Spin the node to execute callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()