#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import math

def float_array_callback(msg, node):
    """
    Callback function for Float32MultiArray messages that visualizes the data as a colored matrix.
    
    Args:
        msg (std_msgs.msg.Float32MultiArray): The received float array message
        node: The ROS2 node (for logging)
    """
    try:
        # Get the data from the message
        data = msg.data
        data_length = len(data)
        
        if data_length == 0:
            print("Received empty array, nothing to display")
            return
            
        # Print basic information
        print(f"Received Float32MultiArray with {data_length} elements")
        
        # Determine the matrix dimensions
        # Try to make it as square as possible
        side = int(math.sqrt(data_length))
        if side * side < data_length:
            # If perfect square not possible, find best rectangle
            #width = side
            #height = math.ceil(data_length / width)
            width = 240
            height = 180
        else:
            width = height = side
            
        print(f"Visualizing as {width}x{height} matrix")
        
        # Create a numpy array and reshape it
        # If the data doesn't fill the matrix completely, pad with zeros
        padded_length = width * height
        padded_data = list(data) + [0] * (padded_length - data_length)
        matrix = np.array(padded_data, dtype=np.float32).reshape(height, width)
        
        # Normalize the data to 0-1 range for colormapping
        if np.min(matrix) != np.max(matrix):  # Avoid division by zero
            matrix_normalized = (matrix - np.min(matrix)) / (np.max(matrix) - np.min(matrix))
        else:
            matrix_normalized = np.zeros_like(matrix)
            
        # Convert to 8-bit for OpenCV
        matrix_8bit = (matrix_normalized * 255).astype(np.uint8)
        
        # Apply colormap (COLORMAP_JET, COLORMAP_VIRIDIS, COLORMAP_PLASMA, etc.)
        colored_matrix = cv2.applyColorMap(matrix_8bit, cv2.COLORMAP_JET)
        
        # Resize for better visualization (optional)
        scale_factor = max(1, min(800 // width, 600 // height))
        display_matrix = cv2.resize(colored_matrix, (width * scale_factor, height * scale_factor), 
                                   interpolation=cv2.INTER_NEAREST)
        
        # Add min/max values as text
        min_value = np.min(matrix)
        max_value = np.max(matrix)
        avg_value = np.mean(matrix)
        
        info_text = f"Min: {min_value:.3f}, Max: {max_value:.3f}, Avg: {avg_value:.3f}"
        cv2.putText(display_matrix, info_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Display the matrix
        cv2.imshow("Float32MultiArray Visualization", display_matrix)
        cv2.waitKey(1)  # Update the window, wait 1ms
            
    except Exception as e:
        print(f"Error processing message: {str(e)}")

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create a simple node
    node = rclpy.create_node('float_array_visualizer')
    
    # Create the subscription with the node as additional argument for logging
    node.create_subscription(
        Float32MultiArray,
        'depth_frame',
        lambda msg: float_array_callback(msg, node),
        10  # QoS profile depth
    )
    
    print("Float32MultiArray visualizer running on depth_frame topic. Press Ctrl+C to exit.")
    print("Waiting for messages...")
    
    try:
        # Spin the node to execute callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Clean up
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()