import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class RockStacking(Node):
    def __init__(self):
        super().__init__('rock_stacking')
        
        # Subscribe to rock position as string (format: "x,y,z")
        self.subscription = self.create_subscription(
            String,
            'rock_position',
            self.position_callback,
            10
        )
        
        # Publisher for G-code commands
        self.gcode_publisher = self.create_publisher(String, '/delta_marlin/gcode', 10)
        
        # Let Marlin start
        time.sleep(2)

        # Send home command at startup
        self.send_gcode('G28')
        time.sleep(5)
        
        self.get_logger().info('Rock Stacking node started')
    
    def send_gcode(self, command):
        """Send G-code command"""
        msg = String()
        msg.data = command
        self.gcode_publisher.publish(msg)
        self.get_logger().info(f'Sent: {command}')
        time.sleep(0.1)
    
    def position_callback(self, msg):
        """Process rock position when received"""
        try:
            # Parse position string "x,y,z"
            coords = msg.data.split(',')
            if len(coords) != 3:
                self.get_logger().warn('Invalid position format. Expected: x,y,z')
                return
            
            rock_x = float(coords[0]) * 1000  # Convert to mm
            rock_y = float(coords[1]) * 1000
            rock_z = float(coords[2]) * 1000
            
            self.get_logger().info(f'Processing rock at ({rock_x:.1f}, {rock_y:.1f}, {rock_z:.1f})')
            
            # Pick and place sequence
            self.send_gcode('M8')  # Open gripper
            self.send_gcode('G0 Z200 F1000')  # Move to safe height
            self.send_gcode(f'G0 X{rock_x:.1f} Y{rock_y:.1f}')  # Move above rock
            self.send_gcode(f'G1 Z{rock_z + 10:.1f} F500')  # Move down to pick
            self.send_gcode('M9')  # Close gripper
            time.sleep(2)  # Wait for grip
            
            self.send_gcode('G0 Z200')  # Lift rock
            self.send_gcode('G0 X0 Y0')  # Move to center
            self.send_gcode('G1 Z110 F500')  # Move down to place
            self.send_gcode('M8')  # Open gripper
            self.send_gcode('G0 Z200')  # Move up
            self.send_gcode('G28')  # Return home
            
            self.get_logger().info('Rock stacking completed')
            
        except ValueError:
            self.get_logger().error('Failed to parse rock position coordinates')


def main(args=None):
    rclpy.init(args=args)
    node = RockStacking()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()