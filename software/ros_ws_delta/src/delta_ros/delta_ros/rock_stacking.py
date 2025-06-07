import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import time


class RockStacking(Node):
    def __init__(self):
        super().__init__('rock_stacking')
        
        # Subscribe to rock centers from your C++ detector
        self.subscription = self.create_subscription(
            PoseArray,
            'rock_centers',
            self.centers_callback,
            10
        )
        
        # Publisher for G-code commands
        self.gcode_publisher = self.create_publisher(String, '/delta_marlin/gcode', 10)
        
        # Send home command at startup
        time.sleep(5) # wait for marlin warming
        self.send_gcode('G28')
        time.sleep(10)
        
        self.get_logger().info('Rock Stacking node started')
    
    def send_gcode(self, command):
        """Send G-code command"""
        msg = String()
        msg.data = command
        self.gcode_publisher.publish(msg)
        self.get_logger().info(f'Sent: {command}')
        time.sleep(0.1)
    
    def centers_callback(self, msg):
        """Process rock centers when received from C++ detector"""
        if not msg.poses:
            self.get_logger().warn('No rocks detected')
            return
        
        # Get first rock position from PoseArray
        rock = msg.poses[0]
        rock_x = rock.position.x * 1000  # Convert to mm
        rock_y = rock.position.y * 1000
        rock_z = rock.position.z * 1000

        #init cairn height to zero
        cairn_height = 0
        safe_movement_height = 100 #from the oming pos
        height_correction = 0.001 * max(abs(rock_y),abs(rock_y)) #higher as the movmeent is away from center 
        tool_offset = (40,0,20) #in mm 
        
        self.get_logger().info(f'Processing rock at ({rock_x:.1f}, {rock_y:.1f}, {rock_z:.1f})')
        
        # Pick and place sequence
        self.send_gcode('M8')  # Open gripper

        self.send_gcode(f'G91')  # relative positioning
        self.send_gcode(f'G1 Z-{safe_movement_height:.1f} F500')  # Move to safe z heigh for x,y movement
        self.send_gcode(f'G90')  # absolute positioning

        self.send_gcode(f'G0 X{rock_x+tool_offset[0]:.1f} Y{rock_y+tool_offset[1]:.1f}')  # Move above rock

        self.send_gcode(f'G91')  # relative positioning
        self.send_gcode(f'G1 Z-{rock_z - safe_movement_height - height_correction - tool_offset[2]:.1f} F500')  # Move down to pick (with correction)
        self.send_gcode(f'G90')  # absolute positioning

        self.send_gcode('M9')  # Close gripper
        time.sleep(10)  # Wait for grip
        
        self.send_gcode('G1 Z200 F1000')  # Lift rock
        self.send_gcode('G0 X0 Y0')  # Move to center

        self.send_gcode(f'G91')  # relative positioning
        self.send_gcode(f'G1 Z-{rock_z - safe_movement_height - height_correction - tool_offset[2]:.1f} F500')  # Move down to drop (here implement the last_rock pos for cairn making)
        cairn_height += rock_z 
        self.send_gcode(f'G90')  # absolute positioning

        self.send_gcode('G1 Z110 F500')  # Move down to place
        self.send_gcode('M8')  # Open gripper
        self.send_gcode('G0 Z200')  # Move up
        time.sleep(10)  # Wait

        self.send_gcode('G28')
        time.sleep(20)
        
        self.get_logger().info(f'Rock stacking completed with a {cairn_height} mm cairn')


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