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
        
        # Variable to store new measurements
        self.new_measurement = None
        
        # Send home command at startup
        time.sleep(5)
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

    def get_updated_z_measurement(self):
        """Wait for one new measurement and return the Z height"""
        self.new_measurement = None
        start_time = time.time()
        
        while self.new_measurement is None and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.new_measurement and self.new_measurement.poses:
            return self.new_measurement.poses[0].position.z * 1000  # Convert to mm
        return None

    def centers_callback(self, msg):
        """Process rock centers when received from C++ detector"""
        # Store the latest measurement for get_updated_z_measurement()
        self.new_measurement = msg
        
        if not msg.poses:
            self.get_logger().warn('No rocks detected')
            return

        # Get rock positions
        placing_rock = msg.poses[0]  # Highest rock (placement location)
        placing_x = placing_rock.position.x * 1000
        placing_y = placing_rock.position.y * 1000
        placing_z = placing_rock.position.z * 1000

        rock = msg.poses[1]  # Second highest rock (to pick)
        rock_x = rock.position.x * 1000
        rock_y = rock.position.y * 1000
        rock_z = rock.position.z * 1000

        # Constants
        safe_height = 200
        height_correction = 50
        tool_offset = (0, 35, 20)

        self.get_logger().info(f'Picking rock at ({rock_x:.1f}, {rock_y:.1f}, {rock_z:.1f})')

        # Pick rock with updated Z measurement
        self.send_gcode('M5')  # Open gripper
        self.send_gcode('G91')
        self.send_gcode(f'G1 Z-{safe_height:.1f} F500')
        self.send_gcode('G90')
        self.send_gcode(f'G1 X{rock_x+tool_offset[0]:.1f} Y{rock_y+tool_offset[1]:.1f} F2000')
        
        # Get updated Z measurement for picking
        self.get_logger().info('Getting updated Z measurement for picking...')
        updated_pick_z = self.get_updated_z_measurement()
        if updated_pick_z:
            rock_z = updated_pick_z
            self.get_logger().info(f'Updated pick height: {rock_z:.1f} mm')
        else:
            self.get_logger().warn('Using original pick Z measurement')
        time.sleep(10)

        self.send_gcode('G91')
        pick_plunge = rock_z - safe_height - height_correction - tool_offset[2]
        self.send_gcode(f'G1 Z-{pick_plunge:.1f} F500')
        self.send_gcode('M4')  # Close gripper
        time.sleep(10)
        self.send_gcode(f'G1 Z{pick_plunge:.1f} F500')  # Lift rock
        self.send_gcode('G90')

        # Move over placement location
        self.send_gcode(f'G1 X{placing_x+tool_offset[0]:.1f} Y{placing_y+tool_offset[1]:.1f} F1000')

        # Get updated Z measurement while over placement rock
        self.get_logger().info('Getting updated Z measurement for placement...')
        updated_placement_z = self.get_updated_z_measurement()
        if updated_placement_z:
            placing_z = updated_placement_z
            self.get_logger().info(f'Updated placement height: {placing_z:.1f} mm')
        else:
            self.get_logger().warn('Using original placement Z measurement')

        # Drop rock using variable z for second plunge
        drop_plunge = placing_z - pick_plunge
        self.send_gcode('G91')
        self.send_gcode(f'G1 Z-{drop_plunge:.1f} F500')
        self.send_gcode('G90')
        self.send_gcode('M5')  # Open gripper
        time.sleep(10)
        
        # Return home
        self.send_gcode('G28')
        time.sleep(20)
        self.get_logger().info('Rock stacking completed')

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