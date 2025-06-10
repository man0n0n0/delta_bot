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
        self.operation_in_progress = False
        self.stage = 'idle'  # 'idle', 'stage1', 'stage2'
        
        # Store placement values between stages
        self.placing_x = None
        self.placing_y = None
        self.placing_z = None
        self.rock_x = None
        self.rock_y = None
        self.rock_z = None
        
        # Constants
        self.safe_height = 150
        self.height_correction = 50
        self.tool_offset = (0, 35, 20)
        
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

    def centers_callback(self, msg):
        """Process rock centers - handles both stage 1 and stage 2"""
        # Store the latest measurement for get_updated_z_measurement()
        self.new_measurement = msg
        
        if not msg.poses:
            self.get_logger().warn('No rocks detected')
            return

        if len(msg.poses) < 2:
            self.get_logger().warn('Only one rock detected')
            return

        # Stage 1: Get placement values and pick rock
        if self.stage == 'idle':
            self.stage1_callback(msg)
        # Stage 2: Get updated rock_z and complete placement
        elif self.stage == 'stage1':
            self.stage2_callback(msg)

    def stage1_callback(self, msg):
        self.get_logger().info('Starting Stage 1: Getting placement values and picking rock')
        self.stage = 'stage1'
        self.operation_in_progress = True
        
        # Get rock positions
        placing_rock = msg.poses[0]  # Highest rock (placement location)
        self.placing_x = placing_rock.position.x * 1000
        self.placing_y = placing_rock.position.y * 1000
        self.placing_z = placing_rock.position.z * 1000

        rock = msg.poses[1]  # Second highest rock (to pick)
        self.rock_x = rock.position.x * 1000
        self.rock_y = rock.position.y * 1000

        self.get_logger().info(f'Placement location: ({self.placing_x:.1f}, {self.placing_y:.1f}, {self.placing_z:.1f})')
        self.get_logger().info(f'Picking rock at: ({self.rock_x:.1f}, {self.rock_y:.1f})')

        # Pick rock sequence
        self.send_gcode('M5')  # Open gripper
        self.send_gcode('G91')
        self.send_gcode(f'G1 Z-{self.safe_height:.1f} F500')
        self.send_gcode('G90')
        self.send_gcode(f'G1 X{self.rock_x+self.tool_offset[0]:.1f} Y{self.rock_y+self.tool_offset[1]:.1f} F2000')
        time.sleep(10)

               
        self.get_logger().info('Stage 1 completed - rock picked and positioned over picking location')
        self.get_logger().info('Waiting for Stage 2 callback to get updated placement Z and complete drop...')

    def stage2_callback(self, msg):
        rock = msg.poses[0]  # Second mesure for rock picking (suppose that we pick the higher is the pool)
        self.rock_x = rock.position.x * 1000
        self.rock_y = rock.position.y * 1000
        self.rock_z = rock.position.z * 1000

        # # Correct the rock placement
        # self.get_logger().info('Correction the rock placement : moving to newly mesured')
        # self.send_gcode('G91')
        # self.send_gcode(f'G1 X{self.rock_x+self.tool_offset[0]:.1f} Y{self.rock_y+self.tool_offset[1]:.1f} F2000')

        # Complete picking sequence
        self.send_gcode('G91')
        pick_plunge = self.rock_z - self.height_correction - self.tool_offset[2]
        self.send_gcode(f'G1 Z-{pick_plunge:.1f} F500')
        self.send_gcode('G90')
        self.send_gcode('M4')  # Close gripper
        time.sleep(10)
        self.send_gcode('G91')
        self.send_gcode(f'G1 Z{pick_plunge:.1f} F500')  # Lift rock

        # Move over placement location
        self.send_gcode('G90')
        self.send_gcode(f'G1 X{self.placing_x+self.tool_offset[0]:.1f} Y{self.placing_y+self.tool_offset[1]:.1f} F1000')
        self.send_gcode('G91')
        self.send_gcode(f'G1 Z-{self.safe_height:.1f} F500')
        
        # Complete placement sequence
        drop_plunge = self.placing_z + self.safe_height - self.height_correction - self.tool_offset[2]
        self.send_gcode(f'G1 Z-{drop_plunge:.1f} F500')
        self.send_gcode('G90')
        self.send_gcode('M5')  # Open gripper
        time.sleep(10)
        
        # Return home
        self.send_gcode('G28')
        time.sleep(20)
        self.get_logger().info('Rock stacking completed - returning to idle state')
        
        # Reset for next operation
        self.stage = 'idle'
        self.operation_in_progress = False
        self.placing_x = None
        self.placing_y = None
        self.placing_z = None
        self.rock_x = None
        self.rock_y = None
        self.rock_z = None

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