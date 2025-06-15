import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String, Float64
import time
import math

class RockStacking(Node):
    def __init__(self):
        super().__init__('rock_stacking')
        # Subscribe to rock centers from your C++ detector
        self.plane_subscription = self.create_subscription(
            Float64,
            'plane_distance',  # Removed leading slash
            self.plane_distance_callback,
            10
        )

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
        self.plane_distance = None
        self.saved_plane_distance = None  # Plane distance saved at each centers callback
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
        self.safe_height = 100
        self.height_correction = 0
        self.tool_offset = (0,45,-20)
        self.approch_coeff = 1.2
        self.unloaded_speed = 5000

        # Debug: Log subscription info
        self.get_logger().info('Rock Stacking node initializing...')
        self.get_logger().info('Subscribed to: rock_centers and plane_distance')
        
        # Send home command at startup
        time.sleep(5)
        self.send_gcode('G28')
        self.send_gcode('G91')
        self.send_gcode(f'G0 Z-100')
        self.send_gcode('G90')
        time.sleep(30)
        self.get_logger().info('Rock Stacking node started')

    def send_gcode(self, command):
        """Send G-code command"""
        msg = String()
        msg.data = command
        self.gcode_publisher.publish(msg)
        self.get_logger().info(f'Sent: {command}')
        time.sleep(0.1)

    def plane_distance_callback(self, msg):
        """Process plane distance measurement"""
        self.plane_distance = msg.data
        self.get_logger().info(f'Received plane distance: {self.plane_distance:.3f}')

    def find_closest_to_center(self, poses):
        """Find the rock center closest to [0,0]"""
        if not poses:
            return None
        
        min_distance = float('inf')
        closest_rock = None
        closest_index = -1
        
        for i, pose in enumerate(poses):
            # Calculate distance from center [0,0]
            distance = math.sqrt(pose.position.x**2 + pose.position.y**2)
            
            if distance < min_distance:
                min_distance = distance
                closest_rock = pose
                closest_index = i
        
        self.get_logger().info(f'Selected rock {closest_index} as closest to center with distance: {min_distance:.3f}')
        return closest_rock

    def centers_callback(self, msg):
        """Process rock centers - handles both stage 1 and stage 2"""
        # Store the latest measurement for get_updated_z_measurement()
        self.new_measurement = msg
                
        if not msg.poses:
            self.get_logger().warn('No rocks detected')
            return

        # Stage 1: Get placement values and pick rock
        if self.stage == 'idle':

            if len(msg.poses) < 2:
                self.get_logger().warn('Only one rock detected')
                return

            # Save the current plane distance for use in calculations (do it at homing pose and before holding rock to have a complete view)
            self.saved_plane_distance = self.plane_distance * 1000
            
            if self.saved_plane_distance is not None:
                self.get_logger().info(f'Saved plane distance for this operation: {self.saved_plane_distance:.3f}')
            else:
                self.get_logger().error('No plane distance available! Check if plane_distance topic is publishing')

            self.stage1_callback(msg)

        # Stage 2: intermediate rock compensation
        elif self.stage == 'stage1':
            self.stage2_callback(msg)

        # Stage 3: Get updated rock_z and complete placement
        elif self.stage == 'stage2':
            self.stage3_callback(msg)

    def stage1_callback(self, msg):
        self.get_logger().info('Starting Stage 1: Getting placement values and first approch')
        self.stage = 'stage1'
        self.operation_in_progress = True
        
        # Get rock positions
        placing_rock = msg.poses[0]  # Highest rock (placement location)
        self.placing_x = placing_rock.position.x * 1000
        self.placing_y = placing_rock.position.y * 1000
        self.placing_z = placing_rock.position.z * 1000

        rock = msg.poses[int(len(msg.poses)*0.8)-1]  # go toward the smallest
        self.rock_x = rock.position.x * 1000
        self.rock_y = rock.position.y * 1000
        self.rock_z = rock.position.z * 1000

        self.rock_first_approch = self.rock_z * 0.4
        self.corrected_y_tool_offset = self.tool_offset[1] + self.tool_offset[1] * (math.sqrt(self.rock_x**2 + self.rock_y**2)/100)

        # Correct the rock placement
        self.send_gcode('M5')  # Open gripper
        self.send_gcode('G91')
        self.send_gcode(f'G1 Z-{self.rock_first_approch:.1f} F{self.unloaded_speed}')
        self.send_gcode(f'G1 X{self.rock_x*self.approch_coeff:.1f} Y{self.rock_y*self.approch_coeff:.1f} F{self.unloaded_speed}')
        time.sleep(7)

    def stage2_callback(self, msg):
        self.get_logger().info('Starting Stage 2: Getting closer to the rock')
        self.stage = 'stage2'

        # Find the rock closest to center [0,0]
        rock = self.find_closest_to_center(msg.poses)
        if rock is None:
            self.get_logger().error('No rocks found in stage 2')
            return
            
        # # select the higher formation 
        # rock = msg.poses[0]

        # rock = msg.poses[len(msg.poses)-1]  # go toward the smallest

        self.rock_x = rock.position.x * 1000
        self.rock_y = rock.position.y * 1000
        self.rock_z = rock.position.z * 1000

        # Correct the rock placement
        self.get_logger().info('Correction the rock placement : moving to newly measured')
        self.send_gcode('G91')
        self.rock_second_approch = 0
        self.send_gcode(f'G1 X{self.rock_x*self.approch_coeff:.1f} Y{(self.rock_y*self.approch_coeff):.1f} Z-{self.rock_second_approch} F{self.unloaded_speed}')
        time.sleep(5)

    def stage3_callback(self, msg):
        self.get_logger().info('Starting Stage 3: Final positioning and rock placement')

        # Find the rock closest to center [0,0]
        rock = self.find_closest_to_center(msg.poses)
        if rock is None:
            self.get_logger().error('No rocks found in stage 3')
            return

        # # select the higher formation 
        # rock = msg.poses[0]

        # rock = msg.poses[len(msg.poses)-1]  # go toward the smallest

        self.rock_x = rock.position.x * 1000
        self.rock_y = rock.position.y * 1000
        self.rock_z = rock.position.z * 1000

        # Correct the rock placement
        self.get_logger().info('Correction the rock placement : moving to newly measured')
        self.send_gcode('G91')
        self.send_gcode(f'G1 X{self.rock_x*self.approch_coeff+self.tool_offset[0]:.1f} Y{(self.rock_y*self.approch_coeff)-self.corrected_y_tool_offset:.1f} F{self.unloaded_speed} ') #correctected offset

        # Complete picking sequence
        self.send_gcode('G91')
        pick_plunge = self.rock_z - self.height_correction + self.tool_offset[2]
        self.send_gcode(f'G1 Z-{pick_plunge:.1f} F{self.unloaded_speed}')
        self.send_gcode('G90')
        self.send_gcode('M4')  # Close gripper
        time.sleep(10)
        self.send_gcode('G91')
        self.send_gcode(f'G1 Z{pick_plunge + self.rock_second_approch + self.rock_first_approch - self.safe_height:.1f} F500')  # Lift rock

        # Move over placement location
        self.send_gcode('G90')
        self.send_gcode(f'G1 X{self.placing_x*self.approch_coeff+self.tool_offset[0]:.1f} Y{self.placing_y*self.approch_coeff-self.tool_offset[1]:.1f} F1000') #bring the piece back to the center of the tower
        self.send_gcode('G91')
        
        # Complete placement sequence - use saved plane distance for drop calculation
        if self.saved_plane_distance is not None:
            # Simplified drop calculation using plane distance
            drop_plunge = (self.placing_z - self.safe_height - (self.saved_plane_distance - self.safe_height - self.rock_z) ) #in mm
            self.get_logger().info(f'Drop plunge calculated: {drop_plunge:.1f}mm')
            self.get_logger().info(f'Using plane distance: {self.saved_plane_distance:.3f}mm, placing_z: {self.placing_z:.3f}mm')
            
        else:
            drop_plunge = 0
            self.get_logger().error('No plane distance available - cannot calculate drop! Leaving rock in air')
        
        self.send_gcode(f'G1 Z{drop_plunge * -1:.1f} F500') #reverted for proper logic
        self.send_gcode('G90')
        self.send_gcode('M5')  # Open gripper
        time.sleep(10)
        
        # Return home
        self.send_gcode('G28')
        self.send_gcode('G91')
        self.send_gcode(f'G0 Z-100')
        self.send_gcode('G90')
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
        self.saved_plane_distance = None

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