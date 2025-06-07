
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import math
import time
import threading


class RockStacking(Node):
    def __init__(self):
        super().__init__('rock_stacking')
        
        # Declare parameters with default values
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('center_z', 0.1)  # Height above table in meters
        self.declare_parameter('approach_height', 0.2)  # Approach height in meters
        self.declare_parameter('pick_height', 0.01)  # Pick height offset in meters
        self.declare_parameter('feedrate', 1000)
        self.declare_parameter('gripper_open_command', 'M8')
        self.declare_parameter('gripper_close_command', 'M9')
        self.declare_parameter('homing_command', 'G28')
        self.declare_parameter('pick_wait_time', 3)
        self.declare_parameter('home_wait_time', 10)
        self.declare_parameter('machine_id', 'delta_marlin')
        
        # Physical limits in meters
        self.declare_parameter('x_min', -0.06)
        self.declare_parameter('x_max', 0.06)
        self.declare_parameter('y_min', -0.06)
        self.declare_parameter('y_max', 0.06)
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('z_max', 0.1)
        
        # Get machine ID for topic name
        self.machine_id = self.get_parameter('machine_id').get_parameter_value().string_value
        
        # Subscribe to rock centers
        self.subscription = self.create_subscription(
            PoseArray,
            'rock_centers',
            self.centers_callback,
            10
        )
        
        # Publisher for G-code commands
        gcode_topic = f'/{self.machine_id}/gcode'
        self.gcode_publisher = self.create_publisher(String, gcode_topic, 10)
        
        # Processing flag
        self.processing_rock = False
        
        # Send homing command at startup
        self.send_homing_command()
        
        self.get_logger().info('Rock Stacking node has started')
        self.get_logger().info(f'Publishing G-code to {gcode_topic}')
    
    def send_homing_command(self):
        """Send homing command at startup"""
        homing_command = self.get_parameter('homing_command').get_parameter_value().string_value
        
        self.get_logger().info(f'Sending homing command: {homing_command}')
        
        msg = String()
        msg.data = homing_command
        self.gcode_publisher.publish(msg)
        
        # Wait for homing to complete
        time.sleep(5)
        
        self.get_logger().info('Homing completed. Ready to process rocks.')
        self.processing_rock = False
    
    def centers_callback(self, msg):
        """Callback for rock centers detection"""
        # Skip if already processing
        if self.processing_rock:
            self.get_logger().info('Already processing a rock, skipping new detection')
            return
            
        if not msg.poses:
            self.get_logger().warn('Received empty PoseArray, no rocks detected')
            return
        
        self.processing_rock = True
        self.get_logger().info('Starting new rock processing sequence')
        
        # Start processing in a separate thread to avoid blocking
        processing_thread = threading.Thread(target=self.process_rock_sequence, args=(msg,))
        processing_thread.start()
    
    def process_rock_sequence(self, msg):
        """Process the rock picking and placing sequence"""
        try:
            # Get parameters
            center_x = self.get_parameter('center_x').get_parameter_value().double_value
            center_y = self.get_parameter('center_y').get_parameter_value().double_value
            center_z = self.get_parameter('center_z').get_parameter_value().double_value
            approach_height = self.get_parameter('approach_height').get_parameter_value().double_value
            pick_height = self.get_parameter('pick_height').get_parameter_value().double_value
            feedrate = self.get_parameter('feedrate').get_parameter_value().integer_value
            gripper_open = self.get_parameter('gripper_open_command').get_parameter_value().string_value
            gripper_close = self.get_parameter('gripper_close_command').get_parameter_value().string_value
            pick_wait_time = self.get_parameter('pick_wait_time').get_parameter_value().integer_value
            home_wait_time = self.get_parameter('home_wait_time').get_parameter_value().integer_value
            
            # Get physical limits
            x_min = self.get_parameter('x_min').get_parameter_value().double_value
            x_max = self.get_parameter('x_max').get_parameter_value().double_value
            y_min = self.get_parameter('y_min').get_parameter_value().double_value
            y_max = self.get_parameter('y_max').get_parameter_value().double_value
            z_min = self.get_parameter('z_min').get_parameter_value().double_value
            z_max = self.get_parameter('z_max').get_parameter_value().double_value
            
            # Find the closest rock to center position
            closest_index = self.find_closest_rock(msg)
            rock_pose = msg.poses[closest_index]
            
            rock_x = rock_pose.position.x
            rock_y = rock_pose.position.y
            rock_z = rock_pose.position.z
            
            # Convert to millimeters for G-code
            x_machine = rock_x * 1000.0
            y_machine = rock_y * 1000.0
            z_machine = rock_z * 1000.0
            
            # Check limits
            if not self.check_position_limits(x_machine/1000.0, y_machine/1000.0, x_min, x_max, y_min, y_max):
                self.get_logger().warn(f'Rock position ({x_machine:.3f}, {y_machine:.3f}) is outside robot limits')
                self.processing_rock = False
                return
            
            center_x_mm = center_x * 1000.0
            center_y_mm = center_y * 1000.0
            center_z_mm = center_z * 1000.0
            
            if not self.check_position_limits(center_x, center_y, x_min, x_max, y_min, y_max) or \
               center_z < z_min or center_z > z_max:
                self.get_logger().warn(f'Center position ({center_x_mm:.3f}, {center_y_mm:.3f}, {center_z_mm:.3f}) is outside robot limits')
                self.processing_rock = False
                return
            
            # Generate and execute G-code sequence
            gcode_commands = self.generate_gcode_sequence(
                x_machine, y_machine, z_machine,
                center_x_mm, center_y_mm, center_z_mm,
                approach_height * 1000.0, pick_height * 1000.0,
                feedrate, gripper_open, gripper_close,
                pick_wait_time, home_wait_time
            )
            
            self.execute_gcode_sequence(gcode_commands)
            
        except Exception as e:
            self.get_logger().error(f'Error in rock processing sequence: {str(e)}')
        finally:
            self.processing_rock = False
    
    def find_closest_rock(self, msg):
        """Find the closest rock to the center position"""
        center_x = self.get_parameter('center_x').get_parameter_value().double_value
        center_y = self.get_parameter('center_y').get_parameter_value().double_value
        
        closest_index = 0
        min_distance = float('inf')
        
        for i, pose in enumerate(msg.poses):
            dx = pose.position.x - center_x
            dy = pose.position.y - center_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        self.get_logger().info(
            f'Closest rock found at ({msg.poses[closest_index].position.x:.3f}, '
            f'{msg.poses[closest_index].position.y:.3f}, '
            f'{msg.poses[closest_index].position.z:.3f}), distance: {min_distance:.3f}'
        )
        
        return closest_index
    
    def check_position_limits(self, x, y, x_min, x_max, y_min, y_max):
        """Check if position is within robot limits"""
        return x_min <= x <= x_max and y_min <= y <= y_max
    
    def generate_gcode_sequence(self, rock_x, rock_y, rock_z, center_x, center_y, center_z,
                              approach_height, pick_height, feedrate, gripper_open, gripper_close,
                              pick_wait_time, home_wait_time):
        """Generate G-code sequence for pick and place operation"""
        commands = []
        
        # Preamble
        commands.extend([
            "; Rock pick and place operation",
            "; Generated by delta_ros rock_stacking node",
            "G21 ; Set units to millimeters",
            "G90 ; Use absolute positioning"
        ])
        
        # Step 1: Open gripper
        commands.append(f"{gripper_open} ; Open gripper")
        
        # Step 2: Move to approach height
        commands.append(f"G0 Z{approach_height:.1f} F{feedrate} ; Move to safe height")
        
        # Step 3: Move above rock
        commands.append(f"G0 X{rock_x:.1f} Y{rock_y:.1f} F{feedrate} ; Move above rock")
        self.get_logger().info(f'Moving to closest rock at ({rock_x:.1f}, {rock_y:.1f})')
        
        # Step 4: Move down to pick (use rock_z + pick_height for pick position)
        pick_z = rock_z + pick_height
        commands.append(f"G1 Z{pick_z:.1f} F{feedrate//2} ; Move down to pick")
        
        # Step 5: Close gripper
        commands.append(f"{gripper_close} ; Close gripper to grab rock")
        
        # Step 6: Wait for secure grasp
        commands.append(f"G4 P{pick_wait_time * 1000} ; Wait {pick_wait_time} seconds")
        self.get_logger().info(f'Picking rock, waiting {pick_wait_time} seconds to secure grasp')
        
        # Step 7: Move back up to safe height
        commands.append(f"G0 Z{approach_height:.1f} F{feedrate} ; Move up with rock")
        
        # Step 8: Move to center position
        commands.append(f"G0 X{center_x:.1f} Y{center_y:.1f} F{feedrate} ; Move to center")
        self.get_logger().info(f'Moving to center position ({center_x:.1f}, {center_y:.1f})')
        
        # Step 9: Move down to place
        place_z = center_z + pick_height
        commands.append(f"G1 Z{place_z:.1f} F{feedrate//2} ; Move down to place")
        
        # Step 10: Open gripper to release
        commands.append(f"{gripper_open} ; Open gripper to release rock")
        
        # Step 11: Move back up to safe height
        commands.append(f"G0 Z{approach_height:.1f} F{feedrate} ; Move back up")
        
        # Step 12: Return to home
        commands.append(f"{self.get_parameter('homing_command').get_parameter_value().string_value} ; Return home")
        self.get_logger().info('Returning to home position')
        
        # Step 13: Wait at home
        commands.append(f"G4 P{home_wait_time * 1000} ; Wait {home_wait_time} seconds at home")
        self.get_logger().info(f'Waiting at home position for {home_wait_time} seconds')
        
        return commands
    
    def execute_gcode_sequence(self, commands):
        """Execute G-code commands by publishing to the topic"""
        for cmd in commands:
            # Skip comments
            if cmd.startswith(';') or not cmd.strip():
                continue
            
            # Extract pure command without comments
            pure_cmd = cmd.split(';')[0].strip()
            if not pure_cmd:
                continue
            
            # Create and publish message
            msg = String()
            msg.data = pure_cmd
            self.gcode_publisher.publish(msg)
            
            self.get_logger().info(f'Sending G-code to /{self.machine_id}/gcode: {pure_cmd}')
            
            # Wait between commands
            time.sleep(0.1)
            
            # Additional wait for movement commands
            if pure_cmd.startswith(('G0', 'G1')):
                time.sleep(1.0)
            
            # Handle wait commands
            if pure_cmd.startswith('G4'):
                try:
                    # Extract wait time from P parameter (in milliseconds)
                    p_index = pure_cmd.find('P')
                    if p_index != -1:
                        wait_ms = int(pure_cmd[p_index+1:].split()[0])
                        time.sleep(wait_ms / 1000.0)
                except (ValueError, IndexError):
                    self.get_logger().warn(f'Failed to parse wait time from: {pure_cmd}')
        
        self.get_logger().info('Rock stacking sequence completed, ready for next rock')


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