# Rock Stacking

This ROS2 package subscribes to rock center positions published by the `rock_detector_node` and generates G-code commands to pick up the closest rock and place it in the center position. It enables a robotic system to perform rock stacking operations.

## Features

- Subscribes to `rock_centers` topic (PoseArray) from the rock detector
- Identifies the closest rock to a defined center position
- Enforces robot physical limits for X, Y, and Z axes
- Generates G-code commands for a pick-and-place operation
- Saves G-code to a file and publishes commands to a ROS2 topic
- Configurable parameters for positions, heights, and G-code settings

## Dependencies

- ROS2 (tested with jazzy)
- geometry_msgs
- std_msgs

## Building the Package

1. Create a ROS2 workspace (if you don't have one already):
   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone or copy this package into the `src` directory
   
3. Build the workspace:
   ```
   cd ~/ros2_ws
   colcon build --packages-select rock_stacking
   ```

4. Source the workspace:
   ```
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Running the Node

Launch the node with default parameters:
```
ros2 launch rock_stacking rock_stacking_launch.py
```

With custom parameters including robot limits:
```
ros2 launch rock_stacking rock_stacking_launch.py center_x:=0.1 center_y:=0.2 approach_height:=0.07 x_min:=-80.0 x_max:=80.0 z_max:=120.0
```

### Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| center_x | X coordinate of center position | 0.0 |
| center_y | Y coordinate of center position | 0.0 |
| center_z | Z coordinate of center position | 0.1 |
| approach_height | Height for approaching rocks | 0.05 |
| pick_height | Height for picking rocks | 0.01 |
| feedrate | Feedrate for G-code movements | 1000 |
| gcode_file | Output G-code filename | rock_pick_place.gcode |
| gripper_open_command | G-code command to open gripper | M8 |
| gripper_close_command | G-code command to close gripper | M9 |
| x_min | Minimum X coordinate (robot limit) | -60.0 |
| x_max | Maximum X coordinate (robot limit) | 60.0 |
| y_min | Minimum Y coordinate (robot limit) | -60.0 |
| y_max | Maximum Y coordinate (robot limit) | 60.0 |
| z_min | Minimum Z coordinate (robot limit) | 0.0 |
| z_max | Maximum Z coordinate (robot limit) | 100.0 |

### Published Topics

- **gcode_commands** (std_msgs/String): G-code commands for the pick-and-place operation

### Subscribed Topics

- **rock_centers** (geometry_msgs/PoseArray): Centers of detected rocks from the rock detector node

## G-code Format

The generated G-code includes:
- Setting units to millimeters (G21)
- Using absolute positioning (G90)
- Commands to approach the rock from above
- Gripper open/close commands (M8/M9 by default)
- Movement to the center position
- Return to home position
- All positions are constrained to stay within the robot's physical limits

## Safety Features

- All coordinates are checked against the robot's physical limits
- Movements outside the allowed range are clipped to safe values
- Warning messages are logged when rocks or target positions are outside limits
- G-code includes comments documenting the robot's physical constraints

## Integration with Rock Detector

This package is designed to work with the `rock_detector_node` which should publish rock center positions to the `rock_centers` topic.

## Notes

- The G-code is scaled from meters (ROS2 standard) to millimeters (G-code standard)
- The gripper commands (M8/M9) may need to be adjusted based on your hardware
- Ensure your machine understands the generated G-code format before executing it