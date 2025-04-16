# marlin_ros

A fast, lightweight ROS2 package for controlling Marlin-based devices (delta robots, 3D printers, CNC machines).

## Features

- **High-speed communication**: Optimized for sending rapid commands without verification
- **Non-blocking operation**: Commands are queued and sent asynchronously
- **Minimal overhead**: Designed for maximum performance with minimal processing

## Installation

```bash
# Clone the package into your ROS2 workspace
cd ~/ros_ws/src
git clone <this-repository>

# Install dependencies
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -y

# Build the package
colcon build --packages-select marlin_ros

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the controller

```bash
ros2 launch marlin_ros marlin_controller.py
```

### Send G-code commands

This package now uses a standard topic-based approach for sending G-code commands:

```bash
# Send a G-code movement command
ros2 topic pub --once /delta_marlin/gcode std_msgs/msg/String "data: 'G1 X10 Y10 Z10 F6000'"

# Home the machine
ros2 topic pub --once /delta_marlin/gcode std_msgs/msg/String "data: 'G28'"
```

### Emergency stop

```bash
ros2 service call /delta_marlin/emergency_stop std_srvs/srv/Trigger
```

### Monitor responses

```bash
ros2 topic echo /delta_marlin/response
```

## Configuration

Edit the configuration file in `config/delta_marlin.yaml`:

```yaml
marlin_controller:
  ros__parameters:
    machine_id: 'delta_marlin'  # The machine ID (used in topic/service names)
    port: '/dev/ttyACM0'        # Serial port for the Marlin device
    baudrate: 250000            # Serial baudrate
    buffer_size: 128            # Command buffer size
```

## Implementation Details

This package is optimized for high-frequency command sending with minimal verification. It:

1. Sends commands without waiting for acknowledgments
2. Uses non-blocking I/O for serial communication 
3. Handles commands in separate threads to avoid blocking the main ROS loop
4. Minimizes processing of command responses

## License

MIT License