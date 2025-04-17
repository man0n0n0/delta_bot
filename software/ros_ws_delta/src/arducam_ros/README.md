# ArduCam ROS

A ROS2 package for ArduCam Time-of-Flight (ToF) cameras with point cloud rotation capability.
(claude made) 

## Features

- Point cloud generation from ArduCam ToF camera
- Configurable rotation for the point cloud to match physical camera orientation
- Real-time depth data publishing
- Simple configuration through ROS parameters

## Installation

### Prerequisites

- ROS2 (tested with jazzy)
- ArduCam ToF Camera
- ArduCam Python SDK (ArducamDepthCamera)

### Setup

```bash
# Clone the repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/your-repo/arducam_ros.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y

# Build the package
colcon build --packages-select arducam_ros

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Command

```bash
# Run with default settings (no rotation)
ros2 run arducam_ros tof_pointcloud
```

### With Point Cloud Rotation

```bash
# Run with 90-degree rotation
ros2 run arducam_ros tof_pointcloud --ros-args -p rotation_angle:=90.0
```

### Launch File

```bash
# Run with launch file (default 0 degrees rotation)
ros2 launch arducam_ros tof_pointcloud.launch.py

# Run with custom rotation angle
ros2 launch arducam_ros tof_pointcloud.launch.py rotation_angle:=180.0
```

## Parameters

- `rotation_angle`: Rotation angle in degrees (0-360) for the point cloud (default: 0.0)
- Additional camera parameters can be configured in the ArduCam SDK

## Published Topics

- `/point_cloud` (sensor_msgs/PointCloud2): 3D point cloud data
- `/depth_frame` (std_msgs/Float32MultiArray): Raw depth data

## Configuration

For advanced camera configuration, you can specify a camera configuration file:

```bash
ros2 run arducam_ros tof_pointcloud --cfg /path/to/camera/config.json --ros-args -p rotation_angle:=90.0
```

## Author

Manoah Camporini <camporini@protonmail.com>

## License

Apache License, Version 2.0