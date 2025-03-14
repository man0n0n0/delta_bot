import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='grbl_ros',
            executable='grbl_node'),

        launch_ros.actions.Node(
            package='arducam_rclpy_tof_pointcloud',
            executable='tof_pointcloud'),
        
        launch_ros.actions.Node(
            package='delta_ros',
            executable='effector_to_closest'),
  ])
