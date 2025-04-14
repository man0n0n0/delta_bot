import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/svevqx/delta_bot/software/ros_ws_delta/src/install/arducam_rclpy_tof_pointcloud'
