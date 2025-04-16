from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('rock_stacking')
    
    # Define path to the config file
    default_config_path = os.path.join(pkg_share, 'config', 'rock_stacking_params.yaml')
    
    # Declare the config file path as a launch argument
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to the YAML configuration file'
    )
    
    # Launch the rock stacking node with the config file
    rock_stacking_node = Node(
        package='rock_stacking',
        executable='rock_stacking',
        name='rock_stacking',
        output='screen',
        parameters=[LaunchConfiguration('config_path')]
    )
    
    return LaunchDescription([
        config_path_arg,
        rock_stacking_node
    ])