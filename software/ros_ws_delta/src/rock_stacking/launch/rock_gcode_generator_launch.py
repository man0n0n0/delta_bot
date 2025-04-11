from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for configurable parameters
        DeclareLaunchArgument(
            'center_x',
            default_value='0.0',
            description='X coordinate of center position'
        ),
        DeclareLaunchArgument(
            'center_y',
            default_value='0.0',
            description='Y coordinate of center position'
        ),
        DeclareLaunchArgument(
            'center_z',
            default_value='0.1',
            description='Z coordinate of center position (height)'
        ),
        DeclareLaunchArgument(
            'approach_height',
            default_value='0.05',
            description='Height for approaching rocks'
        ),
        DeclareLaunchArgument(
            'pick_height',
            default_value='0.01',
            description='Height for picking rocks'
        ),
        DeclareLaunchArgument(
            'feedrate',
            default_value='1000',
            description='Feedrate for G-code movements'
        ),
        DeclareLaunchArgument(
            'gcode_file',
            default_value='rock_pick_place.gcode',
            description='Output G-code filename'
        ),
        
        # Launch the rock stacking node
        Node(
            package='rock_stacking',
            executable='rock_stacking',
            name='rock_stacking',
            output='screen',
            parameters=[{
                'center_x': LaunchConfiguration('center_x'),
                'center_y': LaunchConfiguration('center_y'),
                'center_z': LaunchConfiguration('center_z'),
                'approach_height': LaunchConfiguration('approach_height'),
                'pick_height': LaunchConfiguration('pick_height'),
                'feedrate': LaunchConfiguration('feedrate'),
                'gcode_file': LaunchConfiguration('gcode_file'),
                'gripper_open_command': 'M8',
                'gripper_close_command': 'M9'
            }]
        )
    ])