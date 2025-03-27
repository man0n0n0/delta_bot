from threading import Event
import time

from geometry_msgs.msg import Pose

from grbl_msgs.action import SendGcodeCmd, SendGcodeFile
from grbl_msgs.msg import State
from grbl_msgs.srv import Stop
from grbl_ros import grbl

import rclpy
from rclpy.action import ActionClient, ActionServer

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tf2_ros.transform_broadcaster import TransformBroadcaster


class grbl_node(Node):
    def __init__(self):
        super().__init__('grbl_device')

        self.get_logger().info('Declaring ROS parameters')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('machine_id', 'delta_marlin'),
                ('port', '/dev/ttyACM0'),
                ('baudrate', 250000),
                ('acceleration', 50),  # mm / min^2
                ('x_max', 200),  # mm
                ('y_max', 200),  # mm
                ('z_max', 100),  # mm
                ('default_v', 100),  # mm / min
                ('x_max_v', 150),  # mm / min
                ('y_max_v', 150),  # mm / min
                ('z_max_v', 150),  # mm / min
                ('x_steps', 100),  # mm
                ('y_steps', 100),  # mm
                ('z_steps', 100),  # mm
            ])

        self.machine_id = self.get_parameter('machine_id').get_parameter_value().string_value
        self.get_logger().info('Initializing Publishers & Subscribers')
        # Initialize Publishers
        self.pub_tf_ = TransformBroadcaster(self)
        self.pub_mpos_ = self.create_publisher(Pose, self.machine_id + '/machine_position', 5)
        self.pub_wpos_ = self.create_publisher(Pose, self.machine_id + '/work_position', 5)
        self.pub_state_ = self.create_publisher(State, self.machine_id + '/state', 5)
        # Initialize Services
        self.srv_stop_ = self.create_service(
            Stop, self.machine_id + '/stop', self.stopCallback)
        # Initialize Actions
        self.callback_group = ReentrantCallbackGroup()
        self.action_send_gcode_ = ActionServer(
                self,
                SendGcodeCmd,
                self.machine_id + '/send_gcode_cmd',
                self.gcodeCallback)
        self.action_send_gcode_file_ = ActionServer(
                self,
                SendGcodeFile,
                self.machine_id + '/send_gcode_file',
                self.streamCallback)

        self.get_logger().info('Getting ROS parameters')
        port = self.get_parameter('port')
        baud = self.get_parameter('baudrate')
        acc = self.get_parameter('acceleration')    # axis acceleration (mm/s^2)
        max_x = self.get_parameter('x_max')           # workable travel (mm)
        max_y = self.get_parameter('y_max')           # workable travel (mm)
        max_z = self.get_parameter('x_max')           # workable travel (mm)
        default_speed = self.get_parameter('default_v')   # mm/min
        speed_x = self.get_parameter('x_max_v')     # mm/min
        speed_y = self.get_parameter('y_max_v')     # mm/min
        speed_z = self.get_parameter('z_max_v')     # mm/min
        steps_x = self.get_parameter('x_steps')      # axis steps per mm
        steps_y = self.get_parameter('y_steps')      # axis steps per mm
        steps_z = self.get_parameter('z_steps')      # axis steps per mm

        self.get_logger().warn('  machine_id: ' + str(self.machine_id))
        self.get_logger().warn('  port:       ' + str(port.get_parameter_value().string_value))
        self.get_logger().warn('  baudrate:   ' + str(baud.get_parameter_value().integer_value))

        self.get_logger().info('Initializing GRBL Device')
        self.machine = grbl(self)
        self.get_logger().info('Starting up GRBL Device...')
        self.machine.startup(self.machine_id,
                             port.get_parameter_value().string_value,
                             baud.get_parameter_value().integer_value,
                             acc.get_parameter_value().integer_value,
                             max_x.get_parameter_value().integer_value,
                             max_y.get_parameter_value().integer_value,
                             max_z.get_parameter_value().integer_value,
                             default_speed.get_parameter_value().integer_value,
                             speed_x.get_parameter_value().integer_value,
                             speed_y.get_parameter_value().integer_value,
                             speed_z.get_parameter_value().integer_value,
                             steps_x.get_parameter_value().integer_value,
                             steps_y.get_parameter_value().integer_value,
                             steps_z.get_parameter_value().integer_value)
        if(self.machine.s):
            self.machine.getStatus()
            self.machine.getSettings()
        else:
            self.get_logger().warn('Could not detect GRBL device '
                                   'on serial port ' + self.machine.port)
            self.get_logger().warn('Are you sure the GRBL device '
                                   'is connected and powered on?')
            self.machine.mode = self.machine.MODE.DEBUG

    def gcodeCallback(self, goal_handle):
        """
        Send GCODE ROS2 action callback with minimal waiting.
        """
        result = SendGcodeCmd.Result()
        status = self.machine.send(str(goal_handle.request.command))
        
        # Minimal error checking, no waiting
        if status.find('error') > -1:
            result.success = False
        else:
            result.success = True
        
        goal_handle.succeed()
        return result

    def streamCallback(self, goal_handle):
        """
        Send GCODE file ROS2 action callback with minimal waiting.
        """
        result = SendGcodeFile.Result()
        # open file to read each line
        f = open(goal_handle.request.file_path, 'r')

        line_num = 0
        file_lines = f.readlines()
        file_length = len(file_lines)

        for raw_line in file_lines:
            line = raw_line.strip()  # strip all EOL characters for consistency
            self.machine.send(line)

            # Minimal feedback, no waiting
            status_msg = SendGcodeFile.Feedback()
            status_msg.status = '[ ' + str(line_num) + ' / ' + str(file_length) + \
               ' ] Sent ' + str(line)
            goal_handle.publish_feedback(status_msg)

            line_num += 1

        goal_handle.succeed()
        result.success = True
        return result

    def stopCallback(self, request, response):
        # stop steppers
        if request.data == 's':
            self.machine.disableSteppers()
        # fire steppers
        elif request.data == 'f':
            self.machine.enableSteppers()


def main(args=None):
    rclpy.init(args=args)
    node = grbl_node()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()