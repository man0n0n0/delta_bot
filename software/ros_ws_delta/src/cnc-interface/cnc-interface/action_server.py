import time
import ast
import sys

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_interfaces.action import StringCommand
from printdriver import PrintDriver

class CNCActionServer(Node):

    def __init__(self, usbserial):
        super().__init__('command_action_client')
        self._action_server = ActionServer(
            self,
            StringCommand,
            'command',
            self.execute_callback)
        self.print_driver = PrintDriver(usbserial)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Server is working...')

        command_string = goal_handle.request.command
        self.get_logger().info('Feedback: start command {0}'.format(command_string))
        
        command, args, kargs = self.parse_command(command_string)
        self.execute_command(command, args, kargs)

        feedback_msg = StringCommand.Feedback()

        while not self.print_driver.printcore.clear:
            feedback_msg.current_status = "processing"
            self.get_logger().info('Feedback: {0}'.format(goal_handle.request.command))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = StringCommand.Result()
        result.final_status = 'command {0} done'.format(command)
        return result

    def execute_command(self, command, args, kargs):
        command_op = getattr(self.print_driver, command, None)

        print(command_op)
        print(command)
        print(args)
        print(kargs)

        if command_op:
            command_op(*args, **kargs)

        else:
            raise ValueError("No such command: " + command)


    def parse_command(self, command_string):
        command = ""
        args = []
        kargs = {}

        if command_string[:12] == "executeGcode":
            gcodelist = command_string[13:]
            print(gcodelist)
            gcodelist = ast.literal_eval(gcodelist)
            command = "executeGcode"
            args = [gcodelist]

        elif command_string[:14] == "moveTipToPoint":
            pass

        else:
            raise ValueError("no command found")

        # self.get_logger().info(f'parse result: command {command}')
        return command, args, kargs

def main(usbserial):
    rclpy.init(args=None)

    cnc_action_server = CNCActionServer(usbserial)

    rclpy.spin(cnc_action_server)


def usage():
    return "%s [cmd]"%sys.argv[0]

if __name__ == '__main__':
    print(sys.argv)
    if len(sys.argv) == 2:
        serialport = str(sys.argv[1])

    else:
        print(usage())
        sys.exit(1)

    main(serialport)
