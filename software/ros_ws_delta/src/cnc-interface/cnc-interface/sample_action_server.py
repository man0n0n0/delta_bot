import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_interfaces.action import Fibonacci as MyActionInterface


class SingleCommandActionServer(Node):

    def __init__(self):
        super().__init__('command_action_client')
        self._action_server = ActionServer(
            self,
            MyActionInterface,
            'command',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = MyActionInterface.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = MyActionInterface.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)

    single_command_action_server = SingleCommandActionServer()

    rclpy.spin(single_command_action_server)


if __name__ == '__main__':
    main()
