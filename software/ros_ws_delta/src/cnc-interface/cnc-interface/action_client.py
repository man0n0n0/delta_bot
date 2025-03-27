import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_interfaces.action import StringCommand 


class SingleCommandActionClient(Node):

    def __init__(self):
        super().__init__('command_action_client')
        self._action_client = ActionClient(self, StringCommand, 'command')

    def send_goal(self, command):
        goal_msg = StringCommand.Goal()
        goal_msg.command = command

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('command rejected :(')
            return

        self.get_logger().info('command accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        self.get_logger().info('Result: {0}'.format(result.final_status))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_status))


def main(args=None):
    rclpy.init(args=args)

    action_client = SingleCommandActionClient()

    # action_client.send_goal("executeGcode ['G0 Z20 F1500;', 'G28', 'G90']")
    action_client.send_goal("executeGcode ['G28']")
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
