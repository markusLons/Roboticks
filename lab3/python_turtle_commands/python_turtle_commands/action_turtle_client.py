import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_turtle_commands.action import MessageTurtleCommands

class CommandActionClient(Node):

    def __init__(self):
        super().__init__('action_turtle_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'move_turtle')
        self._action_client.wait_for_server()
        self.send_goals()

    def send_goals(self):
        def send_goal(goal_msg, delay_seconds):
            self.get_logger().info(f'Sending goal: {goal_msg.command}')
            self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

        goal_messages = [
            {'command': 'forward', 's': 2, 'angle': 0},
            {'command': 'turn_right', 's': 0, 'angle': 90},
            {'command': 'forward', 's': 1, 'angle': 0}
        ]

        for goal_data in goal_messages:
            goal_msg = MessageTurtleCommands.Goal()
            goal_msg.command = goal_data['command']
            goal_msg.s = goal_data['s']
            goal_msg.angle = goal_data['angle']

            send_goal(goal_msg, delay_seconds=2)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
        else:
            self.get_logger().info('Goal accepted')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.odom}')

def main(args=None):
    rclpy.init(args=args)
    action_client = CommandActionClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

