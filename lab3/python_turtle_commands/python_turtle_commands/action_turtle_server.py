import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from action_turtle_commands.action import MessageTurtleCommands
import time
import math

# Класс CommandActionServer наследуется от Node и представляет сервер действия для управления черепахой
class CommandActionServer(Node):

    def __init__(self):
        super().__init__('action_turtle_server')
        self.float_odom = 0.0
        self.flag = 0
        self.twist = Twist()
        self.before_pose = Pose()
        self.after_pose = Pose()
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'move_turtle',
            self.execute_callback)
        self.goal_handle = None
        self.subscription 

    # Метод callback обрабатывает сообщения о позиции черепахи
    def callback(self, msg):
        if self.flag == 1:
            self.before_pose = msg
            self.flag = 0
        self.after_pose = msg

    # Метод execute_callback выполняет цели, полученные через сервер действия
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # В зависимости от команды в цели, устанавливаем значения скорости черепахи
        if goal_handle.request.command == 'forward':
            self.twist.linear.x = float(goal_handle.request.s)
            self.twist.angular.z = 0.0
        elif goal_handle.request.command == 'turn_left':
            self.twist.linear.x = 0.0
            self.twist.angular.z = float(goal_handle.request.angle * 3.14 / 180)
        elif goal_handle.request.command == 'turn_right':
            self.twist.linear.x = 0.0
            self.twist.angular.z = -float(goal_handle.request.angle * 3.14 / 180)

        self.flag = 1
        self.publisher_.publish(self.twist)
        
        time.sleep(1)
        
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 1
        goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        self.goal_handle = goal_handle
        return MessageTurtleCommands.Result(result=True)

    # Метод cancel_callback вызывается, если цель отменяется клиентом
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal canceled')
        self.publisher_.publish(Twist())
        goal_handle.canceled()
        self.flag = 0

# Функция main инициализирует ROS 2 и создает экземпляр CommandActionServer
def main(args=None):
    rclpy.init(args=args)
    action_turtle_server = CommandActionServer()
    try:
        rclpy.spin(action_turtle_server)
    finally:
        action_turtle_server.destroy()
        rclpy.shutdown()

# Проверяем, запускается ли код как самостоятельная программа
if __name__ == '__main__':
    main()

