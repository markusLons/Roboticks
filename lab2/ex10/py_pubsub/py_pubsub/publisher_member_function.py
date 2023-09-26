import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TextToCmdVel(Node):

    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.text_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.twist = Twist()

    def text_callback(self, msg):
        command = msg.data
        self.get_logger().info('Received command: "%s"' % command)

        if command == 'turn_right':
            self.twist.angular.z = -1.5  
            self.twist.linear.x = 0.0
        elif command == 'turn_left':
            self.twist.angular.z = 1.5  
            self.twist.linear.x = 0.0
        elif command == 'move_forward':
            self.twist.angular.z = 0.0
            self.twist.linear.x = 1.0  
        elif command == 'move_backward':
            self.twist.angular.z = 0.0
            self.twist.linear.x = -1.0  
        else:
            return

        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)

    text_to_cmd_vel_node = TextToCmdVel()

    rclpy.spin(text_to_cmd_vel_node)

    # Завершение узла
    text_to_cmd_vel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

