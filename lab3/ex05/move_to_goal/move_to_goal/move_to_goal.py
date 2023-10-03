from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import rclpy
import sys
import math
import time

class TurtleBot(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')

        # Publisher for '/turtle1/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber for '/turtle1/pose'. Calls update_pose when a Pose message is received.
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)

        self.pose = Pose()
        self.timer = self.create_timer(0.1, self.move2goal)

    def update_pose(self, data):
        """Callback function called when a new Pose message is received."""
        self.pose = data

    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def move2goal(self):
        """Moves the turtle to the goal."""
        const = 5
        vel_msg = Twist()
        goal_pose = Pose()

        # Get the goal coordinates and angle from command line arguments.
        goal_pose.x = float(sys.argv[1])
        goal_pose.y = float(sys.argv[2])
        goal_pose.theta = math.radians(float(sys.argv[3]))

        # Check if goal coordinates and angle are within valid ranges.
        if not (0 <= goal_pose.x <= 10 and 0 <= goal_pose.y <= 10 and -360 <= goal_pose.theta <= 360):
            self.get_logger().info("Wrong data")
            quit()

        distance_to_goal = math.sqrt((goal_pose.x - self.pose.x) ** 2 + (goal_pose.y - self.pose.y) ** 2)
        angle = self.steering_angle(goal_pose)

        self.get_logger().info('%s radians' % self.pose.theta)
        
        # Rotate to align with the goal angle.
        vel_msg.angular.z = -self.pose.theta
        self.velocity_publisher.publish(vel_msg)
        time.sleep(2)

        # Rotate to the calculated angle.
        vel_msg.angular.z = angle
        self.velocity_publisher.publish(vel_msg)
        time.sleep(2)

        # Move forward to the goal position.
        vel_msg.linear.x = distance_to_goal
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        time.sleep(2)

        # Stop moving.
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = -angle
        self.velocity_publisher.publish(vel_msg)
        time.sleep(2)

        # Rotate to the goal angle.
        vel_msg.angular.z = goal_pose.theta
        self.velocity_publisher.publish(vel_msg)
        time.sleep(2)

        self.get_logger().info("Goal Reached!! ")
        quit()

def main(args=None):
    rclpy.init(args=args)
    x = TurtleBot()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

