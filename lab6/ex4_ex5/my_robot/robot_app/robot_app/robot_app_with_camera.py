# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class PublisherSubscriber(Node):

	def __init__(self):
		super().__init__('robot_app')
		self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
		self.subscription = self.create_subscription(Image, '/depth/image', self.callback, 10)
		self.msg = Twist()
		self.subscription # prevent unused variable warn
	        
	def callback(self, msg):
		self.get_logger().info('I heard: "%s"' % msg.data[3:6])
		allMore = True
		self.msg.linear.x = 0.5
		self.msg.angular.z = 0.0
		# for i in range(len(msg.ranges)):
		# 	if msg.ranges[i] < 0.8:
		# 		allMore = False
		# 		break
		# if (allMore):
		# 	self.msg.linear.x = 0.5
		# 	self.msg.angular.z = 0.0
		# else:
		# 	self.msg.linear.x = 0.0
		# 	self.msg.angular.z = 0.5
		self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)

    robot_app = PublisherSubscriber()

    rclpy.spin(robot_app)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_app.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()