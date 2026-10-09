#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Int64, Bool
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ConstantControl(Node):

	def __init__(self):
		super().__init__('constant_control')
		self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)	
		self.timer = self.create_timer(
			0.2,
			self.timer_callback
		)
		self.kill_sub = self.create_subscription(
			Bool, "/kill", self.kill_callback, 10
		)

	def timer_callback(self):
		msg = Twist()
		msg.linear.x = 0.1
		msg.angular.z = 0.0
		self.cmd_pub.publish(msg)

	def kill_callback(self, msg):
		if msg.data:
			self.timer.cancel()
			self.cmd_pub.publish(Twist())
			self.get_logger().info("Emergency stop: robot stopped")

def main(args=None):
	rclpy.init(args=args)
	
	node = ConstantControl()

	rclpy.spin(node)

	node.destroy_node()

	rclpy.shutdown()

if __name__ == '__main__':
	main()


