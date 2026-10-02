#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ConstantControl(Node):

	def __init__(self):
		super().__init__('constant_control')
			
		self.timer = self.create_timer(
			0.2,
			self.timer_callback
		)

	def timer_callback(self):
		self.get_logger().info('sending constant control...')

def main(args=None):
	rclpy.init(args=args)
	
	node = ConstantControl()

	rclpy.spin(node)

	node.destroy_node()

	rclpy.shutdown()

if __name__ == '__main__':
	main()


