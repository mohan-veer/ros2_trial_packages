#!/usr/bin/env python3

import rclpy
import random
from rclpy.node import Node
from rclpy.parameter import Parameter
from example_interfaces.msg import Int64
from rcl_interfaces.msg import SetParametersResult

class NumberPublisher(Node):
	def __init__(self):
		super().__init__("number_publisher")
		# declaring the ros 2 paramater
		self.declare_parameter("number", 2)
		self.declare_parameter("timer_period", 1.0)
		# get the parameter
		self.number_ = self.get_parameter("number").value
		self.timer_period_  = self.get_parameter("timer_period").value 
		# add the post set parameter callback
		try:
			self.add_post_set_parameters_callback(self.parameters_call_back)
		except:
			self.add_on_set_parameters_callback(self.parameters_call_back)
		self.publisher_ = self.create_publisher(Int64, "number", 10)
		self.timer_ = self.create_timer(self.timer_period_, self.publish_number)
		self.get_logger().info("Number publisher is called")

		

	def publish_number(self):
		msg = Int64()
		msg.data = self.number_
		self.publisher_.publish(msg)

	def parameters_call_back(self, params: list[Parameter]):
		for param in params:
			if param.name == "number":
				self.number_ = param.value

		return SetParametersResult(successful=True)
		

def main(args=None):
	rclpy.init(args=args)
	node = NumberPublisher()
	rclpy.spin(node)
	rclpy.shutdown()
	
	
if __name__ == "__main__":
	main()