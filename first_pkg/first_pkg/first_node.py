#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    
    def __init__(self):
        super().__init__("py_test")
        self.get_logger().info("Hello, inside the constructor")
        self.create_timer(5.0, self.timer_callback)
        
     
    def timer_callback(self):
        self.get_logger().info("call back print!!")

def main(args=None):
    # intialize the ros2 communication
    rclpy.init(args=args)
    
    #create node
    node  = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()