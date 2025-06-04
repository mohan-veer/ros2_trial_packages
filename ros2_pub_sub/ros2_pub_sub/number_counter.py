import rclpy
from rclpy.node import Node 
from example_interfaces.msg import Int64

class NumberCounter(Node):

    def __init__(self):
        super().__init__("number_counter")
        self.subscriber_ = self.create_subscription(Int64, "number", self.sub_number_counter, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)

        self.get_logger().info("Subscriber/Pubisher started")

    def sub_number_counter(self, msg: Int64):
        self.get_logger().info(str(msg.data))
        self.publish_received_number(msg)

    def publish_received_number(self, msg):
        self.get_logger().info("publishing the received the number")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
