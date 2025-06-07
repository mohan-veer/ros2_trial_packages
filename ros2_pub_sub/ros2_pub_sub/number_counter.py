import rclpy
from rclpy.node import Node 
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounter(Node):

    def __init__(self):
        super().__init__("number_counter")
        self.subscriber_ = self.create_subscription(Int64, "number", self.sub_number_counter, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.server_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter)
        self.counter_ = 0

        self.get_logger().info("Subscriber/Pubisher started")

    def sub_number_counter(self, msg: Int64):
        self.get_logger().info(str(msg.data))
        self.counter_ += 1
        counter_msg = Int64()
        counter_msg.data = self.counter_
        self.publish_received_number(counter_msg)

    def publish_received_number(self, msg):
        self.get_logger().info("publishing the received the number")
        self.publisher_.publish(msg)

    def callback_reset_counter(self, request: SetBool.Request, response: SetBool.Response):
        # reset the counter value and return a success message
        try:
            if request.data:
                self.counter_ = 0
                response.success = True
                response.message = "Reset the counter to zero!"
        except Exception as e:
            print(f"Error: {e}")

        return response




def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
