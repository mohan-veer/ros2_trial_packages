import rclpy
from rclpy.node import Node 
from example_interfaces.srv import AddTwoInts 
from functools import partial

class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")

    def call_add_two_ints(self, a, b):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("waiting for service")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.call_back_call_add_two_ints, request=request))


    def call_back_call_add_two_ints(self, future, request):
        response = future.result()
        self.get_logger().info("Got response: "+str(response.sum))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    node.call_add_two_ints(2,7)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()