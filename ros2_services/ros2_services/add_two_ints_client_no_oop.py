import rclpy
from rclpy.node import Node 
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_ints_no_oop")

    #create client
    client = node.create_client(AddTwoInts, "add_two_ints")
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for add 2 ints server...")

    request = AddTwoInts.Request()
    request.a = 5
    request.b = 10

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response = future.result()
    node.get_logger().info(str(request.a)+" + "+str(request.b)+" = "+str(response.sum))
    rclpy.shutdown()

if __name__ == "__main__":
    main()