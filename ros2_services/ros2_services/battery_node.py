import rclpy
from rclpy.node import Node
from ros2_custom_interfaces.srv import SetLedState
from functools import partial


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery")
        self.client_ = self.create_client(SetLedState, "set_led")
        self.get_logger().info("The client is started!!")
        self.battery_status_ = "full"
        self.last_time_battery_checked_ = self.get_current_time_seconds()
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_status)


    def get_current_time_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds/1000000000.0
    
    def check_battery_status(self):
        self.time_now = self.get_current_time_seconds()
        request = SetLedState.Request()
        send_req = False
        if self.battery_status_ == "full":
            if self.time_now - self.last_time_battery_checked_ > 4.0:
                self.battery_status_ = "empty"
                self.last_time_battery_checked_ = self.time_now
                # send request to the led to turn on
                request.set_led_panel_num = 2
                request.panel_state = True
                send_req = True

        else:
            if self.time_now - self.last_time_battery_checked_ > 6.0:
                self.battery_status_ = "full"
                self.last_time_battery_checked_ = self.time_now
                # send request to turn off the led
                request.set_led_panel_num = 2
                request.panel_state = False
                send_req = True

        while not self.client_.wait_for_service(1.0):
            self.get_logger().info("waiting for the service to start")

        if send_req:
            future = self.client_.call_async(request)
            future.add_done_callback(partial(self.call_back_for_led_panel_request, request=request))

    def call_back_for_led_panel_request(self, future, request):
        response = future.result()
        self.get_logger().info("Got response from seerver: "+str(response.result))



def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()