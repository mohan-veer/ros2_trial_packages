import rclpy
from rclpy.node import Node
from ros2_custom_interfaces.srv import SetLedState
from ros2_custom_interfaces.msg import LedPanelState

class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.declare_parameter("led_states", [0,0,0])
        self.server_ = self.create_service(SetLedState, "set_led", self.callback_to_set_ledpanel)
        self.publisher_ = self.create_publisher(LedPanelState, "led_panel_state", 10)
        self.get_logger().info("The server is started!!")

        #print the default led panel state, that we got from run time
        self.led_panel_state_ = self.get_parameter("led_states").value
        self.get_logger().info(f'The defautl state provided by run time is -> {self.led_panel_state_}')

    
    def callback_to_set_ledpanel(self, request: SetLedState.Request, response: SetLedState.Response):
        #set the respective led panel state and publish it to a topic
        led_to_change = request.set_led_panel_num
        led_status_to_change = request.panel_state

        msg = LedPanelState()
        msg.led_panel_status[led_to_change] = 1 if led_status_to_change else 0
        self.publisher_.publish(msg)
        self.get_logger().info("from server, published the message")
        response.result = True

        return response



def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()