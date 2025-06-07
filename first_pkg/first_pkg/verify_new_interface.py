import rclpy
from rclpy.node import Node 
from ros2_custom_interfaces.msg import HardwareStatus

class VerifyHardwareStatusNode(Node):
    def __init__(self):
        super().__init__("verify_hardwareinterface")
        self.hw_status_pub_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hw_status)

    
    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.are_motors_ready = True
        msg.debug_message = "Motors are good to go"
        msg.temperature = 44.22

        self.hw_status_pub_.publish(msg)

    


def main(args=None):
    rclpy.init(args=args)
    node = VerifyHardwareStatusNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

