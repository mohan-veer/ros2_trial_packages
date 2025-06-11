import rclpy
from rclpy.node import Node 
from example_interfaces.msg import String


class RobotNewsStationNode(Node):
    
    def __init__(self):
        super().__init__("robot_news_station")
        # creating a publisher
        self.declare_parameter("robot_name", "M144")
        self.publisher_ = self.create_publisher(String, "robot_news",  10)
        self.timer_ = self.create_timer(1, self.publish_news)
        self.get_logger().info("Robot statement after timer")
        
    def publish_news(self):
        msg = String()
        self.robot_name_ = self.get_parameter("robot_name").value
        msg.data = "Hello I am "+self.robot_name_+" sending the today temperature - 10C"
        self.publisher_.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()