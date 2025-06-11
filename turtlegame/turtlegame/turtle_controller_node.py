import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from ros2_custom_interfaces.msg import NextLocation
from ros2_custom_interfaces.msg import ReachedTarget
import math


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.target_x = None
        self.target_y = None
        self.reached_target_ = None
        self.current_position_ = []
        self.subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.set_current_position, 10)
        self.next_location_subscriber_ = self.create_subscription(NextLocation, "/nexttarget", self.get_next_location, 10)
        self.location_reached_publisher_ = self.create_publisher(ReachedTarget, "/reachedtarget", 10)
        # publisher for turtle movement
        self.turtle_move_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        # main control loop - responsible for movement
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)


    def set_current_position(self, msg: Pose):
        self.get_logger().info("x value = "+str(msg.x)+" y value = "+str(msg.y)+" theta = "+str(msg.theta)+" lvel = "+str(msg.linear_velocity)+" avel = "+str(msg.angular_velocity))
        self.current_position_ = [msg.x, msg.y, msg.theta]

    def get_next_location(self, msg: NextLocation):
        self.target_x = msg.target_x
        self.target_y = msg.target_y
        self.reached_target_ = False

    def control_loop(self):
        if self.current_position_ is None or self.target_x is None or self.target_y is None:
            return
        
        if len(self.current_position_) < 3:
            return

        
        dist_x = self.target_x - self.current_position_[0]
        dist_y = self.target_y - self.current_position_[1]

        distance = math.sqrt(dist_x**2 + dist_y**2)

        cmd = Twist()

        if distance > 0.5:
            # position
            cmd.linear.x = 2*distance

            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.current_position_[2]

            #normalize the angle
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            cmd.angular.z = 6*diff
        else:
            # target reached
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if not self.reached_target_:
                self.reached_target_ = True
                msg = ReachedTarget()
                msg.reached_target = self.reached_target_
                self.location_reached_publisher_.publish(msg)

        self.turtle_move_publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()