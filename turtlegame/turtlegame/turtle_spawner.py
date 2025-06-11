import rclpy
from rclpy.node import Node
from functools import partial

from turtlesim.msg import Pose 
from turtlesim.srv import Spawn, Kill
from ros2_custom_interfaces.msg import NextLocation
from ros2_custom_interfaces.msg import ReachedTarget


import random
import math
import numpy as np

class TurtleSpawner(Node):

    def __init__(self):
        super().__init__("turtle_spawner")
        # Spawn
        # every 0.8 sec the turtle spawns and we get the location where it is spawned
        self.spawn_timer_ = self.create_timer(0.8, self.spawn_new_turtle)
        self.turtles_created_ = 0
        self.spawn_locations_ = [] # list of positions where turtles are spawned and turtle names to be turtle_number
        #create clients
        self.spawn_client_ = self.create_client(Spawn, "/spawn")
        self.kill_client_ = self.create_client(Kill, "/kill")
        self.delete_location_ = -1 # ind of the locationt ocreate_subscription delete after the movement

        self.current_position_ = None
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.set_current_position, 10)

        self.next_location_publisher_ = self.create_publisher(NextLocation, "/nexttarget", 10)
        self.location_reached_subscriber_ = self.create_subscription(ReachedTarget, "/reachedtarget", self.callback_reached_target, 10)
        self.first_call_ = True

    def set_current_position(self, msg: Pose):
        self.get_logger().info("x value = "+str(msg.x)+" y value = "+str(msg.y)+" theta = "+str(msg.theta)+" lvel = "+str(msg.linear_velocity)+" avel = "+str(msg.angular_velocity))
        self.current_position_ = [msg.x, msg.y, msg.theta]

    def spawn_new_turtle(self):
        #hit the spawn service, after checking id the server is up
        while not self.spawn_client_.wait_for_service(1.0):
             self.get_logger().warn("Waiting for the server to start...")

        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)
        request.theta = random.uniform(0.0, 2*math.pi)
        request.name = "turtle_"+str(self.turtles_created_+1)

        future = self.spawn_client_.call_async(request)
        future.add_done_callback(partial(self.call_back_turtle_spawned, request=request))

    def call_back_turtle_spawned(self, future, request):
        response = future.result()
        if request.name == response.name:
            #spawning was successful
            #increment the counter
            self.turtles_created_ += 1
            #add them to the locations list
            self.spawn_locations_.append([request.x, request.y, response.name])
            self.get_logger().info(response.name+" was created at x = "+str(request.x)+" y = "+str(request.y)+" theta = "+str(request.theta))
            # call the get_next_location for forst time and next it is called throught reached target recursively
            if self.first_call_:
                self.get_next_location()
        else:
            self.get_logger().error("turtle didnt spawn!! Error!!")

    def get_next_location(self):
        if len(self.spawn_locations_) > 0:
            # get the optimal/nearest other turtle position from the current location
            locations_only = list(map(lambda item: item[:2], self.spawn_locations_))
            positions = np.array(locations_only)

            distances =  np.sqrt((positions[:,0]-self.current_position_[0])**2+ (positions[:,1]-self.current_position_[1])**2)

            min_idx = np.argmin(distances)
            closest_location = positions[min_idx]
            self.delete_location_ = min_idx

            # publish this next location to topic
            msg = NextLocation()
            msg.target_x = closest_location[0]
            msg.target_y = closest_location[1]
            self.next_location_publisher_.publish(msg)
            if self.first_call_:
                self.first_call_ = False

    def callback_reached_target(self, msg:ReachedTarget):
        if msg is not None:
            if msg.reached_target:
                # delete the location from spawn locations
                # call the kill service
                request = Kill.Request()
                request.name = self.spawn_locations_[self.delete_location_][2]

                future = self.kill_client_.call_async(request=request)
                future.add_done_callback(partial(self.update_locations_upon_movement, request))        

    def update_locations_upon_movement(self, future, request):
        del self.spawn_locations_[self.delete_location_]
        self.delete_location_ = -1
        # publish the next location
        self.get_next_location()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown() 

if __name__ == "__main__":
    main()







