#!/usr/bin/env python3
import sys
import math
import rclpy
from functools import partial
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_interfaces.msg import Turtle
from turtlesim_interfaces.msg import TurtleArray
from turtlesim_interfaces.srv import CatchTurtle

class GoToLocationNode(Node):
    def __init__(self):
        super().__init__("go_to_loc_node")
        self.coeff = 1.5
        self.pose_threshold_linear = 0.5 
        self.pose_threshold_angular = 0.01
        
        self.pose_ = None
        self.new_turtle_to_catch_ = None

        self.new_turtle_subscriber_ = self.create_subscription(
            TurtleArray, "/new_turtles", self.callback_new_turtles, 10
        )

        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)
        
      
        self.timer = self.create_timer(0.1, self.turtle_controller)

        self.get_logger().info("Go To Location has been started.")

    def callback_new_turtles(self, msg):
        if len(msg.turtles) > 0:
            
            if self.new_turtle_to_catch_ is None:
                self.new_turtle_to_catch_ = msg.turtles[0]

    def turtle_controller(self):
        if self.pose_ is None or self.new_turtle_to_catch_ is None:
            return
        
        msg = Twist()
        dist_x = self.new_turtle_to_catch_.x - self.pose_.x
        dist_y = self.new_turtle_to_catch_.y - self.pose_.y
        distance_ = math.sqrt(dist_x**2 + dist_y**2)

        target_theta = math.atan2(dist_y, dist_x)
        
        
        angle_diff = target_theta - self.pose_.theta
        
        #Açıyı normalize edildi.
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if abs(angle_diff) > self.pose_threshold_angular:
            msg.angular.z = angle_diff * 2.0 
        else:
           
            if distance_ >= self.pose_threshold_linear:
                msg.linear.x = distance_ * self.coeff
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                
               
                self.call_catch_server(self.new_turtle_to_catch_.name)
                self.new_turtle_to_catch_ = None
                self.get_logger().info("Success! Turtle caught.")

        self.publisher_.publish(msg)

    def call_catch_server(self, turtle_name):
        client_ = self.create_client(CatchTurtle, "/catch_turtle")
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle,
                                         turtle_name=turtle_name))
        
    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
           
            self.get_logger().info(f"Turtle {turtle_name} has been caught successfully.")
        except Exception as e:
            self.get_logger().error("Service call failed %r"%(e,))

    def callback_pose(self, msg):
        self.pose_ = msg

def main(args=None):
    rclpy.init(args=args)
    node = GoToLocationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()