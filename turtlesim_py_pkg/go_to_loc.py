#!/usr/env/bin python3
import sys
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class GoToLocationNode(Node):
    def __init__(self):
        super().__init__("go_to_loc_node")
        self.pose_threshold = 0.3
        self.target_x = float(sys.argv[1])
        self.target_y = float(sys.argv[2])
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)
        self.timer = self.create_timer(1, self.turtle_contreoller)

        self.get_logger().info("Go To Location has been started.")

    def turtle_contreoller(self):
        msg = Twist()
        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance_ = math.sqrt(dist_x**2 + dist_y**2)

        target_theta = math.atan2(dist_y, dist_x)

        if abs(target_theta - self.pose_.theta) > self.pose_threshold:
            msg.angular.z = (target_theta - self.pose_.theta)
        
        else:
            if distance_ >= self.pose_threshold:
                msg.linear.x = distance_
            else:
                msg.linear.x = 0.0
                self.get_logger().info("Succes")

        self.publisher_.publish(msg)
        

    def callback_pose(self, msg):
        self.pose_ = msg

def main(args=None):
    rclpy.init(args=args)
    node = GoToLocationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()