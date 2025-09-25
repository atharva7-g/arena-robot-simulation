#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class MoveBaseToPosition(Node):
    def __init__(self):
        super().__init__("move_base_to_position")

        # Parameters
        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 1.0)
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("distance_tolerance", 0.05)
        self.declare_parameter("angle_tolerance", 0.05)

        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value
        self.max_linear_speed = self.get_parameter("linear_speed").value
        self.max_angular_speed = self.get_parameter("angular_speed").value
        self.dist_tol = self.get_parameter("distance_tolerance").value
        self.ang_tol = self.get_parameter("angle_tolerance").value

        # Publisher to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, "/sarm/cmd_vel", 10)

        # Subscriber to odom
        self.odom_sub = self.create_subscription(
            Odometry, "/sarm/odom", self.odom_callback, 10
        )

        # Current robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Orientation to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        twist = Twist()

        # Compute distance and angle to goal
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.yaw)

        # Stop if close enough
        if distance < self.dist_tol:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("Goal reached!")
            return

        # Proportional control
        twist.linear.x = (
            min(self.max_linear_speed * distance, self.max_linear_speed)
            if abs(angle_diff) < math.pi / 6
            else 0.0
        )
        twist.angular.z = max(
            -self.max_angular_speed, min(self.max_angular_speed, 2.0 * angle_diff)
        )

        self.cmd_pub.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = MoveBaseToPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
