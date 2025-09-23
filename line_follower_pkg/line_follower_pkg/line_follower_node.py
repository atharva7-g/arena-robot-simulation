#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class LineFollower(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('line_follower_node')

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Create a subscriber to the camera's image topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)

        # Create a publisher for Twist messages
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a Twist message object
        self.twist = Twist()

        # PID controller constants (tuning required)
        self.kp = 0.005 # Proportional gain
        self.ki = 0.0   # Integral gain
        self.kd = 0.01  # Derivative gain

        # PID state variables
        self.previous_error = 0.0
        self.integral = 0.0

        self.get_logger().info("Line Follower Node Initialized (ROS 2)")

    def image_callback(self, msg):
        """
        Processes incoming images to detect the line and calculate control commands.
        """
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # --- Image Processing ---

        height, width, _ = cv_image.shape
        roi_start_row = int(height / 2)
        roi = cv_image[roi_start_row:, :]

        # 1. Grayscale and Blur
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # 2. Thresholding
        _, thresh = cv2.threshold(blur, 127, 255, cv2.THRESH_BINARY_INV)

        # 3. Find Centroid
        M = cv2.moments(thresh)

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])

            # --- Control Logic ---
            error = cx - (width / 2)

            self.integral += error
            derivative = error - self.previous_error

            turn = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.previous_error = error

            # Set velocities
            self.twist.linear.x = 0.2
            self.twist.angular.z = -turn

        else:
            # No line detected, stop
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        # Publish the movement command
        self.cmd_vel_pub.publish(self.twist)

        # Optional: Display the processed image
        cv2.imshow("Processed Image", thresh)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        line_follower.get_logger().info('Node stopped cleanly')
    finally:
        # Destroy the node explicitly
        line_follower.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
