#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('k_angular', 0.004)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.k_angular = self.get_parameter('k_angular').value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def image_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # Crop lower part of image for line detection
        h, w, _ = cv_image.shape
        crop = cv_image[int(h*0.5):, :]  # bottom half

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        # for black line on white ground, threshold low values
        _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)

        # compute centroid of the largest contour
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # find largest contour
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # error from center
                error = (cx - (w/2))
                ang_z = - self.k_angular * error
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = ang_z
                self.pub.publish(twist)
                return

        # if no line found, stop or spin slowly
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
