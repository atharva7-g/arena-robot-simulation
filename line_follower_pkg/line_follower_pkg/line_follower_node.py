import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.get_logger().info('Line Follower Node Initialized (ROS 2)')

        # --- TUNING PARAMETERS ---
        # These are the values you'll need to adjust for your specific robot and track
        self.LINEAR_SPEED = 0.15          # Constant forward speed of the robot (m/s)
        self.KP = 0.004                   # Proportional gain: How strongly it reacts to the error
        self.KD = 0.006                   # Derivative gain: How much it dampens oscillations
        self.MAX_ANGULAR_SPEED = 1.5      # Maximum turning speed (rad/s)
        self.THRESHOLD_VALUE = 100        # Black/White threshold for image processing
        self.ROI_TOP_PERCENT = 0.6        # Percentage from the top of the image to start the Region of Interest
        # --- END TUNING ---

        # Declare parameters for flexibility
        self.declare_parameter('camera_topic', '/camera/image')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_debug_display', False)

        # Get parameter values
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.enable_debug = self.get_parameter('enable_debug_display').get_parameter_value().bool_value

        # Create subscribers and publishers
        self.image_subscriber = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Initialize CvBridge and other variables
        self.bridge = CvBridge()
        self.last_error = 0
        self.get_logger().info('Node setup complete. Waiting for images...')

    def image_callback(self, msg):
        """Callback function for processing incoming camera images."""
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # 1. CROP THE IMAGE (REGION OF INTEREST)
        # We only care about the part of the image with the line in front of the robot
        height, width, _ = frame.shape
        roi_top = int(height * self.ROI_TOP_PERCENT)
        roi = frame[roi_top:, :]

        # 2. PROCESS IMAGE
        # Convert to grayscale, apply a binary threshold to get a black and white image
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # The threshold is inverted because the line is black (darker)
        _, mask = cv2.threshold(gray, self.THRESHOLD_VALUE, 255, cv2.THRESH_BINARY_INV)

        # 3. FIND LINE CENTER
        # Calculate the "center of mass" of the white pixels in the mask.
        # This gives us the position of the line.
        M = cv2.moments(mask)
        twist_msg = Twist()

        if M['m00'] > 0:
            # Calculate the x-coordinate of the line's center
            cx = int(M['m10'] / M['m00'])

            # Draw a circle on the line's center for visualization (only if debug enabled)
            if self.enable_debug:
                cv2.circle(roi, (cx, int(roi.shape[0] / 2)), 10, (0, 255, 0), -1)

            # --- PD CONTROLLER ---
            # Calculate the error: difference between the image center and the line center
            center_of_image = width // 2
            error = cx - center_of_image

            # Proportional term
            p_term = self.KP * error
            # Derivative term (helps to reduce wobbling)
            d_term = self.KD * (error - self.last_error)

            # Calculate the required angular velocity (steering)
            angular_z = -(p_term + d_term)

            # Clamp the angular velocity to the maximum allowed value
            twist_msg.angular.z = max(-self.MAX_ANGULAR_SPEED, min(self.MAX_ANGULAR_SPEED, angular_z))
            twist_msg.linear.x = self.LINEAR_SPEED
            self.last_error = error
        else:
            # --- LOST LINE LOGIC ---
            # If no line is detected (M['m00'] == 0), stop the robot.
            self.get_logger().warn('Line not detected, stopping robot.')
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        # 4. PUBLISH THE TWIST MESSAGE
        self.cmd_vel_publisher.publish(twist_msg)

        # 5. DISPLAY THE PROCESSED IMAGE (Optional, but helpful for debugging)
        if self.enable_debug:
            try:
                cv2.imshow("Processed Image", roi)
                cv2.imshow("Mask", mask)
                cv2.waitKey(1)
            except cv2.error as e:
                self.get_logger().warn(f'Could not display debug images: {e}')

def main(args=None):
    rclpy.init(args=args)
    line_follower_node = LineFollower()
    try:
        rclpy.spin(line_follower_node)
    except KeyboardInterrupt:
        line_follower_node.get_logger().info('Node stopped cleanly')
    finally:
        line_follower_node.destroy_node()
        rclpy.shutdown()
        # Clean up OpenCV windows if they were created
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
