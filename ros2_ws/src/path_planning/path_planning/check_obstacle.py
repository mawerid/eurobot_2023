import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Camera image subscription
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Camera info subscription
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)

        # Twist publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize variables
        self.image = None
        self.camera_matrix = None
        self.distortion_coefficients = None

        # Obstacle detection parameters (adjust based on your environment)
        self.lower_color = np.array([0, 0, 100])  # Adjust for desired obstacle color (HSV)
        self.upper_color = np.array([179, 255, 255])  # Adjust for desired obstacle color (HSV)
        self.min_contour_area = 1000  # Minimum size of an obstacle contour to be considered

        # Velocity control parameters
        self.linear_velocity = 0.2  # Forward velocity (m/s)
        self.angular_velocity = 0.5  # Turning velocity (rad/s)

        self.get_logger().info("Obstacle avoidance node started")

    def image_callback(self, msg):
        self.image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.distortion_coefficients = np.array(msg.D)

    def run(self):
        rclpy.spin(self)

    def avoid_obstacles(self):
        if self.image is None or self.camera_matrix is None or self.distortion_coefficients is None:
            self.get_logger().warn("Missing image or camera info data")
            return

        # Undistort image (if necessary)
        undistorted_image = cv2.undistort(self.image, self.camera_matrix, self.distortion_coefficients)

        # Convert to HSV color space for better color-based segmentation
        hsv_image = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2HSV)

        # Mask out desired obstacle color range
        mask = cv2.inRange(hsv_image, self.lower_color, self.upper_color)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest obstacle contour (if any)
        largest_contour = None
        largest_contour_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_contour_area and area > self.min_contour_area:
                largest_contour = contour
                largest_contour_area = area

        # Calculate movement direction based on obstacle position (if present)
        if largest_contour is not None:
            # Get moments of the largest contour
            M = cv2.moments(largest_contour)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Calculate image center coordinates
            image_center_x = undistorted_image.shape[1] // 2
            image_center_y = undistorted_image.shape[0] // 2

            # Determine direction based on obstacle position relative to image center
            twist_msg = Twist()

            # Move forward if no obstacle detected or obstacle is far away
            if largest_contour_area < self.min_contour_area:
                twist_msg.linear.x = self.linear_velocity
                twist_msg.angular.z = 0.0

            # Turn left if obstacle is closer on the right
            elif cx > image_center_x + 50:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.angular_velocity

            # Turn right if obstacle is closer on the left
            elif cx < image_center_x - 50:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = -self.angular_velocity

            # Slow down and stop if obstacle is in the center
            else:
                twist_msg.linear.x = self.linear_velocity / 2.0
                twist_msg.angular.z = 0.0

            self.velocity_publisher.publish(twist_msg)

            # Display the processed image for debugging (optional)
            # cv2.imshow('Processed Image', undistorted_image)
            # cv2.waitKey(1)


if __name__ == '__main__':
    rclpy.init()
    node = ObstacleAvoidanceNode()
    node.run()
    rclpy.shutdown()
