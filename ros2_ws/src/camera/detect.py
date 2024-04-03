import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped

# ROS2 libraries
import rclpy
from rclpy.node import Node


class ObjectDetectorNode(Node):

    def __init__(self):
        super().__init__('object_detector_node')
        self.bridge = CvBridge()
        self.color_ranges = {
            "blue_square": ((90, 170), (150, 255), (43, 56)),  # Adjust ranges for blue
            "green_plant": ((43, 86), (35, 77), (111, 130)),  # Adjust ranges for green
            "gray_circle": ((0, 179), (0, 255), (100, 255))  # Adjust ranges for gray
        }
        self.pub_points = self.create_publisher(PointStamped, 'object_points')
        self.image_subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, 'camera_info', self.camera_info_callback, 10)

    def camera_info_callback(self, msg):
        # Extract camera intrinsics if needed for further processing
        # (not required for this basic object detection)
        pass

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Object detection logic
        detected_objects = self.detect_objects(cv_image)

        # Publish object information
        for obj in detected_objects:
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now()
            point_msg.point.x = obj["center_x"]
            point_msg.point.y = obj["center_y"]
            point_msg.point.z = obj["id"]  # Assuming ID as depth for demonstration
            self.pub_points.publish(point_msg)

        # Display image (optional)
        # cv2.imshow('Object Detection', cv_image)
        # cv2.waitKey(1)

    def detect_objects(self, image):
        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define object detection logic
        detected_objects = []
        for object_name, color_range in self.color_ranges.items():
            # Filter image based on color range
            mask = cv2.inRange(hsv_image, color_range[0], color_range[1])
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Find contours and identify centers
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                # Calculate center of contour (assuming object is roughly circular)
                M = cv2.moments(cnt)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                detected_objects.append({
                    "id": object_name,
                    "center_x": cX,
                    "center_y": cY
                })

        return detected_objects


def main():
    rclpy.init_application()
    node = ObjectDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
