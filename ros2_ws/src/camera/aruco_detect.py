import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from aruco_msgs.msg import ArucoMarker


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector')
        self.cb = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # ROS2 subscriptions and publications
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)
        self.marker_publisher = self.create_publisher(ArucoMarker, '/aruco_markers', 10)

        self.camera_matrix = None
        self.distortion_coefficients = None

    def image_callback(self, msg):
        image = self.cb.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Aruco marker detection
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        if len(corners) > 0:
            # Estimate marker pose for each detected marker
            for i, corner in enumerate(corners):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corner, 0.03, self.camera_matrix, self.distortion_coefficients)

                # Create and publish ArucoMarker message
                aruco_msg = ArucoMarker()
                aruco_msg.header.stamp = msg.header.stamp
                aruco_msg.id = ids[i][0]

                # Convert tvec (camera frame) to PoseStamped (world frame) using camera info (if available)
                if self.camera_matrix is not None:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = msg.header.stamp
                    pose_msg.pose.position.x = tvec[0][0]
                    pose_msg.pose.position.y = tvec[1][0]
                    pose_msg.pose.position.z = tvec[2][0]
                    # Assuming rotation is negligible for simplicity
                    aruco_msg.pose = pose_msg
                else:
                    self.get_logger().warn("Camera info not available for pose estimation")

                self.marker_publisher.publish(aruco_msg)

        # Optional: Draw markers and IDs on the image for visualization
        # cv2.aruco.drawDetectedMarkers(image, corners, ids)
        # cv2.imshow('Aruco Detection', image)
        # cv2.waitKey(1)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.frombuffer(msg.k, dtype=np.float64).reshape((3, 3))
        self.distortion_coefficients = np.frombuffer(msg.distortion_coefficients, dtype=np.float64)


def main():
    rclpy.init()
    node = ArucoDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
