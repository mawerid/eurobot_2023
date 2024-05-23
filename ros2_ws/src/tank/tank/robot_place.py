import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from geometry_msgs.msg import Vector3

class RobotPlace(Node):
    def __init__(self):
        super().__init__('robot_place')

        self.aruco_dict = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
        }
        self.aruco_type = "DICT_4X4_100"
        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.aruco_dict[self.aruco_type])
        self.arucoParams = cv2.aruco.DetectorParameters()

        self.cameraMatrix = np.array([[1366.125571, 0.000000, 941.464876],[0.000000, 1379.945597, 526.444591],
            [0.000000, 0.000000, 1.000000]])

        self.distCoeffs = np.array([[-5.071431],[10.037195],[0.009970],[-0.000137],[-2.288984],[-4.823626],[8.833973],
                                    [-0.047926],[0.000000],[0.000000],[0.000000],[0.000000],[0.000000],[0.000000]])

        self.intrinsic_camera = np.array([[self.cameraMatrix[0, 0], 0, self.cameraMatrix[0, 2]],
                              [0, self.cameraMatrix[1, 1], self.cameraMatrix[1, 2]],
                              [0, 0, 1]])

        self.distortion = np.array([self.distCoeffs[0], self.distCoeffs[1], self.distCoeffs[2], self.distCoeffs[3]])

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 108)
        self.publisher_place = self.create_publisher(Vector3, 'robot_place', 10)

        self.create_timer(0.01, self.image_callback)


    def image_callback(self):
        ret, img = self.cap.read()
        h, w, _ = img.shape
        width = 1000
        height = int(width * (h / w))
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)

        self.pose_estimation(img, self.aruco_dict[self.aruco_type], self.intrinsic_camera, self.distortion)


    def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters()

        corners, ids, _ = cv2.aruco.detectMarkers(gray, cv2.aruco_dict)


        if len(corners) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.02, matrix_coefficients, distortion_coefficients)
   
            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(frame, corners)

                if rvecs[i] is not None and tvecs[i] is not None:
                    rvec = rvecs[i]
                    tvec = tvecs[i]
                    center_marker = np.mean(corners[i][0], axis=0)
                    if ids[i] == 2:
                        msg = Vector3()
                        msg.x = tvec[i][0]
                        msg.y = tvec[i][1]
                        self.publisher_place.publish(msg)
                        print(rvec, tvec)


def main(args=None):
    rclpy.init(args=args)
    robot_place = RobotPlace()

    rclpy.spin(robot_place)
    robot_place.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

