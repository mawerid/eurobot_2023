import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np
import cv2


class TankCamera(Node):

    def __init__(self):
        super().__init__('tank_camera')
        self.publisher_plant = self.create_publisher(Vector3, 'plant_id', 10)

        self.aruco_dict = {
             "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
        }
        self.aruco_type = "DICT_4X4_100"

        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.aruco_dict[self.aruco_type])
        self.arucoParams = cv2.aruco.DetectorParameters()

        self.cameraMatrix = np.array([[ 1479.056415,  0.000000,  978.753591],
                                    [ 0.000000,  1509.366213,  566.296338],
                                    [ 0.000000,  0.000000,  1.000000]])

        self.distCoeffs = np.array([[ 9.479044],
                                    [ 114.252232],
                                    [-0.000444],
                                    [-0.000154],
                                    [ 111.778905],
                                    [ 9.622339],
                                    [ 110.937167],
                                    [ 116.349463],
                                    [ 0.000000],
                                    [ 0.000000],
                                    [ 0.000000],
                                    [ 0.000000],
                                    [ 0.000000],
                                    [ 0.000000]])

        self.intrinsic_camera = np.array([[self.cameraMatrix[0, 0], 0, self.cameraMatrix[0, 2]],
                              [0, self.cameraMatrix[1, 1], self.cameraMatrix[1, 2]],
                              [0, 0, 1]])

        self.distortion = self.distCoeffs

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 108)

        self.create_timer(0.01, self.image_callback)


    def image_callback(self):
        ret, img = self.cap.read()
        
        self.pose_estimation(img, self.aruco_dict[self.aruco_type], self.intrinsic_camera, self.distortion)

    def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, cv2.aruco_dict)


        if len(corners) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.02, matrix_coefficients, distortion_coefficients)
   
            for i in range(len(ids)):
                if rvecs[i] is not None and tvecs[i] is not None:
                    rvec = rvecs[i]
                    tvec = tvecs[i]
                    msg = Vector3()
                    msg.x = float(ids[i])
                    msg.y = tvec[i][2]
                    self.publisher_plant.publish(msg)
                    print(corners[i])

def main(args=None):
    rclpy.init(args=args)

    tank_camera = TankCamera()
    rclpy.spin(tank_camera)

    tank_camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
