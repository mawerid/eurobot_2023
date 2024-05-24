import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

class StaticRadius(Node):
    def __init__(self):
        super().__init__('static_radius')

        self.aruco_dict = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
        }
        self.aruco_type = "DICT_4X4_50"
        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.aruco_dict[self.aruco_type])
        self.arucoParams = cv2.aruco.DetectorParameters()

        self.cameraMatrix = np.array([[ 1558.756635,  0.000000,  1981.595037],
                                        [ 0.000000,  1594.619169,  1163.131045],
                                         [ 0.000000,  0.000000,  1.000000]])

        self.distCoeffs = np.array([[ 46.444281],
                                    [-101.568717],
                                    [ 0.001139],
                                    [ 0.000560],
                                    [ 84.387716],
                                    [ 46.815570],
                                    [-102.549148],
                                    [ 85.284062],
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
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 108)

        self.static_aruco = self.create_publisher(Vector3, 'static_aruco', 10)
        self.place = self.create_publisher(Vector3, 'robot_place', 10)
        self.obstacle = self.create_publisher(String, 'obstacle_info',10)
        self.enemy = self.create_publisher(Vector3, 'enemy_place',10)

        self.create_timer(0.01, self.image_callback)

        self.marker_coordinates = np.zeros(12)

        self.all_markers_detected = False
        self.message_displayed = False

        self.message_sent = False

    def image_callback(self):
        ret, img = self.cap.read()
        self.pose_estimation(img, self.aruco_dict[self.aruco_type], self.intrinsic_camera, self.distortion)

    def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters()

        corners, ids, _ = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

        marker_1 = None
        marker_2 = None

        if len(corners) > 0:
            for i in range(len(ids)):
                if ids[i] == 1:
                    marker_1 = corners[i]
                elif ids[i] == 2:
                    marker_2 = corners[i]

                if ids[i] in [20, 21, 22, 23]:  # Фильтрация маркеров по ID
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.12 , matrix_coefficients,
                                                                                   distortion_coefficients)

                    cv2.aruco.drawDetectedMarkers(frame, corners)

                    cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

                    # Отображение координат маркера
                    cX = float((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                    cY = float((corners[i][0][0][1] + corners[i][0][2][1]) / 2)

                    if ids[i] == 20:
                        self.marker_coordinates[0] = 20.
                        self.marker_coordinates[1] = cX
                        self.marker_coordinates[2] = cY
                    elif ids[i] == 21:
                        self.marker_coordinates[3] = 21.
                        self.marker_coordinates[4] = cX
                        self.marker_coordinates[5] = cY
                    elif ids[i] == 22:
                        self.marker_coordinates[6] = 22.
                        self.marker_coordinates[7] = cX
                        self.marker_coordinates[8] = cY
                    elif ids[i] == 23:
                        self.marker_coordinates[9] = 23.
                        self.marker_coordinates[10] = cX
                        self.marker_coordinates[11] = cY


        rvec_1, tvec_1, _ = cv2.aruco.estimatePoseSingleMarkers(marker_1, 0.07, matrix_coefficients,
                                                                      distortion_coefficients)
        rvec_2, tvec_2, _ = cv2.aruco.estimatePoseSingleMarkers(marker_2, 0.09, matrix_coefficients,
                                                                      distortion_coefficients)

        # Отображение координат маркеров
        if marker_1 is not None:
            cX_1 = float((marker_1[0][0][0] + marker_1[0][2][0]) / 2)
            cY_1 = float((marker_1[0][0][1] + marker_1[0][2][1]) / 2)
            enem = Vector3()
            enem.x = cX_1
            enem.y = cY_1
            self.enemy.publish(enem)

        if marker_2 is not None:
            cX_2 = float((marker_2[0][0][0] + marker_2[0][2][0]) / 2)
            cY_2 = float((marker_2[0][0][1] + marker_2[0][2][1]) / 2)
            pose = Vector3()
            pose.x = cX_2
            pose.y = cY_2
            self.place.publish(pose)
            #print(pose)

        if marker_1 is not None and marker_2 is not None:
            distance = np.linalg.norm(tvec_1[0][0][:2] - tvec_2[0][0][:2])
        elif marker_2 is not None:
            distance = 10000
        else:
            distance = 10000
 
        obst = String()
        if distance < 100:
            obst.data = "stop_obstacle"
            self.obstacle.publish(obst)
        elif distance >= 100:
            obst.data = "move"
            self.obstacle.publish(obst)

        if all(self.marker_coordinates) and not self.message_displayed:
            self.all_markers_detected = True
            msg = Vector3()
            #print("All markers detected:", self.all_markers_detected)
            #print(self.marker_coordinates)
            self.message_displayed = True
            print(ids)
            for i in range(len(ids)):
                if ids[i] == 20:
                    msg.x = self.marker_coordinates[0]
                    msg.y = self.marker_coordinates[1]
                    msg.z = self.marker_coordinates[2]
                    self.static_aruco.publish(msg) 
                if ids[i] == 21:
                    msg.x = self.marker_coordinates[3]
                    msg.y = self.marker_coordinates[4]
                    msg.z = self.marker_coordinates[5]
                    self.static_aruco.publish(msg) 
                if ids[i] == 22:
                    msg.x = self.marker_coordinates[6]
                    msg.y = self.marker_coordinates[7]
                    msg.z = self.marker_coordinates[8]
                    self.static_aruco.publish(msg) 
                if ids[i] == 23:
                    msg.x = self.marker_coordinates[9]
                    msg.y = self.marker_coordinates[10]
                    msg.z = self.marker_coordinates[11]
                    self.static_aruco.publish(msg) 
            print(self.marker_coordinates)    
            #msg.data = [self.marker_coordinates]                   


def main(args=None):
    rclpy.init(args=args)
    static_radius = StaticRadius()

    rclpy.spin(static_radius)
    static_radius.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
