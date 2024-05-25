import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import scipy


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

        self.intrinsic_camera = np.array([[ 1572.512703,  0.000000,  1892.212752],
                                            [ 0.000000,  1570.878744,  1107.524273],
                                            [ 0.000000,  0.000000,  1.000000]])

        self.distortion = np.array([[-4.455178],
                                    [ 6.454582],
                                    [-0.000104],
                                    [ 0.000088],
                                    [-0.131327],
                                    [-4.435619],
                                    [ 6.366330],
                                    [-0.013529],
                                    [ 0.000000],
                                    [ 0.000000],
                                    [ 0.000000],
                                    [ 0.000000],
                                    [ 0.000000],
                                    [ 0.000000]])

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, )

        self.static_aruco = self.create_publisher(String, 'static_aruco', 10)
        self.robo_place = self.create_publisher(Pose, 'robot_place', 10)
        self.enemy_place = self.create_publisher(Pose, 'enemy_place', 10)
        self.obstacle = self.create_publisher(String, 'obstacle_info', 10)

        self.create_timer(0.01, self.image_callback)

        self.transform_cam2center = np.zeros((4, 4))

        self.map_markers = [[None, None], [None, None], [None, None], [None, None]]

        self.all_markers_detected = False
        self.message_displayed = False

    def image_callback(self):
        ret, img = self.cap.read()

        # annotated_frame = pose_estimation(frame, ARUCO_DICT[aruco_type], cameraMatrix, distCoeffs)
        # cv2.imshow('Annotated Frame', annotated_frame)
        self.pose_estimation(img, self.aruco_dict[self.aruco_type], self.intrinsic_camera, self.distortion)

    def calc_center(self):

        av_rotmat = np.zeros((3, 3))
        av_tvec = np.zeros((1, 3))

        for i in range(4):
            av_rotmat += Rotation.from_rotvec(self.map_markers[i][0][0][0]).as_matrix()
            av_tvec += self.map_markers[i][1][0]

        av_rotmat = scipy.linalg.polar(av_rotmat / 4)[0]
        av_tvec /= 4

        self.transform_cam2center[:3, :3] = av_rotmat
        self.transform_cam2center[:3, 3] = av_tvec
        self.transform_cam2center[3, 3] = 1.0


    def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters()

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        marker_1 = None
        marker_2 = None
        map_corners = [None, None, None, None]

        tvec_1, tvec_2 = np.array([-1., -1., -1.]), np.array([-1., -1., -1.])
        #print(self.transform_cam2center)

        if len(corners) > 0:
            for i in range(len(ids)):

                if ids[i] == 1:
                    marker_1 = corners[i]
                elif ids[i] == 2:
                    marker_2 = corners[i]
                elif ids[i] in [20, 21, 22, 23]:  # Фильтрация маркеров по ID

                    print(ids[i])

                    if ids[i] == 20:
                        map_corners[0] = corners[i]
                    elif ids[i] == 21:
                        map_corners[1] = corners[i]
                    elif ids[i] == 22:
                        map_corners[2] = corners[i]
                    elif ids[i] == 23:
                        map_corners[3] = corners[i]

        
        if map_corners[0] is not None and map_corners[1] is not None and map_corners[2] is not None and map_corners[3] is not None and not self.message_displayed:
            self.all_markers_detected = True
            pose_msg = String()
            # print("All markers detected:", self.all_markers_detected)
            # print(self.marker_coordinates)
            print(ids)
            for i in range(4):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(map_corners[i], 0.07, matrix_coefficients,
                                                                    distortion_coefficients)
                cv2.aruco.drawDetectedMarkers(frame, corners)
                self.map_markers[i] = [rvec, tvec]
                #cv2.imshow('image', frame)

            self.calc_center()
            tvec = self.transform_cam2center[:3, 3]
            rvec = Rotation.from_matrix(self.transform_cam2center[:3, :3]).as_rotvec()
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1)

            cv2.imwrite("axes.png", frame)

            print(self.transform_cam2center)

            pose_msg.data = "Map done"
            self.static_aruco.publish(pose_msg)
            self.message_displayed = True

        if marker_1 is not None:
            rvec_1, tvec_1, _ = cv2.aruco.estimatePoseSingleMarkers(marker_1, 0.07, matrix_coefficients,
                                                                    distortion_coefficients)
            rot_mat = Rotation.from_rotvec(rvec_1[0][0]).as_matrix()
            tf_robo = np.array([[rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], tvec_1[0][0][0]],
                                [rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], tvec_1[0][0][1]],
                                [rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], tvec_1[0][0][2]],
                                [0.0, 0.0, 0.0, 1.0]])

            tf_new = np.matmul(self.transform_cam2center, tf_robo)
            R = Rotation.from_matrix(tf_new[:3, :3]).as_quat()
            tf_new = np.matmul(self.transform_cam2center, tf_robo)
            R = Rotation.from_matrix(tf_new[:3, :3]).as_quat()
            tvec_1 = tf_new[:3, 3]

            # Create Pose message
            pose_msg = Pose()
            pose_msg.position.x = tvec_1[0] / 100
            pose_msg.position.y = tvec_1[1] / 100
            pose_msg.position.z = tvec_1[2] / 100
            pose_msg.orientation.x = R[0]
            pose_msg.orientation.y = R[1]
            pose_msg.orientation.z = R[2]
            pose_msg.orientation.w = R[3]  # Assume unit quaternion for simplicity

            self.enemy_place.publish(pose_msg)

        if marker_2 is not None:
            rvec_2, tvec_2, _ = cv2.aruco.estimatePoseSingleMarkers(marker_2, 0.07, matrix_coefficients,
                                                                    distortion_coefficients)

            rot_mat = Rotation.from_rotvec(rvec_2).as_matrix()
            tf_robo = np.array([[rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], tvec_2[0][0][0]],
                                [rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], tvec_2[0][0][1]],
                                [rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], tvec_2[0][0][2]],
                                [0.0, 0.0, 0.0, 1.0]])

            tf_new = np.matmul(self.transform_cam2center, tf_robo)
            R = Rotation.from_matrix(tf_new[:3, :3]).as_quat(canonical=True)
            tvec_2 = tf_new[:3, 3]

            # Create Pose message
            pose_msg = Pose()
            pose_msg.position.x = tvec_2[0] / 100
            pose_msg.position.y = tvec_2[1] / 100
            pose_msg.position.z = tvec_2[2] / 100
            pose_msg.orientation.x = R[0]
            pose_msg.orientation.y = R[1]
            pose_msg.orientation.z = R[2]
            pose_msg.orientation.w = R[3]  # Assume unit quaternion for simplicity

            self.robo_place.publish(pose_msg)

        if tvec_1.sum() >= 0 and tvec_2.sum() >= 0:
            distance = np.linalg.norm(tvec_1 - tvec_2)
            # print(distance)

            obst = String()
            if distance < 0.2:
                obst.data = "stop_obstacle"
                self.obstacle.publish(obst)
            elif distance >= 0.2:
                obst.data = "move"
                self.obstacle.publish(obst)
            print(tvec_2)



def main(args=None):
    rclpy.init(args=args)
    static_radius = StaticRadius()

    rclpy.spin(static_radius)
    static_radius.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
