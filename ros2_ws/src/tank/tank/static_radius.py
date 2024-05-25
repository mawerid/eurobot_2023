import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import scipy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

TIMER_DELAY = 0.1
MAP_MARKERS_COUNT = 4

MAP_MARKERS = [20, 21, 22, 23]
ROBOT_MARKER = 2
ENEMY_MARKER = 1

MIN_DISTANCE = 0.6
MAP_MARKER_LENGTH = 0.07
ROBOT_MARKER_LENGTH = 0.07

DISTANCE_BUFFER_SIZE = 5


class ArucoMapper(Node):
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

        self.intrinsic_camera = np.array([[1316.741948, 0.000000, 928.288568],
                                          [0.000000, 1344.516871, 544.438518],
                                          [0.000000, 0.000000, 1.000000]], dtype=np.float32)

        self.distortion = np.array([[-0.162078],
                                    [7.725812],
                                    [0.000708],
                                    [-0.002309],
                                    [-0.192252],
                                    [0.096461],
                                    [7.558903],
                                    [2.387815],
                                    [0.000000],
                                    [0.000000],
                                    [0.000000],
                                    [0.000000],
                                    [0.000000],
                                    [0.000000]], dtype=np.float32)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

        self.static_aruco = self.create_publisher(String, 'static_aruco', 10)
        self.robo_place = self.create_publisher(Pose, 'robot_place', 10)
        self.enemy_place = self.create_publisher(Pose, 'enemy_place', 10)
        self.obstacle = self.create_publisher(String, 'obstacle_info', 10)

        self.create_timer(TIMER_DELAY, self.image_callback)

        self.tf_cam2center = np.zeros((4, 4))
        self.tfs_map_markers = [None for _ in range(MAP_MARKERS_COUNT)]
        self.map_build_flag = False

        self.current_distance = []

    def calc_center(self):

        av_rotmat = np.zeros((3, 3))
        av_tvec = np.zeros((1, 3))

        for i in range(MAP_MARKERS_COUNT):
            av_rotmat += Rotation.from_rotvec(self.tfs_map_markers[i][0]).as_matrix()
            av_tvec += self.tfs_map_markers[i][1]

        av_rotmat = scipy.linalg.polar(av_rotmat / MAP_MARKERS_COUNT)[0]
        av_tvec /= MAP_MARKERS_COUNT

        self.tf_cam2center[:3, :3] = av_rotmat
        self.tf_cam2center[:3, 3] = av_tvec
        self.tf_cam2center[3, 3] = 1.0

        reverse = np.array([[-1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        self.tf_cam2center = np.matmul(reverse, self.tf_cam2center)

    def calc_tf(self, marker):
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker, ROBOT_MARKER_LENGTH,
                                                            self.intrinsic_camera,
                                                            self.distortion)

        rot_mat = Rotation.from_rotvec(rvec[0]).as_matrix()[0]
        tf_robo = np.array([[rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], tvec[0][0][0]],
                            [rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], tvec[0][0][1]],
                            [rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], tvec[0][0][2]],
                            [0.0, 0.0, 0.0, 1.0]])

        tf_robo = np.matmul(self.tf_cam2center, tf_robo)
        R = Rotation.from_matrix(tf_robo[:3, :3]).as_quat()
        tvec = tf_robo[:3, 3]

        print(tvec)

        # Create Pose message
        pose_msg = Pose()
        pose_msg.position.x = tvec[0]
        pose_msg.position.y = tvec[1]
        pose_msg.position.z = tvec[2]
        pose_msg.orientation.x = R[0]
        pose_msg.orientation.y = R[1]
        pose_msg.orientation.z = R[2]
        pose_msg.orientation.w = R[3]  # Assume unit quaternion for simplicity

        return tf_robo, pose_msg

    def image_callback(self):
        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        corners, ids, _ = detector.detectMarkers(gray)

        robot_marker = None
        enemy_marker = None
        map_corners = [None for _ in range(MAP_MARKERS_COUNT)]

        tf_robo, tf_enemy = np.zeros((4, 4)), np.zeros((4, 4))

        if len(corners) > 0:
            for i in range(len(ids)):
                if ids[i] == ROBOT_MARKER:
                    robot_marker = corners[i]
                elif ids[i] == ENEMY_MARKER:
                    enemy_marker = corners[i]
                elif ids[i] in MAP_MARKERS:  # Фильтрация маркеров по ID
                    for j in range(len(MAP_MARKERS)):
                        if ids[i] == MAP_MARKERS[j]:
                            map_corners[j] = corners[i]
                            break

        if sum(x is not None for x in map_corners) == MAP_MARKERS_COUNT and not self.map_build_flag:
            print(ids)
            for i in range(len(map_corners)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(map_corners[i], MAP_MARKER_LENGTH,
                                                                    self.intrinsic_camera,
                                                                    self.distortion)
                self.tfs_map_markers[i] = [rvec[0][0], tvec[0]]

            self.calc_center()

            self.map_build_flag = True

        if self.map_build_flag:

            if robot_marker is not None:
                tf_robo, pose_msg = self.calc_tf(robot_marker)
                self.enemy_place.publish(pose_msg)

            if enemy_marker is not None:
                tf_enemy, pose_msg = self.calc_tf(enemy_marker)
                self.robo_place.publish(pose_msg)

            if robot_marker is not None and enemy_marker is not None:
                new = tf_robo * np.linalg.inv(tf_enemy)
                distance = np.linalg.norm(new[:3, 3])

                self.current_distance.append(distance)
                if len(self.current_distance) == DISTANCE_BUFFER_SIZE:
                    distance = sum(self.current_distance) / len(self.current_distance)
                    print(distance)
                    obst = String()
                    if distance < MIN_DISTANCE:
                        obst.data = "stop_obstacle"
                        self.obstacle.publish(obst)
                    else:
                        obst.data = "move"
                        self.obstacle.publish(obst)
                    self.current_distance.pop(0)


def main(args=None):
    rclpy.init(args=args)
    mapper = ArucoMapper()

    rclpy.spin(mapper)
    mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
