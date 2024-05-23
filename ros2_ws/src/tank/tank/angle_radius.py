import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from geometry_msgs.msg import Vector3

class ArucoDetection(Node):
    def __init__(self):
        super().__init__('aruco_detection')

        self.aruco_dict = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
        }
        self.aruco_type = "DICT_4X4_100"
        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.aruco_dict[self.aruco_type])
        self.arucoParams = cv2.aruco.DetectorParameters()

        self.cameraMatrix = np.array([[6.90664227e+02, 0.00000000e+00, 3.42091111e+02],
                                      [0.00000000e+00, 6.90056355e+02, 3.95069510e+02],
                                      [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        self.distCoeffs = np.array([-6.66405451e-03, -5.28018174e-01, 2.91089476e-02, 3.44690910e-03, 1.10418972e+00])

        self.intrinsic_camera = np.array([[self.cameraMatrix[0, 0], 0, self.cameraMatrix[0, 2]],
                                          [0, self.cameraMatrix[1, 1], self.cameraMatrix[1, 2]],
                                          [0, 0, 1]])

        self.distortion = np.array([self.distCoeffs[0], self.distCoeffs[1], self.distCoeffs[2], self.distCoeffs[3]])

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 108)
        self.publisher_aim = self.create_publisher(Vector3, 'aim_topic', 10)

        self.create_timer(0.01, self.image_callback)


    def image_callback(self):
        ret, img = self.cap.read()
        h, w, _ = img.shape
        width = 1000
        height = int(width * (h / w))
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)

        corners, ids, rejected = cv2.aruco.detectMarkers(img, self.arucoDict, parameters=self.arucoParams)
        detected_markers = self.aruco_display(corners, ids, rejected, img)

        output = self.pose_estimation(img, self.aruco_dict[self.aruco_type], self.intrinsic_camera, self.distortion)

        combined_image = cv2.addWeighted(detected_markers, 0.5, output, 0.5, 0)



        cv2.imshow("Combined Image", combined_image)
        cv2.waitKey(1)

    def aruco_display(self, corners, ids, image):
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2)).astype(int)
                for i in range(4):
                    cv2.line(image, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
                cX = int(np.mean(corners[:, 0]))
                cY = int(np.mean(corners[:, 1]))
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                self.get_logger().info("[Inference] ArUco marker ID: %d" % markerID)
        return image

    def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
	    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
	    parameters = cv2.aruco.DetectorParameters()

	    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict)

	    if len(corners) > 0:
		rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.02, matrix_coefficients,
		                                                      distortion_coefficients)

		index_marker_5 = None
		index_marker_18 = None
		index_marker_12 = None
		index_marker_42 = None

		for i in range(len(ids)):
		    cv2.aruco.drawDetectedMarkers(frame, corners)

		    if rvecs[i] is not None and tvecs[i] is not None:
		        rvec = rvecs[i]
		        tvec = tvecs[i]
		        cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.03)

		        center_marker = np.mean(corners[i][0], axis=0)

		        cv2.circle(frame, tuple(center_marker[:2].astype(int)), 5, (0, 0, 255),
		                   -1)

		        if ids[i] == 5:
		            index_marker_5 = i

		        elif ids[i] == 18:
		            index_marker_18 = i

		        elif ids[i] == 12:
		            index_marker_12 = i

		        elif ids[i] == 42:
		            index_marker_42 = i

		if index_marker_5 is not None and index_marker_18 is not None and index_marker_12 is not None and index_marker_42 is not None:
		    center_marker_5 = tvecs[index_marker_5][0]
		    center_marker_18 = tvecs[index_marker_18][0]
		    center_marker_12 = tvecs[index_marker_12][0]
		    center_marker_42 = tvecs[index_marker_42][0]

		    vector_12_to_18 = center_marker_18 - center_marker_12
		    vector_2_to_42 = center_marker_42 - center_marker_5


		    cv2.arrowedLine(frame, tuple(center_marker_12[:2].astype(int)), tuple(center_marker_18[:2].astype(int)),
		                    (255, 0, 0), 2)

		    cv2.arrowedLine(frame, tuple(center_marker_5[:2].astype(int)), tuple(center_marker_42[:2].astype(int)),
		                    (0, 255, 0), 2)


		    angle_rad = np.arccos(np.dot(vector_12_to_18, vector_2_to_42) / (
		                np.linalg.norm(vector_12_to_18) * np.linalg.norm(vector_2_to_42)))
		    angle_deg = np.degrees(angle_rad)


		    print("Угол между векторами 12-18 5-42:", angle_deg)


		    distance_2_to_42 = np.linalg.norm(center_marker_42 - center_marker_5)

		    print("Расстояние между маркерами 5 и 42:", distance_2_to_42)

	    return frame

def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoDetection()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
