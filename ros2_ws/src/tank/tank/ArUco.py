import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class ArUcoDetection(Node):
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
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.create_timer(0.1, self.image_callback)

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

    def aruco_display(self, corners, ids, rejected, image):
        if len(corners) > 0:

            ids = ids.flatten()

            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

                print("[Inference] ArUco marker ID: {}".format(markerID))

        return image

    def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters()

        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict)

        if len(corners) > 0:
            for i in range(0, len(ids)):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                               distortion_coefficients)

                cv2.aruco.drawDetectedMarkers(frame, corners)

                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)


                cX = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                cY = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2)
                cZ = tvec[0][0][2]

                cv2.putText(frame, f"ID: {ids[i]} ({cX:.2f}, {cY:.2f}, {cZ:.2f})", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

        return frame

def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArUcoDetection()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

