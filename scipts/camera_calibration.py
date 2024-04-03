import numpy as np
import cv2
import matplotlib.pyplot as plt
from cv2 import aruco
import matplotlib as mpl
import tqdm
import pathlib
from typing import Tuple


def generate_charuco_board(charuco_board: str, charuco_board_dim: str, charuco_save_dir, plot_results=False) \
        -> Tuple[cv2.aruco.CharucoBoard, cv2.aruco.Dictionary]:
    margin_pixels = 40
    aruco_dict = aruco.getPredefinedDictionary(eval(charuco_board))
    board = aruco.CharucoBoard(eval(charuco_board_dim), 0.0645, 0.0485, aruco_dict)
    board.setLegacyPattern(True)
    board_image = cv2.aruco.CharucoBoard.generateImage(board, (1000, 1000), marginSize=margin_pixels)
    cv2.imwrite(charuco_save_dir + "chessboard_{}_{}.tiff".format(charuco_board, charuco_board_dim.replace(",", '_')),
                board_image)

    if plot_results:
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        plt.imshow(board_image, cmap=mpl.cm.gray, interpolation="nearest")
        ax.axis("off")
        plt.show()

    return board, aruco_dict


def read_chessboards(images, board: cv2.aruco.CharucoBoard, aruco_dict: cv2.aruco.Dictionary, save_dir=None,
                     show_image=False):
    """
    Charuco base pose estimation.
    """
    print("start pose estimations:")
    all_corners = []
    all_ids = []
    image_count = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
    image_size = None
    for im in tqdm.tqdm(images):
        # print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners) > 0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(3, 3),
                                 zeroZone=(-1, -1),
                                 criteria=criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 6 and image_count % 1 == 0:
                all_corners.append(res2[1])
                all_ids.append(res2[2])

        image_count += 1

        ar_image = aruco.drawDetectedMarkers(frame, corners, ids)

        image_size = gray.shape

        cv2.imwrite(str(save_dir.joinpath(pathlib.Path(im).name)), ar_image)
        if show_image:
            plt.imshow(ar_image[:, :, ::-1])
            plt.show()

    return all_corners, all_ids, image_size


def calibrate_camera(allCorners, allIds, imsize, board: cv2.aruco.CharucoBoard):
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[1000., 0., imsize[0] / 2.],
                                 [0., 1000., imsize[1] / 2.],
                                 [0., 0., 1.]])

    distCoeffsInit = np.zeros((5, 1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    # flags = (cv2.CALIB_RATIONAL_MODEL)

    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=allCorners,
        charucoIds=allIds,
        board=board,
        imageSize=imsize,
        cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


def camera_to_world_transformation(rvec, tvec):
    # Convert rvec to a rotation matrix R
    R, _ = cv2.Rodrigues(rvec)

    # Inverse of the rotation is simply its transpose
    R_inv = np.transpose(R)

    # Inverse of the translation
    t_inv = -R_inv @ tvec

    return R_inv, t_inv


# def visualize_cameras(rvecs, tvecs, mtx, plot_limit=200):
#     fig = viz_3d.init_figure()
#
#     count = 0
#     for rvec, tvec in zip(rvecs, tvecs):
#         rvec, tvec = camera_to_world_transformation(np.squeeze(rvec), np.squeeze(tvec))
#
#         if count < plot_limit:
#             viz_3d.plot_camera(fig, rvec, tvec, mtx, color='rgba(255,255,0,0.5)', size=1, text="test")
#         count += 1
#
#     fig.show()
# run calibration in one function, but we will break it down later. You can ignore this part
def run_calibration():
    work_dir = "../data/tank_camera/calibration"
    pathlib.Path(work_dir).mkdir(exist_ok=True)

    charuco_board = "aruco.DICT_5X5_250"
    charuco_board_dim = "(9,6)"

    data_dir = "../data/tank_camera/"

    board, aruco_dict = generate_charuco_board(charuco_board, charuco_board_dim, work_dir, plot_results=False)

    # list all the jpeg files in the image directory
    image_dir = pathlib.Path(data_dir)
    image_paths = list(map(str, (image_dir.glob("*.jpg"))))
    print("The first 10 image paths:{}", image_paths[0:10])

    save_dir = pathlib.Path(work_dir).joinpath("aruco_detected_results")
    save_dir.mkdir(exist_ok=True)

    allCorners, allIds, imsize = read_chessboards(image_paths, board, aruco_dict, save_dir=save_dir)

    ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners, allIds, imsize, board)

    np.set_printoptions(suppress=True)
    print("Camera matrix: \n", mtx)
    print("Distortion coefficients: \n", dist.T[0])
    # print("Rotation vectors: \n", rvecs)
    # print("Translation vectors: \n", tvecs)

    # visualize_cameras(rvecs, tvecs, mtx)


run_calibration()
