import numpy as np
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import os
import sys

import penguinPiC
ppi = penguinPiC.PenguinPi()


def camera_calibration(dataDir):
    # This file can be used to generate camera calibration parameters
    # to improve the default values

    fileNameK = "{}intrinsic.txt".format(dataDir)
    fileNameD = "{}distCoeffs.txt".format(dataDir)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    board = aruco.CharucoBoard_create(3,3,0.94,0.34,aruco_dict)

    allCorners = []
    allIds = []
    decimator = 0

    images = np.array([dataDir + f for f in os.listdir(dataDir) if f.endswith(".png")])
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    for im in images:
        print("=> Processing image {0}".format(im))
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
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator % 1 == 0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        print("Image: {}/{}".format(decimator + 1, len(images)))
        print("Corners found: {}".format(len(corners)))
        decimator += 1

    imsize = gray.shape
    print("\n")
    print("Checkerboard detected in: {}/{} images".format(len(allCorners), decimator))

    cameraMatrixInit = np.array([[1000., 0., imsize[0] / 2.],
                                 [0., 1000., imsize[1] / 2.],
                                 [0., 0., 1.]])

    distCoeffsInit = np.zeros((5, 1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors, _, _, _) = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=allCorners,
        charucoIds=allIds,
        board=board,
        imageSize=imsize,
        cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    np.savetxt(fileNameK, camera_matrix, delimiter=',')
    np.savetxt(fileNameD, distortion_coefficients0, delimiter=',')

    i = 5  # select image id
    plt.figure()
    frame = cv2.imread(images[i])
    img_undist = cv2.undistort(frame, camera_matrix, distortion_coefficients0, None)
    plt.subplot(1, 2, 1)
    plt.imshow(frame)
    plt.title("Raw image")
    plt.axis("off")
    plt.subplot(1, 2, 2)
    plt.imshow(img_undist)
    plt.title("Corrected image")
    plt.axis("off")
    plt.show()


def image_collection(dataDir, images_to_collect):
    for i in range(images_to_collect):
        input(i)
        image = ppi.get_image()
        filename = "{}{}.png".format(dataDir, i)
        cv2.imwrite(filename, image)


if __name__ == "__main__":
    currentDir = os.getcwd()
    dataDir = "{}/camera_calibration/".format(currentDir)
    if not os.path.exists(dataDir):
        os.makedirs(dataDir)
    images_to_collect = 20

    # collect data
    print('Collecting {} images for camera calibration.'.format(images_to_collect))
    print('Press ENTER to capture image.')
    image_collection(dataDir, images_to_collect)
    print('Finished image collection.\n')

    # calibrate camera
    print('Calibrating camera...')
    camera_calibration(dataDir)
    print('Finished camera calibration.')

