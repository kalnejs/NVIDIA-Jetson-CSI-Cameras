#!/usr/bin/python
from  nvidia_jetson_csi_camera import NVIDIA_CSI_Camera as Camera
import numpy as np
import cv2 as cv
import os

chessboard_cols = 9
chessboard_rows = 6

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp_left = np.zeros((chessboard_cols*chessboard_rows,3), np.float32)
objp_left[:,:2] = np.mgrid[0:chessboard_cols,0:chessboard_rows].T.reshape(-1,2)

objp_right = np.zeros((chessboard_cols*chessboard_rows,3), np.float32)
objp_right[:,:2] = np.mgrid[0:chessboard_cols,0:chessboard_rows].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints_left = [] # 3d point in real world space
imgpoints_left = [] # 2d points in image plane.

objpoints_right = [] # 3d point in real world space
imgpoints_right = [] # 2d points in image plane.

saved_cals = 0

def doChessBoardSearch(frame_left, frame_right):

    global saved_cals

    gray_left = cv.cvtColor(frame_left, cv.COLOR_BGR2GRAY)
    gray_right = cv.cvtColor(frame_right, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret_left, corners_left = cv.findChessboardCorners(gray_left, (chessboard_cols,chessboard_rows), None)
    ret_right, corners_right = cv.findChessboardCorners(gray_right, (chessboard_cols,chessboard_rows), None)
    # If found, add object points, image points (after refining them)
    if ret_left and ret_right:

        corners2_left = cv.cornerSubPix(gray_left, corners_left, (11,11), (-1,-1), criteria)
        corners2_right = cv.cornerSubPix(gray_right, corners_right, (11,11), (-1,-1), criteria)

        # Draw and display the corners
        cv.drawChessboardCorners(frame_left, (chessboard_cols,chessboard_rows), corners2_left, ret_left)
        cv.drawChessboardCorners(frame_right, (chessboard_cols,chessboard_rows), corners2_right, ret_right)

        key = cv.waitKey(30)

        if (key & 0xFF) == ord("c"):

            saved_cals += 1
            print("Saved :" + str(saved_cals))

            objpoints_left.append(objp_left)
            imgpoints_left.append(corners_left)

            objpoints_right.append(objp_right)
            imgpoints_right.append(corners_right)

    img = np.hstack((frame_left, frame_right))

    cv.imshow("Cameras", img)

if __name__ == "__main__":

    camera_left = Camera()
    camera_right = Camera()

    # camera_left.open(
    #     Camera.gstreamer_pipeline(
    #         sensor_id=0,
    #         sensor_mode=4,
    #         flip_method=2,
    #         display_height=1280,
    #         display_width=720,
    #     )
    # )
    #
    # camera_right.open(
    #     Camera.gstreamer_pipeline(
    #         sensor_id=1,
    #         sensor_mode=4,
    #         flip_method=2,
    #         display_height=1280,
    #         display_width=720,
    #     )
    # )

    camera_left.open("/dev/video0", False)
    camera_right.open("/dev/video3", False)

    camera_left.start()
    camera_right.start()


    while True:
        _, _, img_left = camera_left.read()
        _, _, img_right = camera_right.read()

        if saved_cals < 20:
            doChessBoardSearch(img_left, img_right)
        else:
            cv.destroyAllWindows()

            gray_left = cv.cvtColor(img_left, cv.COLOR_BGR2GRAY)
            gray_right = cv.cvtColor(img_right, cv.COLOR_BGR2GRAY)

            _, mtx_left, dist_left, _, _ = cv.calibrateCamera(objpoints_left, imgpoints_left, gray_left.shape[::-1], None, None)
            _, mtx_right, dist_right, _, _ = cv.calibrateCamera(objpoints_right, imgpoints_right, gray_right.shape[::-1], None, None)

            cal_file = cv.FileStorage("CameraCal.xml", cv.FileStorage_WRITE)
            cal_file.write("CameraMatrixLeft", mtx_left)
            cal_file.write("DistortionLeft", dist_left)
            cal_file.write("CameraMatrixRight", mtx_right)
            cal_file.write("DistortionRight", dist_right)
            cal_file.release()
            break;

        # img = np.hstack((img_left, img_right))
        #
        # cv.imshow("Cameras", img)

        if cv.getWindowProperty("Cameras", cv.WINDOW_AUTOSIZE) == -1:
            break;

        if cv.waitKey(1) & 0xFF == 'q':
            break

    camera_left.stop()
    camera_left.release()
    camera_right.stop()
    camera_right.release()
    cv.destroyAllWindows()
