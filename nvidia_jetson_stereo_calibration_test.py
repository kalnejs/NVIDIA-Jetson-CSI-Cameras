#!/usr/bin/python
from  nvidia_jetson_csi_camera import NVIDIA_CSI_Camera as Camera
import numpy as np
import cv2 as cv
import os

if __name__ == "__main__":

    cal_file = cv.FileStorage("CameraCal.xml", cv.FileStorage_READ)

    mtx_left = cal_file.getNode("CameraMatrixLeft").mat()
    dist_left = cal_file.getNode("DistortionLeft").mat()
    mtx_right = cal_file.getNode("CameraMatrixRight").mat()
    dist_right = cal_file.getNode("DistortionRight").mat()
    cal_file.release()

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

        h, w = img_left.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx_left, dist_left, (w,h), 1, (w,h))
        mapx, mapy = cv.initUndistortRectifyMap(mtx_left, dist_left, None, newcameramtx, (w,h), 5)
        dst_left = cv.remap(img_left, mapx, mapy, cv.INTER_LINEAR , borderMode=cv.BORDER_CONSTANT)

        h, w = img_right.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx_right, dist_right, (w,h), 1, (w,h))
        mapx, mapy = cv.initUndistortRectifyMap(mtx_right, dist_right, None, newcameramtx, (w,h), 5)
        dst_right = cv.remap(img_right, mapx, mapy, cv.INTER_LINEAR , borderMode=cv.BORDER_CONSTANT)

        img = np.hstack((dst_left, dst_right))

        cv.imshow("Cameras", img)

        if cv.getWindowProperty("Cameras", cv.WINDOW_AUTOSIZE) == -1:
            break;

        if cv.waitKey(1) & 0xFF == 'q':
            break

    camera_left.stop()
    camera_left.release()
    camera_right.stop()
    camera_right.release()
    cv.destroyAllWindows()
