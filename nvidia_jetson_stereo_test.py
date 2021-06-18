#!/usr/bin/python
from  nvidia_jetson_csi_camera import NVIDIA_CSI_Camera as Camera
import numpy as np
import cv2 as cv

if __name__ == "__main__":
    camera_left = Camera()
    camera_right = Camera()

    camera_left.open(
        Camera.gstreamer_pipeline(
            sensor_id=0,
            sensor_mode=4,
            flip_method=2,
            display_height=540,
            display_width=960,
        )
    )

    camera_right.open(
        Camera.gstreamer_pipeline(
            sensor_id=1,
            sensor_mode=4,
            flip_method=2,
            display_height=540,
            display_width=960,
        )
    )

    cv.namedWindow("Cameras", cv.WINDOW_AUTOSIZE)

    camera_left.start()
    camera_right.start()

    cTime = 0
    pTime = 0

    while True:

        _, _, img_left = camera_left.read()
        _, _, img_right = camera_right.read()

        img = np.hstack((img_left, img_right))

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
