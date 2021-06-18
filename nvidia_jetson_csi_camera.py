#!/usr/bin/python
import cv2 as cv
import threading
import numpy as np

def gstreamer_pipeline(
    sensor_id=0,
    sensor_mode=4,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            sensor_mode,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class NVIDIA_Jetson_CSI_Camera:

    def __init__ (self):
        self.running = False
        self.capture = None
        self.thread = None
        self.frame = None
        self.lock = threading.Lock()
        self.skipped_frames = 0

    def open(self, pipeline):
        self.capture = cv.VideoCapture(
            pipeline, cv.CAP_GSTREAMER
        )

        if not self.capture.isOpened():
            print("Unable to open camera")
            print("Pipeline: " + pipeline)
            exit(1)

        _, self.frame = self.capture.read()

    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None

        if self.capture != None:
            self.running=True
            self.thread = threading.Thread(target=self.loop)
            self.thread.start()
        return self

    def stop(self):
        self.running=False
        self.thread.join()

    def release(self):
        if self.capture != None:
            self.capture.release()
            self.capture = None
        if self.thread != None:
            self.thread.join()

    def read(self):
        if self.running != True:
            print("Vidoe capture not running")
            return None, None
        with self.lock:
            frame = self.frame.copy()
            skipped = self.skipped_frames.copy()
            self.skipped_frames = 0
        return skipped, frame

    def loop(self):
        while self.running:
            try:
                _, frame = self.capture.read()
                with self.lock:
                    self.frame = frame
                    self.skipped_frames += 1
            except RuntimeError:
                print("Could not read image from camera")
                self.release()

if __name__ == "__main__":
    
    camera = NVIDIA_Jetson_CSI_Camera()
    camera.open(
        gstreamer_pipeline(
            sensor_id=0,
            sensor_mode=4,
            flip_method=2,
            display_height=540,
            display_width=960,
        )
    )

    camera.start()

    while True:

        _ , img = camera.read()
        cv.imshow("camera", img)

        if cv.waitKey(30) & 0xFF == 'q':
            break

    camera.stop()
    camera.release()
    cv.destroyAllWindows()
