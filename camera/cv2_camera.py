from camera.camera import Camera
import cv2
import threading
import numpy as np
import logging

class CV2Camera (Camera):
    __read_lock = threading.Lock()

    def __init__(self, high_res = False, flipped = True, default_focus_distance = 5, auto_optimize = True, auto_optimize_object_locator = None, sensor_id = 0):
        self.__sensor_id = sensor_id
        self.__res_w = 1640 if high_res == True else 1920
        self.__res_h = 1232 if high_res == True else 1080
        self.__frame_rate = 10
        self.__flipped = flipped
        
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        #self.read_lock = threading.Lock()
        self.running = False
        
        self.__open(self.__gstreamer_pipeline())
        self.start_camera()

    # no additional processing necessary
    def preprocess_image (self, image):
        return image

    # captures an image buffer
    def capture_image (self):
        grabbed, frame = self.read()
        #if grabbed:
        #    logging.getLogger(__name__).info(f"Image shape: {frame.shape}")
        #    cv2.imwrite("/tmp/cv2img.png", frame)
        return frame

    def __open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            # Grab the first frame to start the video capturing
            self.grabbed, self.frame = self.video_capture.read()

        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)

    def start_camera(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop_camera (self):
        self.running = False
        # Kill the thread
        self.read_thread.join()
        self.read_thread = None

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with CV2Camera.__read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened

    def read(self):
        grabbed = None
        frame = None
        with CV2Camera.__read_lock:
            if self.frame is not None:
                frame = self.frame.copy()
                grabbed = self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()


    def __gstreamer_pipeline(self):
        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                self.__sensor_id,
                self.__res_w,
                self.__res_h,
                self.__frame_rate,
                2 if self.__flipped else 0,
                self.__res_w,
                self.__res_h,
            )
        )



    def __gstreamer_pipeline_old(self):
        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                self.__sensor_id,
                self.__res_w,
                self.__res_h,
                self.__frame_rate,
                2 if self.__flipped else 0,
                self.__res_w,
                self.__res_h,
            )
        )
