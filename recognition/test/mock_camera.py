from camera.camera import Camera
import cv2
import numpy as np

# emulates a camera by reading images from the resources directory
class MockCamera (Camera):
    def __init__(self, image_file):
        self.__image_file = image_file
        
    # sets focus on the given meters.
    # the optional next_direction_closer sets the flag for future
    # focus search to go closer or more distant. If you wnt it to search closer on, set to False
    def set_focus_meters (self, meters : int, next_direction_closer : bool):
        pass

    # if doing a focus search, this will set to the next setp of the search
    def set_next_focus (self):
        pass

    # returns the distance of current focus, in meters
    def get_current_focus (self) -> int:
        pass

    # starts the camera
    def start_camera (self):
        pass

    # captures an image buffer
    def capture_image (self):
        # read the given file into a buffer
        im = cv2.imread(self.__image_file,cv2.IMREAD_UNCHANGED)#[:,:,-1]
        return im

    # stops the camera
    def stop_camera (self):
        pass

    # cleans up any resources
    def release(self):
        pass

    # sets focus, finds (if necessary) optimal settings, and applies optimal settings
    # needs to be pointed at some objects during this optimization, or default settings will be used
    def optimize_for_distance (self, meters : int, force_search : bool, match_threshold_pct : float, expected_num_objects : int, object_ids : list, min_confidence : float):
        pass
  
    def preprocess_image (self, image):
        return image
