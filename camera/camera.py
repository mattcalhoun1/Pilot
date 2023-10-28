# this is an interface to be implemented by any class that 
# can function as a navigation camera
class Camera:
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
        pass

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
        pass
