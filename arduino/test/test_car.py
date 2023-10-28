# mock car to be used for unit test purposes
class TestCar:
    def __init__(self, left_cam_starting_rotation = 0, right_cam_starting_rotation = 0):
        self.__camera_configs = {
            '0': {
                'rotation':left_cam_starting_rotation,
                'tilt':90,
                'min_rotation':0,
                'max_rotation':0,
                'min_tilt':0,
                'max_tilt':0
            },
            '1': {
                'rotation':right_cam_starting_rotation,
                'tilt':90,
                'min_rotation':0,
                'max_rotation':0,
                'min_tilt':0,
                'max_tilt':0
            }
        }

    def is_streaming_lidar (self):
        return self.__streaming_lidar
    
    def is_streaming_camera_info(self):
        return self.__streaming_camera_info

    def get_all_configurations (self, timeout : float = 10.0):
        return True

    def get_config (self, config_key):
        True

    def get_cameras (self, timeout : float = 10.0, force_refresh = False):
        return self.__camera_configs
    

    def handle_message(self, msg):
        pass

    def get_live_lidar_map (self, timeout = 10):
        return None

    def get_lidar_map (self, wait_for_result = False):
        True

    def rotate (self, degrees : float, wait_for_result = False):
        return True
    
    def stop (self):
        return True

    def look (self, rotation : int, tilt : int, wait_for_result = False):
        return True

    def look_multi (self, angles, wait_for_result = False):
        return True

    def go (self, speed = 1, duration_millis = 1000):
        return True

    def forward_distance (self, speed = 1, distance_units = 10, wait_for_result = False):
        return True

    def reverse_distance (self, speed = 1, distance_units = 10, wait_for_result = False):
        return True

    def strafe_left (self, wait_for_result = False):
        return True

    def strafe_right (self, wait_for_result = False):
        return True

    def find_measurement (self, degrees: float, angle_tolerance: float, expected_distance : float, distance_tolerance : float, max_age : int, timeout = 10):
        return None,None

    def measure (self, degrees: float, tolerance: float, timeout = 10):
        return None,None

    def display_mode (self, mode : str, wait_for_result = False):
        return True

    def display_status (self, status : str, wait_for_result = False):
        return True

    def display_command (self, command : str, wait_for_result = False):
        return True

    def display_position (self, x: float, y:float, heading: float, wait_for_result = False):
        return True

    def display_objects (self, obj_dist_map, wait_for_result = False):
        return True
