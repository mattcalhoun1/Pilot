from arduino.arduino import Arduino
from arduino.arduino_constants import ArduinoConstants
from lidar.lidar_map import LidarMap
import logging
import time

class Observer(Arduino):
    def __init__(self):
        self.__lidar_offset = None
        self.__lidar_granularity = None
        self.__camera_configs = {}
        self.__streaming_lidar = False # this was intended to make ps4 user not have to wait as long, but causes locks in some situations
        self.__streaming_camera_info = False

        connected = False
        for trial_port in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']:
            try:
                logging.getLogger(__name__).info(f"Trying connection on {trial_port}")
                Arduino.__init__(self, usb_port = trial_port, baud = 115200, timeout = 1)
                logging.getLogger(__name__).info(f"Connected to Arduino on {trial_port}")
                connected = True
            except Exception as e:
                logging.getLogger(__name__).info(f"Failed to connect on {trial_port}: {e}")
            if connected:
                break
        if not connected:
            logging.getLogger(__name__).error("Failed to connect to vehicle!")
            raise Exception("Arduino connection error")

    def is_streaming_lidar (self):
        return self.__streaming_lidar
    
    def is_streaming_camera_info(self):
        return self.__streaming_camera_info

    def get_all_configurations (self, timeout : float = 10.0):
        if self.__lidar_offset is None or self.__lidar_granularity is None:
            start_time = time.time()

            while time.time() - start_time < timeout:
                if self.has_message():
                    msg = self.get_message()
                    if msg.startswith(ArduinoConstants.MESSAGE_PREFIX_CONFIG):
                        logging.getLogger(__name__).info(f"Received config: {msg}")
                        config_entry = msg.split(':')[1]
                        config_key = config_entry.split('|')[0]
                        config_val = config_entry.split('|')[1]
                        if config_key == 'LidarHeading':
                            self.__lidar_offset = int(config_val)
                        elif config_key == 'LidarGranularity':
                            self.__lidar_granularity = float(config_val)
                    #else:
                    #    logging.getLogger(__name__).info(f"Received unexpected message: {msg}")
                else:
                    time.sleep(0.25)

                if self.__lidar_offset is None:
                    self.get_config("LidarHeading")
                elif self.__lidar_granularity is None:
                    self.get_config("LidarGranularity")
                else:
                    break
        
        return self.__lidar_offset is not None and self.__lidar_granularity is not None

    def get_config (self, config_key):
        self.send_message(f"GetConfig:{config_key}")
        return self.wait_for_result(10)

    def get_cameras (self, timeout : float = 10.0, force_refresh = False):
        if self.__streaming_camera_info:
            return self.__camera_configs
        else:
            self.send_message(f"GetCameras:none")
            start_time = time.time()

            if force_refresh or len(self.__camera_configs) == 0:
                while time.time() - start_time < timeout:
                    if self.has_message():
                        msg = self.get_message()
                        if msg.startswith(ArduinoConstants.MESSAGE_PREFIX_CAMERAS):
                            logging.getLogger(__name__).info(f"Received cameras: {msg}")

                            # Cameras:[rotation1]|[tilt1]|[minrotation1]|[maxrotation1]|[mintilt1]|[maxtilt1],[rotation2]|...
                            full_cam_config = msg.split(':')[1]
                            all_cams = full_cam_config.split(',')
                            for cam_id, cam in enumerate(all_cams):
                                this_cam = cam.split('|')
                                self.__camera_configs[cam_id] = {
                                    'rotation':int(this_cam[0]),
                                    'tilt':int(this_cam[1]),
                                    'min_rotation':int(this_cam[2]),
                                    'max_rotation':int(this_cam[3]),
                                    'min_tilt':int(this_cam[4]),
                                    'max_tilt':int(this_cam[5]),
                                }
                        #else:
                        #    logging.getLogger(__name__).info(f"Received unexpected message: {msg}")
                    else:
                        time.sleep(0.25)

            return self.__camera_configs
    
    

    def handle_message(self, msg):
        if msg.startswith(ArduinoConstants.MESSAGE_PREFIX_LIDAR_MAP):
            logging.getLogger(__name__).info(f"Received lidar: {msg}")
            self.__last_lidar = LidarMap(offset = self.__lidar_offset, granularity = self.__lidar_granularity, lidar_data = msg.split(':')[1])
            self.__last_lidar_time = time.time()
        elif msg.startswith(ArduinoConstants.MESSAGE_PREFIX_CAMERAS):
            logging.getLogger(__name__).info(f"Received cameras: {msg}")

            # Cameras:[rotation1]|[tilt1]|[minrotation1]|[maxrotation1]|[mintilt1]|[maxtilt1],[rotation2]|...
            full_cam_config = msg.split(':')[1]
            all_cams = full_cam_config.split(',')
            for cam_id, cam in enumerate(all_cams):
                this_cam = cam.split('|')
                self.__camera_configs[cam_id] = {
                    'rotation':int(this_cam[0]),
                    'tilt':int(this_cam[1]),
                    'min_rotation':int(this_cam[2]),
                    'max_rotation':int(this_cam[3]),
                    'min_tilt':int(this_cam[4]),
                    'max_tilt':int(this_cam[5]),
                }
        elif msg.startswith(ArduinoConstants.MESSAGE_PREFIX_LOG):
            logging.getLogger(__name__).info(f"Arduino:{msg}")


    def get_live_lidar_map (self, max_lidar_age_millis = 6000, timeout = 10):
        if self.__streaming_lidar:
            return self.__last_lidar
        else:
            self.send_message(f"Map:{max_lidar_age_millis}")
            start = time.time()
            if self.wait_for_result(timeout):
                while (time.time() - start <= timeout):
                    if (self.has_message()):
                        msg = self.get_message()
                        if msg.startswith(ArduinoConstants.MESSAGE_PREFIX_LIDAR_MAP):
                            #logging.getLogger(__name__).info(f"Received lidar: {msg}")
                            self.__last_lidar = LidarMap(offset = self.__lidar_offset, granularity = self.__lidar_granularity, lidar_data = msg.split(':')[1])
                            self.__last_lidar_time = time.time()
                            return self.__last_lidar
            
                    time.sleep(0.25)
            return None

    def get_lidar_map (self, max_lidar_age_millis = 6000, wait_for_result = False):
        self.send_message(f"Map:{max_lidar_age_millis}")
        return wait_for_result == False or self.wait_for_result(10)

    def rotate (self, degrees : float, wait_for_result = False):
        logging.getLogger(__name__).error("Observer vehicle does not support movement")
        return False
    
    def stop (self):
        logging.getLogger(__name__).error("Observer vehicle does not support movement")
        return False

    def look (self, rotation : int, tilt : int, wait_for_result = False):
        self.send_message(f"Look:{int(rotation)}|{int(tilt)}")

        # force a refresh of camera configs
        self.__camera_configs = {}

        return wait_for_result == False or self.wait_for_result(10)

    def look_multi (self, angles, wait_for_result = False):
        msg = "Look:"
        for i,(rotation,tilt) in enumerate(angles):
            if i > 0:
                msg = msg + "|"
            msg = msg + f"{int(rotation)}|{int(tilt)}"
        self.send_message(msg)

        # force a refresh of camera configs
        self.__camera_configs = {}

        return wait_for_result == False or self.wait_for_result(10)

    def go (self, speed = 1, duration_millis = 1000):
        logging.getLogger(__name__).error("Observer vehicle does not support movement")
        return False

    def forward_distance (self, speed = 1, distance_units = 10, wait_for_result = False):
        logging.getLogger(__name__).error("Observer vehicle does not support movement")
        return False

    def reverse_distance (self, speed = 1, distance_units = 10, wait_for_result = False):
        logging.getLogger(__name__).error("Observer vehicle does not support movement")
        return False

    def strafe_left (self, wait_for_result = False):
        logging.getLogger(__name__).error("Observer vehicle does not support movement")
        return False

    def strafe_right (self, wait_for_result = False):
        logging.getLogger(__name__).error("Observer vehicle does not support movement")
        return False

    def find_measurement (self, degrees: float, angle_tolerance: float, expected_distance : float, distance_tolerance : float, max_age : int, timeout = 10):
        self.send_message(f"FindMeasurement:{degrees}|{angle_tolerance}|{expected_distance}|{distance_tolerance}|{max_age}")
        return self.__wait_and_return_measurement(timeout)

    def measure (self, degrees: float, tolerance: float, timeout = 10):
        self.send_message(f"Measure:{degrees}|{tolerance}")
        return self.__wait_and_return_measurement(timeout)

    def display_mode (self, mode : str, wait_for_result = False):
        self.send_message(f"ShowMode:{mode}")
        return wait_for_result == False or self.wait_for_result(10)

    def display_status (self, status : str, wait_for_result = False):
        self.send_message(f"ShowStatus:{status}")
        return wait_for_result == False or self.wait_for_result(10)

    def display_command (self, command : str, wait_for_result = False):
        self.send_message(f"ShowCommand:{command}")
        return wait_for_result == False or self.wait_for_result(10)

    def display_position (self, x: float, y:float, heading: float, wait_for_result = False):
        self.send_message(f"ShowPosition:{round(x,1)}|{round(y,1)}|{round(heading,1)}")
        return wait_for_result == False or self.wait_for_result(10)

    def display_objects (self, obj_dist_map, wait_for_result = False):
        if len(obj_dist_map) > 0:
            obj_plus_dist = []
            for obj in obj_dist_map:
                # put asterisk if lidar measurement
                obj_display = f"{obj}*" if obj_dist_map[obj]['islidar'] == True else obj
                obj_plus_dist.append(f"{obj_display}|{round(obj_dist_map[obj]['ground'],1)}")
            obj_string = "|".join(obj_plus_dist)
            self.send_message(f"ShowObjects:{obj_string}")
            return wait_for_result == False or self.wait_for_result(10)
        return False

    def __wait_and_return_measurement (self, timeout):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.has_message():
                msg = self.get_message()
                if msg.startswith(ArduinoConstants.MESSAGE_PREFIX_MEASUREMENT):
                    logging.getLogger(__name__).info(f"Received measurement: {msg}")

                    # Measurement:[angle]|[distance]
                    measurement_vals = msg.split(':')[1].split('|')
                    return float(measurement_vals[0]), float(measurement_vals[1])
            else:
                time.sleep(0.25)

        logging.getLogger(__name__).info(f"No measurement received before timeout")

        return None,None

