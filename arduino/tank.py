from arduino.arduino import Arduino
from arduino.arduino_constants import ArduinoConstants
from lidar.lidar_map import LidarMap
import logging
import time

class Tank(Arduino):
    def __init__(self):
        self.__lidar_offset = None
        self.__lidar_granularity = None

        connected = False
        for trial_port in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']:
            try:
                logging.getLogger(__name__).info(f"Trying connection on {trial_port}")
                Arduino.__init__(self, usb_port = trial_port, baud = 9600, timeout = 1)
                logging.getLogger(__name__).info(f"Connected to Arduino on {trial_port}")
                connected = True
            except Exception as e:
                logging.getLogger(__name__).info(f"Failed to connect on {trial_port}: {e}")
            if connected:
                break
        if not connected:
            logging.getLogger(__name__).error("Failed to connect to vehicle!")
            raise Exception("Arduino connection error")

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
                    else:
                        logging.getLogger(__name__).info(f"Received unexpected message: {msg}")
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

    def get_cameras (self):
        self.send_message(f"GetCameras:none")
        return self.wait_for_result(10)

    def get_live_lidar_map (self, timeout = 10):
        self.send_message(f"Map:none")
        start = time.time()
        if self.wait_for_result(timeout):
            while (time.time() - start <= timeout):
                if (self.has_message()):
                    msg = self.get_message()
                    if msg.startswith(ArduinoConstants.MESSAGE_PREFIX_LIDAR_MAP):
                        self.__last_lidar = LidarMap(offset = self.__lidar_offset, granularity = self.__lidar_granularity, lidar_data = msg.split(':')[1])
                        self.__last_lidar_time = time.time()
                        return self.__last_lidar
                time.sleep(0.25)

        return None

    def get_lidar_map (self):
        self.send_message(f"Map:none")
        return self.wait_for_result(10)

    def rotate (self, degrees : float):
        self.send_message(f"Rotate:{degrees}")
        return self.wait_for_result(10)
    
    def stop (self):
        self.send_message(f"Stop:0")
        return self.wait_for_result(10)

    def look (self, rotation : int, tilt : int):
        self.send_message(f"Look:{rotation}|{tilt}")
        return self.wait_for_result(10)

    def go (self, speed = 1, duration_millis = 1000):
        self.send_message(f"Go:{speed}|{duration_millis}")
        return self.wait_for_result(10)

    def forward_distance (self, speed = 1, distance_units = 10):
        self.send_message(f"Forward:{distance_units}|{speed}")
        return self.wait_for_result(10)

    def reverse_distance (self, speed = 1, distance_units = 10):
        self.send_message(f"Reverse:{distance_units}|{speed}")
        return self.wait_for_result(10)
    
    def measure (self, degrees: float, tolerance: float):
        self.send_message(f"Measure:{degrees}|{tolerance}")
        return self.wait_for_result(10)

