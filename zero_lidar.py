from arduino.observer import Observer
from arduino.arduino_constants import ArduinoConstants
from lidar.lidar_map import LidarMap
import time
import logging

if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
    vehicle = Observer()

    if vehicle.wait_for_ready() and vehicle.get_all_configurations():
        print(f"Connected.")

        while(True):
            lidar_map = vehicle.get_live_lidar_map()

            print('\n'.join([
                f"  0 | {lidar_map.get_closest_available_angle(0.0):>5} : {lidar_map.get_measurement(lidar_map.get_closest_available_angle(0.0),0.1):<6}",
                f" 90 | {lidar_map.get_closest_available_angle(90.0):>5} : {lidar_map.get_measurement(lidar_map.get_closest_available_angle(90.0),0.1):<6}",
                f"180 | {lidar_map.get_closest_available_angle(180.0):>5} : {lidar_map.get_measurement(lidar_map.get_closest_available_angle(180.0),0.1):<6}",
                f"360 | {lidar_map.get_closest_available_angle(270.0):>5} : {lidar_map.get_measurement(lidar_map.get_closest_available_angle(270.0),0.1):<6}"
                    ]))

            time.sleep(.5)
    else:
        print(f"Vehicle connection failed")
