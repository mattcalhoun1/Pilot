from arduino.meccar import MecCar
from arduino.arduino_constants import ArduinoConstants
from lidar.lidar_map import LidarMap
import time
import logging

if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
    tank = MecCar()
    print(f"Waiting for tank usb connection...")
    last_message = 0
    last_map_refresh = 0
    map_refresh_frequency = 10 # 10 sec
    stale_connection_threshold = 15 # 15 sec

    log_only = False

    lidar_offset = None
    lidar_granularity = None

    if tank.wait_for_ready():
        print(f"Connected.")

        while(True):
            if not log_only:
                if lidar_offset is None:
                    tank.get_config("LidarHeading")
                elif lidar_granularity is None:
                    tank.get_config("LidarGranularity")
                elif time.time() - last_map_refresh > map_refresh_frequency:
                    if (tank.get_lidar_map()):
                        last_map_refresh = time.time()
                    time.sleep(.1) # give time for the map to come across

            while tank.has_message():
                msg = tank.get_message()
                if msg.startswith(ArduinoConstants.MESSAGE_PREFIX_LOG):
                    print(f"Log: {msg.split(':')[1]}")
                elif msg.startswith(ArduinoConstants.MESSAGE_PREFIX_RESULT):
                    print(f"Result: {msg.split(':')[1]}")
                elif msg.startswith(ArduinoConstants.MESSAGE_PREFIX_MEASUREMENT):
                    print(f"Measurement: {msg.split(':')[1]}")
                elif msg.startswith(ArduinoConstants.MESSAGE_PREFIX_LIDAR_MAP):
                    #print(f"Received Map: {msg.split(':')[1]}")
                    # print the distance at angles 0, 90, 180, -90
                    lidarMap = LidarMap(offset = lidar_offset, granularity = lidar_granularity, lidar_data = msg.split(':')[1])

                    print('\n'.join([
                        f"{lidarMap.get_closest_available_angle(0.0):>5} : {lidarMap.get_measurement(lidarMap.get_closest_available_angle(0.0),0.1):<6}",
                        f"{lidarMap.get_closest_available_angle(90.1):>5} : {lidarMap.get_measurement(lidarMap.get_closest_available_angle(90.1),0.1):<6}",
                        f"{lidarMap.get_closest_available_angle(180.20):>5} : {lidarMap.get_measurement(lidarMap.get_closest_available_angle(180.2),0.1):<6}",
                        f"{lidarMap.get_closest_available_angle(270.3):>5} : {lidarMap.get_measurement(lidarMap.get_closest_available_angle(270.3),0.1):<6}"
                    ]))

                elif msg.startswith(ArduinoConstants.MESSAGE_PREFIX_CONFIG):
                    print(f"Received config: {msg}")
                    config_entry = msg.split(':')[1]
                    config_key = config_entry.split('|')[0]
                    config_val = config_entry.split('|')[1]
                    if config_key == 'LidarHeading':
                        lidar_offset = int(config_val)
                    elif config_key == 'LidarGranularity':
                        lidar_granularity = float(config_val)

                elif msg != ArduinoConstants.MESSAGE_PREFIX_READY:
                    print(f"{msg}")
                last_message = time.time()

            time.sleep(.5)

            if time.time() - last_message > stale_connection_threshold:
                try:
                    tank.refresh_connection()
                except Exception as e:
                    print(f"Unable to refresh connection: {e}")
    else:
        print(f"Tank connection failed")


