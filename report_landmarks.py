from pilot.pilot_resources import PilotResources
from pilot.test.mock_pilot_logger import MockPilotLogger
from pilot.pilot_navigation import PilotNavigation
from arduino.test.test_car import TestCar
import time
import logging
import traceback
import platform

# this script simply attempts to use the cameras without any elaborate searching or movement
# and logs (to stdout) the landmarks it sees

if __name__ == '__main__':
    logging.basicConfig(format='%(asctime)s [%(levelname)s] %(module)s:%(message)s', level=logging.INFO)
    
    # determine if we are on jetson nano or other, choose config file accordingly
    pilot_settings_file = '/home/matt/projects/NavConfig/pi_cm4/pilot_settings.json'
    if platform.processor() == 'aarch64':
	    pilot_settings_file = '/home/matt/projects/NavConfig/jetson_nano/pilot_settings.json'

    pilot_resources = PilotResources(pilot_settings_file)

    # set the default map
    map_id = 'basement_v2'
    pilot_resources.download_map (map_id=map_id, use_cached_maps = True, use_cached_models = True)
    pilot_nav = PilotNavigation(
        pilot_resources = pilot_resources, 
        pilot_logger = MockPilotLogger(),
        map_id = map_id,
        field_map = pilot_resources.get_map(map_id), 
        starting_altitude = pilot_resources.get_config()['Altitude'],
        vehicle = TestCar(left_cam_starting_rotation=0, right_cam_starting_rotation=0))

    go = True

    try:
        while (go):
            landmarks = pilot_nav.locate_landmarks()
            logging.getLogger(__name__).info(f"Reported Landmarks: {landmarks}")
            
            time.sleep(1)
    except KeyboardInterrupt:
        logging.getLogger(__name__).info("Keyboard interrupt")
    except Exception as e:
        logging.getLogger(__name__).warning(f"Exception thrown: {e}")
        traceback.print_exc()


    
