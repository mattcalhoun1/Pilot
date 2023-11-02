from pilot.pilot import Pilot
from arduino.observer import Observer
import logging
import time

if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)    
    
    vehicle = Observer()
    p = Pilot('/home/matt/projects/NavConfig/jetson_nano/pilot_settings.json', vehicle=vehicle)

    last_position_log = 0
    default_map = 'basement'
    position_frequency = 20.0

    while p.is_alive():
        # check connection
        if p.connect_to_vehicle():
            if p.has_assignments():
                p.complete_assignments()
        else:
            logging.getLogger(__name__).warning("Not connected to vehicle")
            time.sleep(2)
