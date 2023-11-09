from pilot.pilot import Pilot
from arduino.tank import Tank
import logging
import time

if __name__ == '__main__':
    logging.basicConfig(format='%(asctime)s [%(levelname)s] %(module)s:%(message)s', level=logging.INFO)
    
    p = Pilot('/home/matt/projects/NavConfig/pilot_settings.json', vehicle=Tank())
    #p.download_resources(use_cached_maps = True, use_cached_models = True)

    while p.is_alive():
        # check connection
        if p.connect_to_vehicle():
            logging.getLogger(__name__).info("Connected to vehicle")
            if p.has_assignments():
                p.complete_assignments()
            else:
                time.sleep(1)
        else:
            logging.getLogger(__name__).warn("Not connected to vehicle")
            time.sleep(2)
