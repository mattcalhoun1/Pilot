from pilot.pilot import Pilot
from arduino.observer import Observer
import logging
import time
import sys

if __name__ == '__main__':
    logging.basicConfig(format='%(asctime)s [%(levelname)s] %(module)s:%(message)s', level=logging.INFO)

    session_id = None
    if len(sys.argv) > 1:
        session_id = sys.argv[1]
        logging.getLogger(__name__).info(f"Beginning session {session_id}")
    

    vehicle = Observer()
    p = Pilot('/home/matt/projects/NavConfig/jetson_nano/pilot_settings.json', vehicle=vehicle, session_id=session_id)
    
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
