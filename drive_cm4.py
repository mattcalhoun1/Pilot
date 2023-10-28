from pilot.pilot import Pilot
from arduino.meccar import MecCar
import logging
import time

if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)    
    
    car = MecCar()
    p = Pilot('/home/matt/projects/NavConfig/pi_cm4/pilot_settings.json', vehicle=car)
    #p.download_resources(use_cached_maps = True, use_cached_models = True)

    last_position_log = 0
    default_map = 'basement'
    position_frequency = 20.0

    while p.is_alive():
        # check connection
        if p.connect_to_vehicle():
            #p.search(objects=['gazing_ball',], pilot_nav = p.get_navigator(default_map))
            #car.look_multi([(33.0, 90),], wait_for_result=True)
            
            #logging.getLogger(__name__).info("Connected to vehicle")
            #p.enter_controlled_mode(pilot_nav=p.get_navigator('basement'))

            if True:
                if p.has_assignments():
                    p.complete_assignments()
                #elif time.time() - last_position_log > position_frequency:
                #    p.log_position(pilot_nav = p.get_navigator(default_map))
                #    last_position_log = time.time()
                #else:
                #    p.search(objects=['gazing_ball'], pilot_nav = p.get_navigator(default_map))
                #    time.sleep(1)
        else:
            logging.getLogger(__name__).warning("Not connected to vehicle")
            time.sleep(2)
