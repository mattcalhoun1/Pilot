import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.actions.action_base import ActionBase
import time

class SleepAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        self.__vehicle = vehicle
        self.__pilot = pilot
        self.__pilot_nav = pilot_nav
        self.__pilot_logger = pilot_logger
        self.__pilot_config = pilot_config

    def get_name (self):
        return "Sleep"

    def execute (self, params):
        try:
            sleep_seconds = float(params['seconds'])
            logging.getLogger(__name__).info(f"Sleeping for {sleep_seconds} seconds")

            start_time = time.time()
            while (time.time() - start_time < sleep_seconds):
                self.__vehicle.display_status(f"Sleep {round(time.time() - sleep_seconds)}", True)
                time.sleep(0.1)
        except:
            logging.getLogger(__name__).error("Sleep failed")
            return False
        
        return True
