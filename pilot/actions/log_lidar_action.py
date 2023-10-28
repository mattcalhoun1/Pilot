import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.actions.action_base import ActionBase

class LogLidarAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        self.__vehicle = vehicle
        self.__pilot = pilot
        self.__pilot_nav = pilot_nav
        self.__pilot_logger = pilot_logger
        self.__pilot_config = pilot_config

    def get_name (self):
        return "LogLidar"

    def execute (self, params):
        return self.log_lidar()

    def log_lidar (self):
        # get live lidar from the vehicvle
        if self.__vehicle.wait_for_ready() and self.__vehicle.get_all_configurations():
            lidar_map = self.__vehicle.get_live_lidar_map(timeout=10.0)
            if lidar_map is not None:
                return self.__pilot_logger.log_lidar(lidar_map)
            else:
                logging.getLogger(__name__).error("Lidar map not retrieved from vehicle")
        return False
