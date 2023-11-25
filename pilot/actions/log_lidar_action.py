import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.actions.action_base import ActionBase

class LogLidarAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        super().__init__(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger=pilot_logger, pilot_config=pilot_config, pilot=pilot)

    def get_name (self):
        return "LogLidar"

    def execute (self, params):
        return self.log_lidar()

    def log_lidar (self):
        # get live lidar from the vehicvle
        if self.get_vehicle().wait_for_ready() and self.get_vehicle().get_all_configurations():
            # invalidate any cache
            self.get_pilot_nav().invalidate_position()
            
            lidar_map = self.get_vehicle().get_live_lidar_map(timeout=10.0)
            if lidar_map is not None:
                return self.get_pilot_logger().log_lidar(lidar_map)
            else:
                logging.getLogger(__name__).error("Lidar map not retrieved from vehicle")
        return False
