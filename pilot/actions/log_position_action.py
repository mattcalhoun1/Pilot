import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.actions.action_base import ActionBase

class LogPositionAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        super().__init__(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger=pilot_logger, pilot_config=pilot_config, pilot=pilot)

    def get_name (self):
        return "Log (x,y)"

    def execute (self, params):
        return self.log_position()

    def log_position(self):
        logging.getLogger(__name__).info("Log position action triggered")
        x,y,heading,confidence = self.get_pilot_nav().get_coords_and_heading()
        logging.getLogger(__name__).info(f"Log position result: {x},{y} - {heading}")
        return x is not None and heading is not None
