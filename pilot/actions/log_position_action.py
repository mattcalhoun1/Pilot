import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.actions.action_base import ActionBase

class LogPositionAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        self.__vehicle = vehicle
        self.__pilot = pilot
        self.__pilot_nav = pilot_nav
        self.__pilot_logger = pilot_logger
        self.__pilot_config = pilot_config

    def get_name (self):
        return "Log (x,y)"

    def execute (self, params):
        return self.log_position()

    def log_position(self):
        logging.getLogger(__name__).info("Log position action triggered")
        x,y,heading,confidence = self.__pilot_nav.get_coords_and_heading()
        logging.getLogger(__name__).info(f"Log position result: {x},{y} - {heading}")
        return x is not None and heading is not None
