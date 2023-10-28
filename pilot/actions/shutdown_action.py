import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.actions.action_base import ActionBase
import os
import logging

class ShutdownAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        self.__vehicle = vehicle
        self.__pilot = pilot
        self.__pilot_nav = pilot_nav
        self.__pilot_logger = pilot_logger
        self.__pilot_config = pilot_config

    def get_name (self):
        return "Shutdown"

    def execute (self, params):
        logging.getLogger(__name__).info("Shutting down")
        os.system('/usr/bin/sudo /usr/sbin/poweroff')

    def mark_complete_immediately (self):
        return True