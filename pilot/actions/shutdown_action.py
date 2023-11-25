import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.actions.action_base import ActionBase
import os
import logging

class ShutdownAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        super().__init__(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger=pilot_logger, pilot_config=pilot_config, pilot=pilot)

    def get_name (self):
        return "Shutdown"

    def execute (self, params):
        logging.getLogger(__name__).info("Shutting down")
        os.system('/usr/bin/sudo /usr/sbin/poweroff')

    def mark_complete_immediately (self):
        return True