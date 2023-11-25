import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.actions.action_base import ActionBase

class DoNothingAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        super().__init__(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger=pilot_logger, pilot_config=pilot_config, pilot=pilot)

    def get_name (self):
        return "Ready"

    def execute (self, params):
        logging.getLogger(__name__).info("Do nothing action, does nothing")
        return True

