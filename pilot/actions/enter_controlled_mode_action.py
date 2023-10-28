import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.actions.action_base import ActionBase
from pilot.controlled_drive import ControlledDrive

class EnterControlledModeAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        self.__vehicle = vehicle
        self.__pilot_nav = pilot_nav
        self.__pilot_logger = pilot_logger
        self.__pilot_config = pilot_config
        self.__pilot = pilot

    def get_name (self):
        return "Controlled"

    def execute (self, params):
        self.__vehicle.display_mode(mode='PS4', wait_for_result=True)
        self.enter_controlled_mode()
        self.__vehicle.display_mode(mode='Autonomous', wait_for_result=True)

    def enter_controlled_mode (self):
        logging.getLogger(__name__).info("Now in controlled mode.")
        # vehicle stays stuck in this mode until given an 'autonomous' assignment
        
        # to do, listen for ps4 commands and check at intervals for autonomous or shutdown command
        controlled = ControlledDrive(self.__vehicle, pilot=self.__pilot, pilot_nav=self.__pilot_nav)
        controlled.start_driving()
        
        # returning form this method re-enters autonomous mode
        logging.getLogger(__name__).info("Exiting controlled mode.")

        self.__pilot_nav.invalidate_position()
