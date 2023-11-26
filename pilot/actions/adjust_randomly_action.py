import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.actions.action_base import ActionBase
from planner.assignment import *
from arduino.arduino_constants import ArduinoConstants
import numpy as np

class AdjustRandomlyAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        super().__init__(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger=pilot_logger, pilot_config=pilot_config, pilot=pilot)

    def get_name (self):
        return "Adjust Randomly"

    def execute (self, params):
        strafe_dir = self.__get_random_direction()
        strafe_millis = 500
        rotate_deg = self.__get_random_rotation()

        # attempt to execute both moves on the car. If it fails, it's not a big deal
        logging.getLogger(__name__).info(f"Adjusting by strafing {strafe_dir} for {strafe_millis} millis and rotating {rotate_deg}")
        try:
            self.get_vehicle().strafe (strafe_direction = strafe_dir, millis = strafe_millis, wait_for_result = True)

            self.get_vehicle().rotate(degrees = rotate_deg, wait_for_result = True)
            logging.getLogger(__name__).info(f"Adjusting complete")

        except Exception as e:
            logging.getLogger(__name__).warning(f"Random adjust failed: {e}")

        return True

    def __get_random_rotation (self):
        return np.random.choice([
            #-30.0,
            -20.0,
            -15.0,
            -10.0,
            -5.0,
            -2.5,
            2.5,
            5.0,
            10.0,
            15.0,
            20.0,
            #30.0
        ])

    def __get_random_direction (self):
        return np.random.choice([
            #ArduinoConstants.STRAFE_LEFT_FORWARD,
            ArduinoConstants.STRAFE_LEFT_LEFT,
            #ArduinoConstants.STRAFE_LEFT_BACKWARD,
            #ArduinoConstants.STRAFE_RIGHT_FORWARD,
            ArduinoConstants.STRAFE_RIGHT_RIGHT,
            #ArduinoConstants.STRAFE_RIGHT_BACKWARD
        ])

    def strafe_left (self):
        self.get_vehicle().strafe

    def face_position (self, target_x : float, target_y : float):
        arrived = False
        path_finder = self.get_path_finder()
        
        if self.get_vehicle().wait_for_ready ():
            last_x, last_y, last_heading, _,_ = self.get_pilot_nav().get_last_coords_and_heading()

            target_heading, target_dist = path_finder.find_direct_path(last_x, last_y, target_x, target_y)

            # use the face heading action to complete
            face_heading_params = {'heading':target_heading}
            face_heading_action = self.get_pilot().get_action_factory().get_secondary_action(command=TaskType.FaceHeading, params=face_heading_params, base_action=self)
            arrived = face_heading_action.execute(params=face_heading_params)

            # turn the tank to to the correct heading
            if not arrived:
                logging.getLogger(__name__).error("Face position failed")
        else:
            logging.getLogger(__name__).info("Vehicle not ready!")

        self.get_pilot_nav().invalidate_position()

        return arrived