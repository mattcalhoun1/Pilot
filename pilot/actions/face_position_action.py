import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.actions.action_base import ActionBase

class FacePositionAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        self.__vehicle = vehicle
        self.__pilot = pilot
        self.__pilot_nav = pilot_nav
        self.__pilot_logger = pilot_logger
        self.__pilot_config = pilot_config

    def get_name (self):
        return "Face (x,y)"

    def execute (self, params):
        return self.face_position(float(params['X']),float(params['Y']))

    def face_position (self, target_x : float, target_y : float):
        arrived = False
        path_finder = PathFinder()
        
        if self.__vehicle.wait_for_ready ():
            last_x, last_y, last_heading, _,_ = self.__pilot_nav.get_last_coords_and_heading()

            target_heading, target_dist = path_finder.find_direct_path(last_x, last_y, target_x, target_y)
            logging.getLogger(__name__).info(f"Rotate {target_heading} degrees")

            # turn the tank to to the correct heading
            target_rotation = path_finder.find_rotation(last_heading, target_heading)
            if not self.__vehicle.rotate(target_rotation):
                logging.getLogger(__name__).error("rotation failed")
            else:
                arrived = True
                
            # update our position for the next distance check
            self.__pilot_nav.get_coords_and_heading()
        else:
            logging.getLogger(__name__).info("Vehicle not ready!")

        self.__pilot_nav.invalidate_position()

        return arrived
