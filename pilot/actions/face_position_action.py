import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.actions.action_base import ActionBase
from pilot.actions.action_factory import ActionFactory
from planner.assignment import *

class FacePositionAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        super().__init__(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger=pilot_logger, pilot_config=pilot_config, pilot=pilot)

    def get_name (self):
        return "Face (x,y)"

    def execute (self, params):
        return self.face_position(float(params['X']),float(params['Y']))

    def face_position (self, target_x : float, target_y : float):
        arrived = False
        path_finder = self.get_path_finder()
        
        if self.__vehicle.wait_for_ready ():
            last_x, last_y, last_heading, _,_ = self.__pilot_nav.get_last_coords_and_heading()

            target_heading, target_dist, x, y = path_finder.find_direct_path(last_x, last_y, target_x, target_y)

            # use the face heading action to complete
            face_heading_params = {'heading':target_heading}
            face_heading_action = ActionFactory.get_secondary_action(command=TaskType.FaceHeading, params=face_heading_params, base_action=self)
            arrived = face_heading_action.execute(params=face_heading_params)

            # turn the tank to to the correct heading
            if not arrived:
                logging.getLogger(__name__).error("Face position failed")
        else:
            logging.getLogger(__name__).info("Vehicle not ready!")

        self.__pilot_nav.invalidate_position()

        return arrived
