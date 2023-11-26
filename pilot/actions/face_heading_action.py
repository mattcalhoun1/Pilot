import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.actions.action_base import ActionBase

class FaceHeadingAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        super().__init__(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger=pilot_logger, pilot_config=pilot_config, pilot=pilot)

        # if we are this many degrees +/- the desired heading, it's ok to go ahead
        self.__rotation_degree_allowance = self.get_pilot_config()['Driving']['RotationDegreeAllowance']

        # max number of times to attempt rotate or go forward
        self.__max_attempts = self.get_pilot_config()['Driving']['RotationMaxAttempts']

    def get_name (self):
        return "Face °"

    def execute (self, params):
        recheck = True if 'recheck' not in params else params['recheck']
        return self.face_heading(float(params['heading']), recheck)

    def face_heading (self, target_heading : float, recheck : bool):
        arrived = False
        path_finder = self.get_path_finder()
        if self.get_vehicle().wait_for_ready ():
            rotation_attempts = 0
            while arrived == False and rotation_attempts < self.__max_attempts:
                if rotation_attempts != 0:
                    # update the heading, since we've moved since last check
                    self.find_new_position()

                current_x, current_y, current_heading, _,_ = self.get_pilot_nav().get_last_coords_and_heading()

                # turn the tank to to the correct heading, repeat until we are facing close to hte correct 
                # direction
                target_rotation = path_finder.find_rotation(current_heading=current_heading, target_heading=target_heading)
                if abs(target_rotation) <= self.__rotation_degree_allowance:
                    logging.getLogger(__name__).info(f"Wanted heading requires rotation of {target_rotation}, which is within our heading allowance.")
                    arrived = True
                else:
                    logging.getLogger(__name__).info(f"Rotation of {target_rotation} is required.")
                    if self.get_vehicle().rotate(target_rotation, wait_for_result=True):
                        # if not rechecking, assume we've arrived
                        if not recheck:
                            arrived = True
                            logging.getLogger(__name__).info("Rotation succeeded. Assuming heading is now correct, returning success")
                        else:
                            logging.getLogger(__name__).info("Rotation complete. Checking updated heading")
                            rotation_attempts += 1
                    else:
                        logging.getLogger(__name__).error("rotation failed, getting updated heading")
                        rotation_attempts += 1
                
        self.get_pilot_nav().invalidate_position()

        return arrived
