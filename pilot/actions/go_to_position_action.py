import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.actions.action_base import ActionBase

class GoToPositionAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        self.__vehicle = vehicle
        self.__pilot = pilot
        self.__pilot_nav = pilot_nav
        self.__pilot_logger = pilot_logger
        self.__pilot_config = pilot_config

    def get_name (self):
        return "Go (x,y)"

    def execute (self, params):
        return self.go_to_position(float(params['X']),float(params['Y']))

    def go_to_position (self, target_x : float, target_y : float):
        arrived = False
        max_dist = 2 # lets be within 2 of the target_dist
        path_finder = PathFinder(field_map=self.__pilot_nav.get_field_map())
        step_size_seconds = 2.0
        speed = 1
        max_steps = 10
        curr_step = 0
        
        if self.__vehicle.wait_for_ready () and self.__vehicle.get_all_configurations():
            while not arrived and curr_step < max_steps:
                curr_step += 1
                last_x, last_y, last_heading, _,_ = self.__pilot_nav.get_last_coords_and_heading()

                # grab lidar if available
                lidar = self.__vehicle.get_live_lidar_map(10.0)

                if path_finder.is_close_enough (last_x, last_y, target_x, target_y, max_dist):
                    logging.getLogger(__name__).info(f"Arrived at {last_x}, {last_y} in {curr_step} steps")
                    arrived = True
                else:
                    logging.getLogger(__name__).info(f"Compute path to get from {last_x},{last_y} to: {target_x},{target_y}")
                    paths = path_finder.find_potential_paths(last_x, last_y, target_x, target_y, lidar_map=lidar, current_heading=last_heading)
                    #target_heading, target_dist = path_finder.find_direct_path(last_x, last_y, target_x, target_y)
                    if len(paths) > 0:
                        logging.getLogger(__name__).info(f"Selected Path: {paths[0]}")
                        last_leg_heading = last_heading

                        for target_heading,target_dist in paths[0]:
                            logging.getLogger(__name__).info(f"Head {target_heading} degrees for {target_dist} distance")

                            # turn the tank to to the correct heading
                            target_rotation = path_finder.find_rotation(last_leg_heading, target_heading)
                            if self.__vehicle.rotate(target_rotation, wait_for_result=True):
                                if not self.__vehicle.forward_distance(speed=10.0, distance_units = target_dist, wait_for_result=True):
                                    logging.getLogger(__name__).error("Vehicle failed to complete leg")
                            else:
                                logging.getLogger(__name__).error("rotation failed")
                            
                        # update our position for the next distance check
                        self.__pilot_nav.get_coords_and_heading()
                    else:
                        logging.getLogger(__name__).info("No path to that location found, maybe blocked or out of bounds!!")
        else:
            logging.getLogger(__name__).info("Vehicle not ready!")
        
        return arrived

