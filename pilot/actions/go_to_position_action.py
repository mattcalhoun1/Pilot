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

        self.__driving_speed = self.__pilot_config['Driving']['GoSpeed']
        self.__max_steps = self.__pilot_config['Driving']['GoMaxSteps']
        self.__max_target_distance = self.__pilot_config['Driving']['GoTargetPositionDistance']

        self.__wanted_angle_allowance = 3.0 # if we are this many degrees +/- the desired heading, it's ok to go ahead
        self.__max_leg_attempts = 7 # max number of times to attempt rotate or go forward

    def get_name (self):
        return "Go (x,y)"

    def execute (self, params):
        return self.go_to_position(float(params['X']),float(params['Y']))

    def go_to_position (self, target_x : float, target_y : float):
        arrived = False
        path_finder = PathFinder(field_map=self.__pilot_nav.get_field_map())
        curr_step = 0
        
        if self.__vehicle.wait_for_ready () and self.__vehicle.get_all_configurations():
            while not arrived and curr_step < self.__max_steps:
                curr_step += 1
                last_x, last_y, last_heading, _,_ = self.__pilot_nav.get_last_coords_and_heading()

                # grab lidar if available
                lidar = self.__vehicle.get_live_lidar_map(10.0)

                if path_finder.is_close_enough (last_x, last_y, target_x, target_y, self.__max_target_distance):
                    logging.getLogger(__name__).info(f"Arrived at {last_x}, {last_y} in {curr_step} steps")
                    arrived = True
                else:
                    logging.getLogger(__name__).info(f"Compute path to get from {last_x},{last_y} to: {target_x},{target_y}")
                    paths = path_finder.find_potential_paths(last_x, last_y, target_x, target_y, lidar_map=lidar, current_heading=last_heading)
                    #target_heading, target_dist = path_finder.find_direct_path(last_x, last_y, target_x, target_y)
                    if len(paths) > 0:
                        selected_path = paths[0]
                        logging.getLogger(__name__).info(f"Selected Path: {selected_path}")

                        leg_id = 0
                        leg_attempts = 0
                        while leg_id < len(selected_path) and leg_attempts < self.__max_leg_attempts:
                            target_heading,target_dist = selected_path[leg_id]
                            logging.getLogger(__name__).info(f"Head {target_heading} degrees for {target_dist} distance")

                            if leg_id != 0 or leg_attempts != 0:
                                # update the heading, since we've moved since last check
                                self.__pilot_nav.get_coords_and_heading()
                                last_x, last_y, last_heading, _,_ = self.__pilot_nav.get_last_coords_and_heading()
                            last_leg_heading = last_heading

                            # turn the tank to to the correct heading, repeat until we are facing close to hte correct 
                            # direction
                            target_rotation = path_finder.find_rotation(last_leg_heading, target_heading)
                            if abs(target_rotation) <= self.__wanted_angle_allowance:
                                logging.getLogger(__name__).info(f"Wanted heading requires rotation of {target_rotation}, which is within our heading allowance, so will go forward.")
                                if not self.__vehicle.forward_distance(speed=self.__driving_speed, distance_units = target_dist, wait_for_result=True):
                                    logging.getLogger(__name__).error("Vehicle failed to complete leg")
                                else:
                                    # move onto the next leg
                                    leg_id += 1
                                    leg_attempts = 0
                            else:
                                logging.getLogger(__name__).info(f"Rotation of {target_rotation} is required.")
                                if self.__vehicle.rotate(target_rotation, wait_for_result=True):
                                    logging.getLogger(__name__).info("Rotation complete. Checking updated heading")
                                    leg_attempts += 1
                                else:
                                    logging.getLogger(__name__).error("rotation failed")
                                    leg_attempts += 1
                            
                        # update our position for the next distance check
                        self.__pilot_nav.get_coords_and_heading()
                    else:
                        self.__vehicle.display_status("No Path", True)
                        logging.getLogger(__name__).info("No path to that location found, maybe blocked or out of bounds!!")
        else:
            logging.getLogger(__name__).info("Vehicle not ready!")
        
        return arrived

