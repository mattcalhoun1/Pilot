import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.actions.action_base import ActionBase
from planner.assignment import *

class GoToPositionAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        super().__init__(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger=pilot_logger, pilot_config=pilot_config, pilot=pilot)

        self.__driving_speed = self.get_pilot_config()['Driving']['GoSpeed']
        self.__max_legs = self.get_pilot_config()['Driving']['GoMaxLegs']
        self.__close_enough = self.get_pilot_config()['Driving']['GoCloseEnough']

        # max percentage of the map width to travel in a single leg
        self.__max_dist_per_leg = self.get_pilot_config()['Driving']['MaxDistancePerLeg'] * self.get_pilot_nav().get_field_map().get_width()

        # max number of times to attempt rotate or go forward per leg
        self.__max_leg_attempts = self.get_pilot_config()['Driving']['MaxLegAttempts']

        # whether to stay in the rotation loop until exact heading is achieved, prior to moving forward
        self.__recheck_after_rotation = self.get_pilot_config()['Driving']['RecheckAfterRotation']

    def get_name (self):
        return "Go (x,y)"

    def execute (self, params):
        return self.go_to_position(float(params['X']),float(params['Y']))

    # this method arrives at the desired position by...
    # 1) See if we've arrived. if so, we're done
    # 2) Find an open path (max 2 legs) to the given destination
    # 3) Face the end of the first leg, from step 1. If it fails, go back to step 1
    # 4) Drive toward the destination from step 1, up to max distance defined in settings
    # 5) Go back to step 1

    def go_to_position (self, target_x : float, target_y : float):
        arrived = False
        path_finder = self.get_path_finder()
        curr_leg = 0
        
        if self.get_vehicle().wait_for_ready () and self.get_vehicle().get_all_configurations():
            while not arrived and curr_leg < self.__max_legs:
                curr_leg += 1
                last_x, last_y, last_heading, _,_ = self.get_pilot_nav().get_last_coords_and_heading()

                if path_finder.is_close_enough (last_x, last_y, target_x, target_y, self.__close_enough):
                    logging.getLogger(__name__).info(f"Arrived at {last_x}, {last_y} in {curr_leg} legs")
                    arrived = True
                else:
                    logging.getLogger(__name__).info(f"Compute path to get from {last_x},{last_y} to: {target_x},{target_y}")
                    # grab lidar if available
                    paths = path_finder.find_potential_paths(
                        last_x, last_y, 
                        target_x, target_y, 
                        lidar_map=self.get_vehicle().get_live_lidar_map(10.0),
                        current_heading=last_heading)
                    #target_heading, target_dist = path_finder.find_direct_path(last_x, last_y, target_x, target_y)
                    if len(paths) > 0:
                        selected_path = paths[0]
                        logging.getLogger(__name__).info(f"Selected Path: {selected_path}")

                        # get the first leg
                        leg_heading, leg_dist, leg_x, leg_y = selected_path[0]
                        safe_dist = min(leg_dist, self.__max_dist_per_leg)

                        if safe_dist != leg_dist:
                            logging.getLogger(__name__).info(f"Desired distance of {leg_dist} trimmed down to {safe_dist} for this leg")

                        leg_attempts = 0
                        leg_complete = False
                        while leg_complete == False and leg_attempts < self.__max_leg_attempts:
                            
                            logging.getLogger(__name__).info(f"Head {leg_heading} degrees for {safe_dist} distance")
                            last_x, last_y, last_heading, _,_ = self.get_pilot_nav().get_last_coords_and_heading()

                            # face the given heading
                            face_heading_params = {"heading":leg_heading, "recheck":self.__recheck_after_rotation}
                            face_heading_action = self.get_pilot().get_action_factory().get_secondary_action(command=TaskType.FaceHeading, params=face_heading_params, base_action=self)
                            if face_heading_action.execute(params=face_heading_params):
                                logging.getLogger(__name__).info(f"Face heading {leg_heading} was successful. driving a safe distance of {safe_dist}")
                                # we should now be facing the correct heading. Ok to go forward
                                if self.get_vehicle().forward_distance(speed=self.__driving_speed, distance_units = safe_dist, wait_for_result=True):
                                    logging.getLogger(__name__).info("Vehicle completed leg")
                                    leg_complete = True
                                else:
                                    # rotation was successful, but the drive was not. maybe hit an obstacle or something
                                    # need to recalculate path
                                    logging.getLogger(__name__).info("Vehicle attempted drive, but leg did not complete")
                                    leg_attempts = self.__max_leg_attempts
                            else:
                                # recompute path from whereever we're facing now
                                leg_attempts += 1
                                logging.getLogger(__name__).info("Face heading failed. Recalculating path.")

                            # either way, we need updated coordinates/heading to figure out what's next
                            self.find_new_position()

                    else:
                        self.get_vehicle().display_status("No Path", True)
                        logging.getLogger(__name__).info("No path to that location found, maybe blocked or out of bounds!!")
        else:
            logging.getLogger(__name__).info("Vehicle not ready!")
        
        self.get_pilot_nav().invalidate_position()

        return arrived

