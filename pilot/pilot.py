import logging
import json
import traceback
from pilot.pilot_resources import PilotResources
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.actions.action_factory import ActionFactory
from planner.assignment import *

class Pilot:
    def __init__(self, config_file, vehicle, use_cached_maps = True, use_cached_models = True, session_id = None):
        self.__resources = PilotResources(config_file)
        self.__pilot_logger = PilotLogger(config_file, session_id)
        self.__resources.download_resources(use_cached_maps=use_cached_maps, use_cached_models=use_cached_models)
        self.__running = True
        self.__vehicle = vehicle
        self.__load_config(config_file=config_file)
        self.__action_factory = ActionFactory()

        # these are needed for the search function. maybe refactor out of here
        self.__default_camera_positions = [ # these are vehicle-centric, meaning 0 is front. navigation considers 90 the front, so its different
            self.__config['Cameras']['Left']['DefaultHeading'] - 90,
            self.__config['Cameras']['Right']['DefaultHeading'] - 90
        ] 

    def __load_config (self, config_file):
        self.__config_file = config_file
        with open(self.__config_file, 'r') as cfg:
            self.__config = json.loads(cfg.read())

    def __requires_location (self, assignment):
        for s in assignment['assignment']['steps']:
            if s['command'] in [TaskType.Go, TaskType.Face]:
                return True
        return False

    def __execute_step (self, assignment, command, params, pilot_nav):
        action_handler = self.get_action_factory().get_action_for(
            command = command,
            params = params,
            vehicle = self.__vehicle,
            pilot_nav = pilot_nav,
            pilot_logger = self.__pilot_logger,
            pilot_config = self.__config,
            pilot = self)

        self.__vehicle.display_mode(action_handler.get_name())
        
        # if this task/step tells us to mark the assignment complete, 
        # do that before going any further
        if action_handler.mark_complete_immediately():
            self.__resources.complete_assignment(assignment)

        return action_handler.execute(params)

    def get_action_factory (self):
        return self.__action_factory

    def is_alive (self):
        # gets switched to false with control+c
        return self.__running
    
    def connect_to_vehicle (self):
        connected_to_vehicle = self.__vehicle.wait_for_ready () and self.__vehicle.get_all_configurations()

        if connected_to_vehicle:
            self.return_cameras_to_nav_position()

        return connected_to_vehicle
    
    def has_assignments (self):
        self.__vehicle.display_mode(mode='Autonomous', wait_for_result=True)
        return len(self.__resources.get_assignments(refresh=True)) > 0

    # if cameras have moved since this last call, it is reset to default    
    def return_cameras_to_nav_position(self, pilot_nav = None):
        self.__vehicle.look_multi([(self.__default_camera_positions[0], 90),(self.__default_camera_positions[1], 90)], wait_for_result=True)
        if pilot_nav is not None: # might not be set if initializing
            pilot_nav.read_and_update_camera_positions(self.__vehicle)
    

    def get_navigator(self, map_id):
        # ensure the map is downloaded, cache is ok
        self.__resources.download_map(map_id)

        return PilotNavigation(
            pilot_resources = self.__resources, 
            pilot_logger = self.__pilot_logger,
            map_id = map_id,
            field_map = self.__resources.get_map(map_id), 
            starting_altitude = self.__resources.get_config()['Altitude'],
            vehicle = self.__vehicle)
                
    def complete_assignments (self):
        # Initialize navigation for each assigment
        for assignment in self.__resources.get_assignments():
            pilot_nav = None
            try:
                # initialize navigation for the required map 
                map_id = assignment['assignment']['map_id']
                pilot_nav = self.get_navigator(map_id)

                starting_x = None
                starting_y = None
                starting_heading = None
                starting_confidence = None
                location_required = self.__requires_location(assignment)
                
                if location_required:
                    # determine starting position
                    starting_x, starting_y, starting_heading, starting_confidence = pilot_nav.get_coords_and_heading()
                    logging.getLogger(__name__).info(f"Map {map_id} - Starting Location ({starting_x},{starting_y}) Heading {starting_heading}, Confidence: {starting_confidence}")

                success = True
                if location_required == False or (starting_x is not None and starting_y is not None):
                    for s in assignment['assignment']['steps']:
                        step_command = s['command']
                        step_params = s['params']
                        logging.getLogger(__name__).info(f"Executing {step_command} with params {step_params}")
                        
                        if not self.__execute_step(assignment, step_command, step_params, pilot_nav):
                            logging.getLogger(__name__).error(f"Failed to Execute {step_command} with params {step_params}")

                            self.return_cameras_to_nav_position()
                            success = False
                            break
                        else:
                            self.return_cameras_to_nav_position()
                        
                    # if it was successful, mark the assignment complete
                    if success:
                        self.__resources.complete_assignment(assignment)
                else:
                    logging.getLogger(__name__).error("Location is required for this assignment, but not currently available")
                        
            except KeyboardInterrupt:
                logging.getLogger(__name__).info("Keyboard interrupt")
                self.__running = False
            except Exception as e:
                logging.getLogger(__name__).warning(f"Exception thrown: {e}")
                traceback.print_exc()            
            
            # cleanup so nav system can be reinitialized
            if pilot_nav is not None:
                pilot_nav.cleanup()
                
