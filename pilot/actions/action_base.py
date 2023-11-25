import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from planner.assignment import *
from position.confidence import Confidence

class ActionBase:
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        self.__vehicle = vehicle
        self.__pilot = pilot
        self.__pilot_nav = pilot_nav
        self.__pilot_logger = pilot_logger
        self.__pilot_config = pilot_config

        # if we get stuck finding position, this allows small movements to get better positioning
        self.__max_positioning_adjustments = self.get_pilot_config()['Driving']['MaxPositioningAdjustments']

        # driving positioning may have lower confidence preferred, to allow less delays in movments
        self.__driving_preferred_position_confidence = self.get_pilot_config()['Driving']['PreferredPositionConfidence']


    ####################
    ## The following methods should be overridden by actions that are subclasses of this
    ####################

    # for logging and display purposes
    def get_name (self) -> str:
        return "Base"

    # This method is called by the pilot in order to fulfill the given action
    def execute (self, params):
        pass

    # whether to record the task as complete, prior to completing it.
    # this is important for multitasking or shutting down the device
    def mark_complete_immediately (self):
        return False


    ####################
    ## End of methods that need to be overridden
    ####################

    def get_vehicle (self):
        return self.__vehicle
    
    def get_pilot (self):
        return self.__pilot
    
    def get_pilot_nav (self) -> PilotNavigation:
        return self.__pilot_nav
    
    def get_pilot_logger (self) -> PilotLogger:
        return self.__pilot_logger
    
    def get_pilot_config (self):
        return self.__pilot_config
   
    def get_vehicle_height (self) -> float:
        return self.__pilot_config['VehicleShape']['Height']

    def get_vehicle_width (self) -> float:
        return self.__pilot_config['VehicleShape']['Width']

    def get_vehicle_length (self) -> float:
        return self.__pilot_config['VehicleShape']['Length']
    
    def get_path_finder (self) -> PathFinder:
        return PathFinder(
            field_map=self.__pilot_nav.get_field_map(), 
            vehicle_width=self.get_vehicle_width(),
            vehicle_height=self.get_vehicle_height(),
            vehicle_length=self.get_vehicle_length())
    
    # triggers pilot to find current position, physically adjusting the vehicle as needed (if configured)
    def find_new_position (self):
        found_position = False
        adjustments = 0
        while not found_position and adjustments <= self.__max_positioning_adjustments:
            driving_confidence = self.__get_confidence(self.__driving_preferred_position_confidence)
            new_x, new_y, new_heading, new_conf, new_basis = self.get_pilot_nav().get_coords_and_heading(preferred_confidence=driving_confidence)
            if new_x is None and adjustments < self.__max_positioning_adjustments:
                adjustments += 1
                adjust_action = self.get_pilot().get_action_factory().get_secondary_action(command=TaskType.AdjustRandomly, params={}, base_action=self)
                adjust_action.execute({})
            elif new_x is not None:
                found_position = True

        if adjustments > 0 and found_position:
            logging.getLogger(__name__).info(f"Found position, but required {adjustments} physical adjustments")
        elif adjustments > 0:
            logging.getLogger(__name__).info(f"Failed to find position after {adjustments} physical adjustments")

        return found_position    
    
    def __get_confidence (confidence_str):
        if confidence_str.lower() == 'medium':
            return Confidence.CONFIDENCE_MEDIUM
        if confidence_str.lower() == 'low':
            return Confidence.CONFIDENCE_LOW
        if confidence_str.lower() == 'high':
            return Confidence.CONFIDENCE_HIGH
        
        return Confidence.CONFIDENCE_FACT

