import logging
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from pilot.path_finder import PathFinder
from pilot.pilot import Pilot

class ActionBase:
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot : Pilot) :
        self.__vehicle = vehicle
        self.__pilot = pilot
        self.__pilot_nav = pilot_nav
        self.__pilot_logger = pilot_logger
        self.__pilot_config = pilot_config

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
    
    def get_pilot (self) -> Pilot:
        return self.__pilot
    
    def get_pilot_nav (self) -> PilotNavigation:
        return self.__pilot_nav
    
    def get_pilot_logger (self) -> PilotLogger:
        return self.__pilot_logger
    
    def get_pilot_config (self):
        return self.__pilot_config
   
    def get_vehicle_height (self) -> float:
        return self.__pilot_config['Vehicle']['Height']

    def get_vehicle_width (self) -> float:
        return self.__pilot_config['Vehicle']['Width']

    def get_vehicle_length (self) -> float:
        return self.__pilot_config['Vehicle']['Length']
    
    def get_path_finder (self) -> PathFinder:
        return PathFinder(
            field_map=self.__pilot_nav.get_field_map(), 
            vehicle_width=self.get_vehicle_width(),
            vehicle_height=self.get_vehicle_height(),
            vehicle_length=self.get_vehicle_length())