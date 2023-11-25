from pilot.actions.search_action import SearchAction
from pilot.actions.face_position_action import FacePositionAction
from pilot.actions.face_heading_action import FaceHeadingAction
from pilot.actions.go_to_position_action import GoToPositionAction
from pilot.actions.log_lidar_action import LogLidarAction
from pilot.actions.log_position_action import LogPositionAction
from pilot.actions.do_nothing_action import DoNothingAction
from pilot.actions.enter_controlled_mode_action import EnterControlledModeAction
from pilot.actions.shutdown_action import ShutdownAction
from pilot.actions.sleep_action import SleepAction
from pilot.actions.adjust_randomly_action import AdjustRandomlyAction
from pilot.actions.action_base import ActionBase
from planner.assignment import *
from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
import logging

class ActionFactory:
    def __init__(self):
        pass

    def get_secondary_action (self, command, params, base_action : ActionBase) -> ActionBase:
        return self.get_action_for(
            command=command,
            params=params,
            vehicle=base_action.get_vehicle(),
            pilot_nav=base_action.get_pilot_nav(),
            pilot_logger=base_action.get_pilot_logger(),
            pilot_config=base_action.get_pilot_config(),
            pilot=base_action.get_pilot())

    def get_action_for(self, command, params, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) -> ActionBase:
        if command == TaskType.Go:
            return GoToPositionAction(vehicle=vehicle, pilot_nav = pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)

        elif command == TaskType.LogPosition:
            return LogPositionAction(vehicle=vehicle, pilot_nav = pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)
    
        elif command == TaskType.Face:
            return FacePositionAction(vehicle=vehicle, pilot_nav = pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)

        elif command == TaskType.FaceHeading:
            return FaceHeadingAction(vehicle=vehicle, pilot_nav = pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)

        elif command == TaskType.LogLidar:
            return LogLidarAction(vehicle=vehicle, pilot_nav = pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)

        elif command == TaskType.Search:
            return SearchAction(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)

        elif command == TaskType.BeginAutonomousMode:
            return DoNothingAction(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)
            
        elif command == TaskType.BeginControlledMode:
            return EnterControlledModeAction(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)

        elif command == TaskType.Shutdown:
            return ShutdownAction(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)

        elif command == TaskType.Sleep:
            return SleepAction(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)

        elif command == TaskType.AdjustRandomly:
            return AdjustRandomlyAction(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)

        logging.getLogger(__name__).warning(f"Command not supported: {command}")
        return DoNothingAction(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger = pilot_logger, pilot_config = pilot_config, pilot=pilot)
