from planner.assignment import *
from planner.goals import *

# uses a given map and goal to plan actions
class Planner:
    def __init__(self, vehicles, goals):
        self.__vehicles = vehicles
        self.__goals = goals

    def create_assignments (self):
        pass