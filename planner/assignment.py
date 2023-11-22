class TaskType:
    Go = 'GO'
    LogPosition = 'POSITION'
    Face = 'FACE'
    LogLidar = 'LIDAR'
    Search = 'SEARCH'
    BeginControlledMode = 'CONTROLLED'
    BeginAutonomousMode = 'AUTONOMOUS'
    Shutdown = 'SHUTDOWN'
    Sleep = 'SLEEP'

class Task:
    def __init__(self, task_type, task_details = {}):
        self.task_type = task_type
        self.details = task_details

class AssignableVehicle:
    def __init__(self, vehicle_id, current_position_x, current_position_y, sight_range):
        self.vehicle_id = vehicle_id
        self.last_position_x = current_position_x
        self.last_position_y = current_position_y
        self.sight_range = sight_range

class Assignment:
    def __init__(self, vehicle : AssignableVehicle, tasks : list):
        self.vehicle = vehicle
        self.tasks = tasks

