import logging
from field.field_map import FieldMap
from planner.assignment import *

class PlannerGoal:
    def __init__(self):
        pass

    def generate_assignments (self, vehicles : list):
        pass

class SearchGoal(PlannerGoal):
    # default to max search box size of 10' by 10'
    def __init__(self, field_map_id, field_map, target_objects, search_spaces = None, max_search_area = 12*10*12*10):
        self.__field_map_id = field_map_id
        self.__field_map = field_map
        self.__target_objects = target_objects
        self.__max_search_area = max_search_area

        # the search cannot be unbounded
        if search_spaces is not None:
            self.__search_spaces = search_spaces
        else:
            map_xmin, map_ymin, map_xmax, map_ymax = field_map.get_boundaries()
            if map_xmin is not None:
                self.__search_spaces = [(map_xmin, map_ymin, map_xmax, map_ymax)]
            else:
                logging.getLogger(__name__).error("Unbounded searches not allowed!")
                raise Exception("Unbounded search not allowed")

    def generate_assignments(self, vehicles: list):
        logging.getLogger(__name__).info(f"Generating search assignments for {len(vehicles)} available vehicles. Looking for {self.__target_objects} on map {self.__field_map_id} within {len(self.__search_spaces)} space(s)")


        # break the search space into multiple spaces, based on max search size and # of vehicles available
        spaces = self.divide_search_spaces(vehicles)

        # divide the spaces up between the vehicles.
        assigned_spaces = self.get_space_assignments(vehicles=vehicles, all_spaces=spaces)
        vehicle_assignments = {}

        for vid in assigned_spaces:
            vehicle_assignments[vid] = []
            for v_space in assigned_spaces[vid]:
                this_vehicle = [* [v for v in vehicles if v.vehicle_id == vid]][0]
                vehicle_assignments[vid].append(self.assign_search_tasks(vehicle=this_vehicle, single_search_space=v_space))

        return vehicle_assignments
    
    def assign_search_tasks (self, vehicle, single_search_space):
        # this asumes the search space is squarish. if it's not, the search coudl be sped up significantly by nto subdividing into quartiles

        # the search grid may be just this search space, if vehicle can see far enough, or may need to be divided smaller
        grid_x = single_search_space[0]
        grid_y = single_search_space[1]
        grid_x2 = single_search_space[2]
        grid_y2 = single_search_space[3]

        grid_height = abs(grid_y2 - grid_y)
        grid_width = abs(grid_x2 - grid_x)
        divide_only_if_necessary = grid_height <= vehicle.sight_range and grid_width <= vehicle.sight_range

        search_sized_spaces = self.__divide_search_space_into_quarters(
            xmin=grid_x, ymin=grid_y, xmax=grid_x2, ymax=grid_y2,
            max_area=(vehicle.sight_range*2)**2, 
            only_if_necessary=divide_only_if_necessary)


        # for each space, drive vehicle to center ant tell it to search 360
        tasks = []
        for x1, y1, x2, y2 in search_sized_spaces:
            center_x = x1 + (x2 - x1)/2
            center_y = y1 + (y2 - y1)/2
            tasks.append(Task(TaskType.Go, task_details={'X':center_x, 'Y':center_y}))
            tasks.append(Task(TaskType.Search, task_details={'objects':self.__target_objects, 'start_heading':0.0, 'end_heading':-1.0}))


        return Assignment(vehicle=vehicle, tasks=tasks)

    def get_space_assignments (self, vehicles: list, all_spaces : list):
        # this is a little simplistic, but sort the search areas by xmin, divide those up between vehicles
        # then after dividdd up by vehicle, further sort by ymin, so the vehicel is always close to the next space

        # this is the tuple sort behavior done by default in python
        sorted_all = sorted(all_spaces)
        unassigned_index = 0
        assigned = {}
        assignments_per_vehicle = int(len(sorted_all)/len(vehicles))

        for v in vehicles:
            assigned[v.vehicle_id] = []
            while unassigned_index < len(sorted_all) and len(assigned[v.vehicle_id]) < assignments_per_vehicle:
                assigned[v.vehicle_id].append(sorted_all[unassigned_index])
                unassigned_index += 1
        
        # any leftovers to go last vehicle
        while unassigned_index < len(sorted_all):
            assigned[vehicles[len(vehicles)-1]].append(unassigned_index)
            unassigned_index += 1

        return assigned





    def divide_search_spaces (self, vehicles):
        # if there is only one vehicle, dont divide search space unless necessary. Otherwise, DO divide so seach can be shared
        spaces = []
        for xmin, ymin, xmax, ymax in self.__search_spaces:
            spaces = spaces + self.__divide_search_space_into_quarters(
                xmin=xmin, ymin=ymin, xmax=xmax, ymax=ymax,
                max_area=self.__max_search_area,
                only_if_necessary=len(vehicles) == 1)
        
        return spaces
    
    def __divide_search_space_into_quarters (self, xmin, ymin, xmax, ymax, max_area, only_if_necessary = True):
        width = abs(xmax - xmin)
        height = abs(ymax - ymin)

        if (width * height <= max_area) and only_if_necessary:
            return [(xmin, ymin, xmax, ymax)]

        new_quartiles = [
            (xmin, ymin, xmin + (width/2), ymin + (height/2)),
            (xmin+(width/2), ymin, xmax, ymin + (height/2)),
            (xmin, ymin+(height/2), xmin + (width/2), ymax),
            (xmin+(width/2), ymin+(height/2), xmax, ymax),
        ]

        final_spaces = []

        # further subdivide any areas too big
        for qx1, qy1, qx2, qy2 in new_quartiles:
            final_spaces = final_spaces + self.__divide_search_space_into_quarters(qx1, qy1, qx2, qy2, max_area=max_area, only_if_necessary=True)
        
        return final_spaces
