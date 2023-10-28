import unittest
from field.field_map import FieldMap
from planner.assignment import *
from planner.goals import *
import logging

class TestGoals(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    
    def __get_vehicles (self):
        return [
            AssignableVehicle('Tank', 0.0, 0.0, sight_range=20.0),
            AssignableVehicle('MecCar', 20.0, 20.0, sight_range=25.0)
        ]
    
    def test_search_generate_assignments (self):
        max_subdivision = 12*10*12*10 # 14,400
        goal = SearchGoal(
            field_map_id='basement',
            field_map=self.get_basic_map(),
            target_objects=['gazing_ball'],
            max_search_area=max_subdivision)
        
        vehicles = self.__get_vehicles()

        # we have 2 vehicles and the search space is way too big, so it should be divided.
        # the search space is 240x288=69,120 square inches. max search space is 14,400.
        # make sure we git at least 4 spaces and none of them are larger than 14,400
        assignments = goal.generate_assignments(vehicles=vehicles)

        #logging.getLogger(__name__).info(f"Space assignments: {space_assignments}")
        # make sure the spaces were divided equally
        self.assertEqual(len(assignments.keys()), 2)
        #for vid in assignments:
        #    for a in assignments[vid]:
        #        logging.getLogger(__name__).info(f"Vehicle: {vid}")
        #        for t in a.tasks:
        #            logging.getLogger(__name__).info(f"{t.task_type} ({t.details})")


    def test_search_space_assignments (self):
        max_subdivision = 12*10*12*10 # 14,400
        goal = SearchGoal(
            field_map_id='basement',
            field_map=self.get_basic_map(),
            target_objects=['gazing_ball'],
            max_search_area=max_subdivision)
        
        vehicles = self.__get_vehicles()

        # we have 2 vehicles and the search space is way too big, so it should be divided.
        # the search space is 240x288=69,120 square inches. max search space is 14,400.
        # make sure we git at least 4 spaces and none of them are larger than 14,400
        spaces = goal.divide_search_spaces(vehicles=vehicles)

        space_assignments = goal.get_space_assignments (vehicles = vehicles, all_spaces = spaces)

        #logging.getLogger(__name__).info(f"Space assignments: {space_assignments}")
        # make sure the spaces were divided equally
        self.assertEqual(len(space_assignments.keys()), 2)
        self.assertEqual(len(space_assignments['Tank']), len(space_assignments['MecCar']))

    def test_search_space_assignment_tiny (self):
        max_subdivision = 12*10*12*10 # 14,400
        goal = SearchGoal(
            field_map_id='basement',
            field_map=self.get_basic_map(),
            target_objects=['gazing_ball'],
            max_search_area=max_subdivision,
            search_spaces=[(10.0,10.0,20.0,20.0)])
        
        # we have 2 vehicles and the search space is way too big, so it should be divided.
        # the search space is 240x288=69,120 square inches. max search space is 14,400.
        # make sure we git at least 4 spaces and none of them are larger than 14,400
        vehicles = self.__get_vehicles()[:1]
        spaces = goal.divide_search_spaces(vehicles=vehicles)
        space_assignments = goal.get_space_assignments (vehicles = vehicles, all_spaces = spaces)

        #logging.getLogger(__name__).info(f"Space assignments: {space_assignments}")
        # make sure the spaces were divided equally
        self.assertEqual(len(space_assignments.keys()), 1)
        self.assertEqual(len(space_assignments['Tank']), 1)

    
    def test_search_divide_spaces (self):
        max_subdivision = 12*10*12*10 # 14,400
        goal = SearchGoal(
            field_map_id='basement',
            field_map=self.get_basic_map(),
            target_objects=['gazing_ball'],
            max_search_area=max_subdivision)
        
        # we have 2 vehicles and the search space is way too big, so it should be divided.
        # the search space is 240x288=69,120 square inches. max search space is 14,400.
        # make sure we git at least 4 spaces and none of them are larger than 14,400
        spaces = goal.divide_search_spaces(self.__get_vehicles())

        self.assertGreaterEqual(len(spaces), 4)
        self.assertLessEqual(len(spaces),16)

        for i,(spx1,spy1,spx2,spy2) in enumerate(spaces):
            #logging.getLogger(__name__).info(f"Space {i}: ({spx1},{spy1}):({spx2},{spy2}) = {self.__get_area(spx1, spy1, spx2, spy2)} sq in")
            self.assertLessEqual(self.__get_area(spx1, spy1, spx2, spy2), max_subdivision)

    def test_search_divide_space_not_necessary (self):
        max_subdivision = 12*10*12*10 # 14,400
        goal = SearchGoal(
            field_map_id='basement',
            field_map=self.get_basic_map(),
            target_objects=['gazing_ball'],
            max_search_area=max_subdivision,
            search_spaces=[(10.0,10.0,20.0,20.0)])
        
        # we have 2 vehicles and the search space is way too big, so it should be divided.
        # the search space is 240x288=69,120 square inches. max search space is 14,400.
        # make sure we git at least 4 spaces and none of them are larger than 14,400
        spaces = goal.divide_search_spaces(self.__get_vehicles()[:1])

        self.assertEqual(len(spaces), 1)

        for i,(spx1,spy1,spx2,spy2) in enumerate(spaces):
            #logging.getLogger(__name__).info(f"Space {i}: ({spx1},{spy1}):({spx2},{spy2}) = {self.__get_area(spx1, spy1, spx2, spy2)} sq in")
            self.assertLessEqual(self.__get_area(spx1, spy1, spx2, spy2), max_subdivision)


    def __get_area(self, x1, y1, x2, y2):
        return (abs(x2-x1)*abs(y2-y1))


    def get_basic_map (self):
        return FieldMap( {
            "n_light": {
                "pattern":"3",
                "type":"light",
                "model":"lights",
                "x":26,
                "y":132,
                "height":43,
                "altitude":40,
                "confidence":0.25
            },
            "e_light": {
                "pattern":"2",
                "type":"light",
                "model":"lights",
                "x":136,
                "y":-28,
                "height":11,
                "altitude":29,
                "confidence":0.25
            },
            "nw_light": {
                "pattern":"4",
                "type":"light",
                "model":"lights",
                "x":-112,
                "y":130,
                "height":42,
                "altitude":21,
                "confidence":0.25
            },                
            "w_tree": {
                "pattern":"na",
                "type":"cat_tree",
                "model":"basement",
                "x":-93,
                "y":-52,
                "height":24.5,
                "altitude":12.25,
                "confidence":0.6
            },   
            "w_house": {
                "pattern":"na",
                "type":"house",
                "model":"basement",
                "x":-57,
                "y":1,
                "height":7.75,
                "altitude":3.875,
                "confidence":0.6
            }},
            boundaries={
                "xmin":-120.0,
                "ymin":-48.0,
                "xmax":120.0,
                "ymax":240.0
            },
            obstacles={
                'tree':{'xmin':5, 'ymin':-22, 'xmax':7, 'ymax':-21},
                'post':{'xmin':-5, 'ymin':-5, 'xmax':5, 'ymax':5}
            }              
        )
