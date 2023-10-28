import unittest
from field.field_map import FieldMap
from planner.planner import *
from planner.assignment import *
import logging

class TestPlanner(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()



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
