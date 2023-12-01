import unittest
from field.field_map import FieldMap
from field.field_scaler import FieldScaler
import logging

class TestFieldScaler(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()
    
    def test_get_nearest_travelable_lvps_coords (self):
        # this method needs tested!

        # i pass it a starting position, ending position, and max distance i want to go.
        # it is returning the wrong direction or soemthign. i think degree/slope calculation is off
        

        pass

    def get_basic_map (self):
        return FieldMap( {
                'n1': {
                    'x':-40.0,
                    'y':38.0,
                    'height':24.0,
                    'width':2.5,
                    'altitude':22.75
                },
                'n2': {
                    'x':-106.0,
                    'y':-79.0,
                    'height':44.0,
                    'width':2.5,
                    'altitude':40.5
                },
                'e1': {
                    'x':152.0,
                    'y':-220.0,
                    'height':24.0,
                    'width':2.5,
                    'altitude':22.75
                }
            },
            boundaries={'xmin':-10, 'ymin':-20, 'xmax':10, 'ymax':20},
            obstacles={
                'tree':{'xmin':5, 'ymin':-22, 'xmax':7, 'ymax':-21},
                'post':{'xmin':-2, 'ymin':-2, 'xmax':2, 'ymax':2}
            },
            near_boundaries={'xmin':-12, 'ymin':-22, 'xmax':12, 'ymax':22}
        )