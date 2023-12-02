import unittest
from field.field_map import FieldMap
from field.field_scaler import FieldScaler
import logging

class TestFieldScaler(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()
    
    def test_get_nearest_travelable_lvps_coords (self):
        scaler = FieldScaler(self.get_basic_map(), 640, 640, 1)

        # should be a clear path anywhere toward the top
        target_x = 55.0
        target_y = 60.0
        clear_x, clear_y = scaler.get_nearest_travelable_lvps_coords (
            starting_x = 50.0,
            starting_y = 50.0,
            target_x = target_x,
            target_y = target_y,
            max_dist=500)
        
        logging.getLogger(__name__).info(f"Nearest to {target_x},{target_y} is {clear_x},{clear_y}")

        self.assertEqual(round(clear_x,1), round(target_x,1))
        self.assertEqual(round(clear_y,1), round(target_y,1))

    def test_get_nearest_travelable_lvps_coords_too_far (self):
        scaler = FieldScaler(self.get_basic_map(), 640, 640, 1)

        # should be a clear path anywhere toward the top
        target_x = 75.0
        target_y = 85.0
        clear_x, clear_y = scaler.get_nearest_travelable_lvps_coords (
            starting_x = 50.0,
            starting_y = 50.0,
            target_x = target_x,
            target_y = target_y,
            max_dist=10)
        
        logging.getLogger(__name__).info(f"Nearest to {target_x},{target_y} is {clear_x},{clear_y}")

        self.assertLess(round(clear_x,1), round(target_x,1))
        self.assertLess(round(clear_y,1), round(target_y,1))


    def test_get_nearest_travelable_lvps_coords_backward (self):
        scaler = FieldScaler(self.get_basic_map(), 640, 640, 1)

        # should be a clear path anywhere toward the top
        target_x = 50.0
        target_y = 50.0
        clear_x, clear_y = scaler.get_nearest_travelable_lvps_coords (
            starting_x = 75.0,
            starting_y = 85.0,
            target_x = target_x,
            target_y = target_y,
            max_dist=100)
        
        logging.getLogger(__name__).info(f"Nearest to {target_x},{target_y} is {clear_x},{clear_y}")

        self.assertEqual(round(clear_x,1), round(target_x,1))
        self.assertEqual(round(clear_y,1), round(target_y,1))

    def test_get_nearest_travelable_lvps_coords_backward_too_far (self):
        scaler = FieldScaler(self.get_basic_map(), 640, 640, 1)

        # should be a clear path anywhere toward the top
        target_x = 50.0
        target_y = 50.0
        clear_x, clear_y = scaler.get_nearest_travelable_lvps_coords (
            starting_x = 75.0,
            starting_y = 85.0,
            target_x = target_x,
            target_y = target_y,
            max_dist=10)
        
        logging.getLogger(__name__).info(f"Nearest to {target_x},{target_y} is {clear_x},{clear_y}")

        self.assertGreater(round(clear_x,1), round(target_x,1))
        self.assertGreater(round(clear_y,1), round(target_y,1))


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
            boundaries={'xmin':-10, 'ymin':-20, 'xmax':200, 'ymax':250},
            obstacles={
                'tree':{'xmin':5, 'ymin':-22, 'xmax':7, 'ymax':-21},
                'post':{'xmin':-2, 'ymin':-2, 'xmax':2, 'ymax':2}
            },
            near_boundaries={'xmin':-12, 'ymin':-22, 'xmax':12, 'ymax':22}
        )