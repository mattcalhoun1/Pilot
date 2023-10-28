import unittest
from visual.visual_distance import VisualDistanceCalculator
import logging

class TestVisualDistanceCalculator(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def test_estimate_height_above_below (self):
        calc = VisualDistanceCalculator(horizontal_fov = 120.0, vertical_fov = 66.0, view_width=1280.0, view_height=720.0)

        # from below looking up
        ground_distance, top_distance, bottom_distance = calc.estimate_distances_given_viewed_height (
            view_altitude = 7.0, 
            obj_height_degrees = 15.0, 
            obj_known_height = 30.0, 
            obj_center_altitude = 25.0)

        #Ground: 111.4061329455111, Top: 116.19090522873508, Bottom: 111.44651837483705
        self.assertEqual(round(ground_distance,2), 111.41)
        self.assertEqual(round(top_distance,2), 116.19)
        self.assertEqual(round(bottom_distance,2), 111.45)
        

        # from above looking down
        ground_distance, top_distance, bottom_distance = calc.estimate_distances_given_viewed_height (
            view_altitude = 43.0, 
            obj_height_degrees = 15.0, 
            obj_known_height = 30.0, 
            obj_center_altitude = 25.0)

        #Ground: 111.4061329455111, Bottom: 116.19090522873508, Top: 111.44651837483705
        self.assertEqual(round(ground_distance,2), 111.41)
        self.assertEqual(round(bottom_distance,2), 116.19)
        self.assertEqual(round(top_distance,2), 111.45)

    def test_estimate_height_beside (self):
        calc = VisualDistanceCalculator(horizontal_fov = 120.0, vertical_fov = 66.0, view_width=1280.0, view_height=720.0)

        # Ground: 114.51882774215599, Top: 114.62792813026674, Bottom: 115.50565788363247

        # toward the top
        ground_distance, top_distance, bottom_distance = calc.estimate_distances_given_viewed_height (
            view_altitude = 35.0, 
            obj_height_degrees = 15.0, 
            obj_known_height = 30.0, 
            obj_center_altitude = 25.0)

        self.assertEqual(round(ground_distance,2), 114.52)
        self.assertEqual(round(top_distance,2),114.63)
        self.assertEqual(round(bottom_distance,2), 115.51)
        
    def test_estimate_width (self):
        pass

