import unittest
from position.position_estimator import Confidence
from lidar.lidar_map import LidarMap
from field.field_map import FieldMap
import logging
import numpy as np
from pilot.pilot import Pilot
from arduino.test.test_car import TestCar
import logging
import time
import statistics
import platform

class TestPilotNavigation(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def __getNavigation (self):
        car = TestCar()
        pilot_settings_file = '/home/matt/projects/Pilot/pilot/test/resources/pilot_settings.json'
        if platform.processor() == 'aarch64':
	        pilot_settings_file = '/home/matt/projects/Pilot/pilot/test/resources/nano_pilot_settings.json'
        p = Pilot(pilot_settings_file, vehicle=car, use_cached_maps=True, use_cached_models=True)
        default_map = 'basement'
        return p.get_navigator(default_map)


    def __get_visual_coord_samples (self, nav, combined_landmarks, num_samples = 5):
        x_vals = []
        y_vals = []
        headings = []

        # Get position multiple times
        for attempt in range(num_samples):
            # a sample given only two points with only visual data
            x, y, heading, confidence = nav.get_coords_and_heading_for_landmarks (combined_landmarks, allow_lidar = False)
            if x is not None and y is not None:
                x_vals.append(x)
                y_vals.append(y)
                headings.append(heading)

                # confidnce should be low since we only have 2 measuremnts and it's entirely visual
                self.assertEqual(confidence, Confidence.CONFIDENCE_LOW)

        return x_vals, y_vals, headings

    def __get_visual_coord_samples_with_lidar (self, nav, combined_landmarks, lidar_map, num_samples = 2):
        x_vals = []
        y_vals = []
        headings = []

        # Get position multiple times
        for attempt in range(num_samples):
            # a sample given only two points with only visual data
            x, y, heading, confidence = nav.get_coords_and_heading_for_landmarks (combined_landmarks, allow_lidar = True, lidar_map = lidar_map)
            if x is not None and y is not None:
                x_vals.append(x)
                y_vals.append(y)
                headings.append(heading)

                # confidnce should be at least medium since we have lidar data
                self.assertGreaterEqual(confidence, Confidence.CONFIDENCE_MEDIUM)

        return x_vals, y_vals, headings

    def XtestGetCoordsGivenLandmarksNoLidar (self):
        nav = self.__getNavigation()
        # attempt to get coordinates for a given set of data
        landmarks_from_nw_view = [{'nw_house': {'id': 'nw_house', 'time': 1698322229.0, 'x1': 435.36268615722656, 'x2': 552.5671234130859, 'y1': 354.53688526153564, 'y2': 444.97687911987305, 'confidence': 0.91126066, 'camera_heading': 122.0}}, {'ne_tree': {'id': 'ne_tree', 'time': 1698322230.9, 'x1': 830.6252746582031, 'x2': 1022.7104187011719, 'y1': 296.8298234939575, 'y2': 496.44701957702637, 'confidence': 0.9999117, 'camera_heading': 56.0}}]
        x_vals, y_vals, headings = self.__get_visual_coord_samples(nav, landmarks_from_nw_view)

        # based on this map, the heading should be ~-150 degrees and position should be ~(-80, 240)
        heading_tolerance = .05
        xy_tolerance = .18 # very high tolerance since there is no lidar and only two landmarks
        expected_heading = -150.0
        expected_x = -80.0
        expected_y = 240.0
        map_width = nav.get_field_map().get_width()
        map_height = nav.get_field_map().get_length()

        logging.getLogger(__name__).info(f"Mean X: {statistics.mean(x_vals)}, Mean Y: {statistics.mean(y_vals)}, Mean heading: {statistics.mean(headings)}")

        self.assertGreaterEqual(statistics.mean(headings), expected_heading - (360.0 * heading_tolerance))
        self.assertLessEqual(statistics.mean(headings), expected_heading + (360.0 * heading_tolerance))

        self.assertGreaterEqual(statistics.mean(x_vals), expected_x - (map_width * xy_tolerance))
        self.assertLessEqual(statistics.mean(x_vals), expected_x + (map_width * xy_tolerance))

        self.assertGreaterEqual(statistics.mean(y_vals), expected_y - (map_height * xy_tolerance))
        self.assertLessEqual(statistics.mean(y_vals), expected_y + (map_height * xy_tolerance))

    def XtestGetCoordsGivenLandmarksNoLidar_BelowAxis(self):
        nav = self.__getNavigation()
        # attempt to get coordinates for a given set of data
        landmarks_from_s_view = [{'ne_tree': {'id': 'ne_tree', 'time': 1698328517.9, 'x1': 438.7635955810547, 'x2': 632.5604095458984, 'y1': 450.77611541748047, 'y2': 688.3780517578125, 'confidence': 0.99950165, 'camera_heading': 123.0}}, {'nw_house': {'id': 'nw_house', 'time': 1698328522.7, 'x1': 747.1549987792969, 'x2': 859.6235046386719, 'y1': 739.6250495910645, 'y2': 834.1692352294922, 'confidence': 0.9813694, 'camera_heading': 57.0}}]
        x_vals, y_vals, headings = self.__get_visual_coord_samples(nav, landmarks_from_s_view)

        # based on this map, the heading should be ~-150 degrees and position should be ~(-80, 240)
        heading_tolerance = .05
        xy_tolerance = .18 # very high tolerance since there is no lidar and only two landmarks
        expected_heading = -0.1
        expected_x = 10.0
        expected_y = -75.0
        map_width = nav.get_field_map().get_width()
        map_height = nav.get_field_map().get_length()

        logging.getLogger(__name__).info(f"Mean X: {statistics.mean(x_vals)}, Mean Y: {statistics.mean(y_vals)}, Mean heading: {statistics.mean(headings)}")

        self.assertGreaterEqual(statistics.mean(headings), expected_heading - (360.0 * heading_tolerance))
        self.assertLessEqual(statistics.mean(headings), expected_heading + (360.0 * heading_tolerance))

        self.assertGreaterEqual(statistics.mean(x_vals), expected_x - (map_width * xy_tolerance))
        self.assertLessEqual(statistics.mean(x_vals), expected_x + (map_width * xy_tolerance))

        self.assertGreaterEqual(statistics.mean(y_vals), expected_y - (map_height * xy_tolerance))
        self.assertLessEqual(statistics.mean(y_vals), expected_y + (map_height * xy_tolerance))

    def XtestGetCoordsGivenLandmarksNoLidar_BelowAxisFacingNW (self):
        nav = self.__getNavigation()
        # attempt to get coordinates for a given set of data
        landmarks_from_s_view_toward_nw = [{'ne_tree': {'id': 'ne_tree', 'time': 1698329632.8, 'x1': 1376.6650085449219, 'x2': 1536.2433471679688, 'y1': 463.8591842651367, 'y2': 709.3725128173828, 'confidence': 0.9988494, 'camera_heading': 123.0}}, {'w_windmill': {'id': 'w_windmill', 'time': 1698329637.7, 'x1': 668.4318695068359, 'x2': 743.1235198974609, 'y1': 686.303129196167, 'y2': 816.0226879119873, 'confidence': 0.9408851, 'camera_heading': 57.0}}]
        x_vals, y_vals, headings = self.__get_visual_coord_samples(nav, landmarks_from_s_view_toward_nw)

        # based on this map, the heading should be ~-150 degrees and position should be ~(-80, 240)
        heading_tolerance = .05
        xy_tolerance = .18 # very high tolerance since there is no lidar and only two landmarks
        expected_heading = -25.0
        expected_x = 10.0
        expected_y = -75.0
        map_width = nav.get_field_map().get_width()
        map_height = nav.get_field_map().get_length()

        logging.getLogger(__name__).info(f"Mean X: {statistics.mean(x_vals)}, Mean Y: {statistics.mean(y_vals)}, Mean heading: {statistics.mean(headings)}")

        self.assertGreaterEqual(statistics.mean(headings), expected_heading - (360.0 * heading_tolerance))
        self.assertLessEqual(statistics.mean(headings), expected_heading + (360.0 * heading_tolerance))

        self.assertGreaterEqual(statistics.mean(x_vals), expected_x - (map_width * xy_tolerance))
        self.assertLessEqual(statistics.mean(x_vals), expected_x + (map_width * xy_tolerance))

        self.assertGreaterEqual(statistics.mean(y_vals), expected_y - (map_height * xy_tolerance))
        self.assertLessEqual(statistics.mean(y_vals), expected_y + (map_height * xy_tolerance))

    def __generate_lidar (self, measurement_map):
        lidar_data = []
        for deg in np.arange(0, 360.25, .25):
            if deg in measurement_map:
                lidar_data.append(f"{measurement_map[deg]}")
            else:
                lidar_data.append(f"{-1.0}")
        return LidarMap(0, 0.25, '|'.join(lidar_data))


    def testGetCoordinatesInCenter_WithLidar (self):
        nav = self.__getNavigation()
        # attempt to get coordinates for a given set of data
        landmarks_from_s_view_from_center = [{'n_light': {'id': 'n_light', 'time': 1698412369.6, 'x1': 793.7166442871094, 'x2': 836.647694905599, 'y1': 14.968448013067245, 'y2': 327.3316125869751, 'confidence': 0.6978665, 'camera_heading': 56.0}}, {'nw_light': {'id': 'nw_light', 'time': 1698412396.6, 'x1': 52.489739418029785, 'x2': 110.7551924387614, 'y1': 136.43402910232544, 'y2': 288.6086940765381, 'confidence': 0.98032725, 'camera_heading': 78.0}}, {'ne_tree': {'id': 'ne_tree', 'time': 1698412442.7, 'x1': 94.1880874633789, 'x2': 551.3310852050781, 'y1': 90.39797115325928, 'y2': 601.922721862793, 'confidence': 0.99436563, 'camera_heading': 229.0}}, {'se_pineapple': {'id': 'se_pineapple', 'time': 1698412442.7, 'x1': 1472.428955078125, 'x2': 1535.0650024414062, 'y1': 408.0011987686157, 'y2': 513.0489921569824, 'confidence': 0.86597323, 'camera_heading': 229.0}}, {'nw_house': {'id': 'nw_house', 'time': 1698412450.7, 'x1': 779.6971130371094, 'x2': 863.4103698730469, 'y1': 531.2561702728271, 'y2': 649.1806011199951, 'confidence': 0.7989067, 'camera_heading': -51.0}}]
        #INFO:Substituting Lidar reading of 181.66338582677167 in place of visual estimate of 185.7450190274486.
        #INFO:Substituting Lidar reading of 168.0708661417323 in place of visual estimate of 118.71849828385389.

        lidar_map = self.__generate_lidar({0.25:100.0})

        x_vals, y_vals, headings = self.__get_visual_coord_samples_with_lidar(nav, landmarks_from_s_view_from_center, lidar_map)

        heading_tolerance = .1
        xy_tolerance = .15 
        expected_heading = -1.0
        expected_x = 10.0
        expected_y = 150.0
        map_width = nav.get_field_map().get_width()
        map_height = nav.get_field_map().get_length()

        try:
            logging.getLogger(__name__).info(f"Mean X: {statistics.mean(x_vals)}, Mean Y: {statistics.mean(y_vals)}, Mean heading: {statistics.mean(headings)}")

            self.assertGreaterEqual(statistics.mean(headings), expected_heading - (360.0 * heading_tolerance))
            self.assertLessEqual(statistics.mean(headings), expected_heading + (360.0 * heading_tolerance))

            self.assertGreaterEqual(statistics.mean(x_vals), expected_x - (map_width * xy_tolerance))
            self.assertLessEqual(statistics.mean(x_vals), expected_x + (map_width * xy_tolerance))

            self.assertGreaterEqual(statistics.mean(y_vals), expected_y - (map_height * xy_tolerance))
            self.assertLessEqual(statistics.mean(y_vals), expected_y + (map_height * xy_tolerance))
        except Exception as e:
            self.fail(f"Exception thrown, maybe no coords returned: {e}")
