import unittest
from pilot.path_finder import PathFinder
from lidar.lidar_map import LidarMap
import logging
import numpy as np

class TestPathFinder(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def test_get_closest_available_angle (self):
        blocked_angles = [0.0, 10.5, 11.0, 11.5, 12.5, 100.0, 200.0, 250.0, 250.5 ]
        blocked_distances = []
        for a in blocked_angles:
            blocked_distances.append(20.0)

        lidar = self.__get_lidar_map_with_paths_blocked(blocked_angles, blocked_distances)

        # direct matches
        self.assertEqual(lidar.get_closest_available_angle(0.0), 0.0)
        self.assertEqual(lidar.get_closest_available_angle(11.0), 11.0)
        self.assertEqual(lidar.get_closest_available_angle(250.5), 250.5)

        # angle before end
        self.assertEqual(lidar.get_closest_available_angle(245.0), 250.0)

        # angle after end
        self.assertEqual(lidar.get_closest_available_angle(265.0), 250.5)

        # angle before beginning
        self.assertEqual(lidar.get_closest_available_angle(-1.0), 0.0)

        # angle toward middle
        self.assertIn(lidar.get_closest_available_angle(12.0), [11.5, 12.5])
        self.assertEqual(lidar.get_closest_available_angle(13.0), 12.5)

    def test_get_measurement(self):
        blocked_angles = [0.0, 10.5, 11.0, 11.5, 12.5, 100.0, 200.0, 250.0, 250.5 ]
        blocked_distances = []
        for a in blocked_angles:
            blocked_distances.append(a+1.0)

        lidar = self.__get_lidar_map_with_paths_blocked(blocked_angles, blocked_distances)
        
        # make sure drift check hides useless measurements
        self.assertEqual(lidar.get_measurement(5.0, 0.5), -1)

        # direct hit
        self.assertEqual(lidar.get_measurement(10.5, 0.5), 11.5)

        # close enough
        self.assertEqual(lidar.get_measurement(249.5, 0.5), 251.0)



    def __get_lidar_map_with_paths_blocked (self, headings : list, distances : list):
        s_map = ''
        for degree in np.arange(0.0,360.1,0.5):
            if len(s_map) > 0:
                s_map = s_map + '|'
            
            if degree in headings:
                s_map = s_map + f"{distances[headings.index(degree)]}"
                #logging.getLogger(__name__).info(f"Adding block of {distances[headings.index(degree)]} @ {degree} deg")
            else:
                s_map = s_map + "-1.0"
        
        # offset is zero, for this test we assume lidar 0 = front of vehicle
        return LidarMap(0, 0.5, s_map)