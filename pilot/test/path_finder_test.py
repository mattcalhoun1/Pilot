import unittest
from pilot.path_finder import PathFinder
from lidar.lidar_map import LidarMap
from field.field_map import FieldMap
import logging
import numpy as np

class TestPathFinder(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def test_direct_path (self):
        path_finder = PathFinder()
        
        heading, distance = path_finder.find_direct_path(0.0, 0.0, 50.0, 50.0)
        
        # should be 45deg, 70.7 dist
        self.assertEqual(round(distance,1), 70.7)
        self.assertEqual(round(heading,1), 45.0)

        heading, distance = path_finder.find_direct_path(0.0, 0.0, -50.0, 50.0)
        
        # should be -45deg, 70.7 dist
        self.assertEqual(round(distance,1), 70.7)
        self.assertEqual(round(heading,1), -45.0)
        
        
        heading, distance = path_finder.find_direct_path(0.0, 0.0, -50.0, -50.0)
        
        self.assertEqual(round(distance,1), 70.7)
        self.assertEqual(round(heading,1), -135.0)

        heading, distance = path_finder.find_direct_path(0.0, 0.0, 50.0, -50.0)
        
        self.assertEqual(round(distance,1), 70.7)
        self.assertEqual(round(heading,1), 135.0)

    def test_find_rotation(self):
        path_finder = PathFinder()
        
        rotation = path_finder.find_rotation(0, 90)
        self.assertEqual(rotation, 90)
        
        rotation = path_finder.find_rotation(0, -90)
        self.assertEqual(rotation, -90)
        
        rotation = path_finder.find_rotation(90, -90)
        self.assertTrue(rotation == -180 or rotation == 180)
        
        rotation = path_finder.find_rotation(-150, 150)
        self.assertEqual(rotation, -60)
        
        rotation = path_finder.find_rotation(-20, 20)
        self.assertEqual(rotation, 40)
        
        rotation = path_finder.find_rotation(150, -150)
        self.assertEqual(rotation, 60)
        
        rotation = path_finder.find_rotation(20, -20)
        self.assertEqual(rotation, -40)

    def test_find_point (self):
        path_finder = PathFinder()

        x,y = path_finder.find_point(0.0, 0.0, 45.0, 6)
        self.assertEqual(round(x), 4)
        self.assertEqual(round(y), 4)
        

        x,y = path_finder.find_point(0.0, 0.0, -45.0, 6)
        self.assertEqual(round(x), -4)
        self.assertEqual(round(y), 4)

        x,y = path_finder.find_point(0.0, 0.0, 135.0, 6)
        self.assertEqual(round(x), 4)
        self.assertEqual(round(y), -4)
        

        x,y = path_finder.find_point(0.0, 0.0, -135.0, 6)
        self.assertEqual(round(x), -4)
        self.assertEqual(round(y), -4)

        x,y = path_finder.find_point(0.0, 0.0, -90.0, 6)
        self.assertEqual(round(x), -6)
        self.assertEqual(round(y), 0)

        x,y = path_finder.find_point(0.0, 0.0, 90.0, 6)
        self.assertEqual(round(x), 6)
        self.assertEqual(round(y), 0)

        x,y = path_finder.find_point(0.0, 0.0, 0.0, 6)
        self.assertEqual(round(x), 0)
        self.assertEqual(round(y), 6)

        x,y = path_finder.find_point(0.0, 0.0, 180.0, 6)
        self.assertEqual(round(x), 0)
        self.assertEqual(round(y), -6)

    # finding potential paths with no boundaries or lidar.
    # should just return direct path
    def test_find_potential_paths (self):
        path_finder = PathFinder()
        paths = path_finder.find_potential_paths (
            start_x=0.0, start_y=0.0,
            end_x=50.0, end_y=50.0,
            lidar_map = None, 
            current_heading = 0.0)
        self.assertEqual(len(paths),1)
    
        self.assertEqual(len(paths[0]), 1)
        (heading,dist,x,y) = paths[0][0]
        self.assertEqual(round(heading),45)
        self.assertEqual(round(dist), 71)
        self.assertEqual(x,50.0)
        self.assertEqual(y,50.0)

        paths = path_finder.find_potential_paths (
            start_x=50.0, start_y=50.0,
            end_x=-50.0, end_y=-50.0,
            lidar_map = None, 
            current_heading = 0.0)
        self.assertEqual(len(paths),1)
    
        self.assertEqual(len(paths[0]), 1)
        (heading,dist,x,y) = paths[0][0]
        self.assertEqual(round(heading),-135)
        self.assertEqual(round(dist), 141)
        self.assertEqual(x,-50.0)
        self.assertEqual(y,-50.0)

    # find potential paths with lidar taken into account
    def test_find_paths_lidar_block (self):
        # wants to go 0,0 - 50,50, so put a block near 25,25 and see how it goes around
        # assuming vehicle facing 90 for purposes of lidar

        blocked_angles = [360.0-45.0, 360.0-45.5,360.0-44.5]
        blocked_distances = [20.0,20.0,20.0]

        lidar = self.__get_lidar_map_with_paths_blocked(blocked_angles, blocked_distances)

        vehicle_heading = 90.0

        # should be given a path that goes above and one that goes below 15,15
        path_finder = PathFinder()
        paths = path_finder.find_potential_paths(
            start_x=0, start_y=0,
            end_x=50.0, end_y=50.0,
            lidar_map=lidar,
            current_heading=vehicle_heading
        )

        self.assertEqual(2, len(paths))
        self.assertEqual(2, len(paths[0]))
        self.assertEqual(2, len(paths[1]))

    # find potential paths with field map boundaries and obstacles taken into account
    def test_find_paths_map_obstacles(self):
        # wants to go 0,0 - 50,50, so put a block near 25,25 and see how it goes around
        # assuming vehicle facing 90 for purposes of lidar

        blocked_angles = [360.0-45.0, 360.0-45.5,360.0-44.5]
        blocked_distances = [20.0,20.0,20.0]
        lidar = self.__get_lidar_map_with_paths_blocked(blocked_angles, blocked_distances)
        
        field_map = self.__get_field_map_with_obstacles()

        vehicle_heading = 90.0

        # not taking lidar into account, the path should go around 0,0, as it's blocked by a post
        path_finder = PathFinder(field_map=field_map, vehicle_width=10.0, vehicle_height=12.0, vehicle_length=20.0)
        paths = path_finder.find_potential_paths(
            start_x=-500.0, start_y=-500.0,
            end_x=500.0, end_y=500.0,
            lidar_map=None,
            current_heading=vehicle_heading
        )

        logging.getLogger(__name__).info(f"Paths: {paths}")

        self.assertEqual(2, len(paths))
        self.assertEqual(2, len(paths[0]))
        self.assertEqual(2, len(paths[1]))        

    # find potential paths with boundaries taken into account

    # returns a lidar map that is wide open except for the given blockages, which must be in .5 increments
    def __get_lidar_map_with_paths_blocked (self, headings : list, distances : list):
        s_map = ''
        for degree in np.arange(0.0,360.1,0.5):
            if len(s_map) > 0:
                s_map = s_map + '|'
            
            if degree in headings:
                s_map = s_map + f"{distances[headings.index(degree)]}"
                logging.getLogger(__name__).info(f"Adding block of {distances[headings.index(degree)]} @ {degree} deg")
            else:
                s_map = s_map + "-1.0"
        
        # offset is zero, for this test we assume lidar 0 = front of vehicle
        return LidarMap(0, 0.5, s_map)

    def __get_field_map_with_obstacles (self):
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
            boundaries={'xmin':-1000, 'ymin':-1000, 'xmax':1000, 'ymax':1000},
            obstacles={
                #'tree':{'xmin':5, 'ymin':-22, 'xmax':7, 'ymax':-21},
                'post':{'xmin':-2, 'ymin':-2, 'xmax':2, 'ymax':2}
            }
        )
