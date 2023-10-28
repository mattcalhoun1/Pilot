import unittest
from field.field_map import FieldMap
import logging

class TestFieldMap(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

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

    def test_near_boundaries (self):
        curr_map = self.get_basic_map()

        self.assertTrue(curr_map.is_near_bounds(x=11, y=22))
        self.assertFalse(curr_map.is_near_bounds(x=13, y=22))

    def test_obstacles (self):
        curr_map = self.get_basic_map()

        # 0,0 is blocked by a post
        is_blocked, obstacle_id = curr_map.is_blocked(0,0)
        self.assertTrue(is_blocked)
        self.assertEqual('post', obstacle_id)

        # 4,4 is open
        is_blocked, obstacle_id = curr_map.is_blocked(4,4)
        self.assertFalse(is_blocked)

        # 5, -21 is blocked by a tree
        is_blocked, obstacle_id = curr_map.is_blocked(5, -21)
        self.assertTrue(is_blocked)
        self.assertEqual('tree', obstacle_id)

    def test_path_blocked (self):
        curr_map = self.get_basic_map()

        # anything passing directly through 0,0 is blocked by post
        is_blocked, obstacle_id = curr_map.is_path_blocked(100, 100, -100, -100)
        self.assertTrue(is_blocked)
        self.assertEqual('post', obstacle_id)

        is_blocked, obstacle_id = curr_map.is_path_blocked(0, 100, 0, -100)
        self.assertTrue(is_blocked)
        self.assertEqual('post', obstacle_id)

        is_blocked, obstacle_id = curr_map.is_path_blocked(100, 0, -100, 0)
        self.assertTrue(is_blocked)
        self.assertEqual('post', obstacle_id)

        # going above obstacle is ok
        is_blocked, obstacle_id = curr_map.is_path_blocked(100, 10, -100, 10)
        self.assertFalse(is_blocked)

    def test_path_blocked_width (self):
        curr_map = self.get_basic_map()

        path_width = 10.0

        # anything passing directly through 0,0 is blocked by post
        is_blocked, obstacle_id = curr_map.is_path_blocked(100, 100, -100, -100, path_width=path_width)
        self.assertTrue(is_blocked)
        self.assertEqual('post', obstacle_id)

        # going slightly above should blocked
        is_blocked, obstacle_id = curr_map.is_path_blocked(100, 3, -100, 3, path_width=path_width)
        self.assertTrue(is_blocked)
        self.assertEqual('post', obstacle_id)

        # should be allowed
        is_blocked, obstacle_id = curr_map.is_path_blocked(100, path_width + .5, -100, path_width +.5, path_width=path_width)
        self.assertFalse(is_blocked)

        # diagonal but passing closely should be blocked
        is_blocked, obstacle_id = curr_map.is_path_blocked(101, 100, -101, -100, path_width=path_width)
        self.assertTrue(is_blocked)
        self.assertEqual('post', obstacle_id)



    def test_quadrants (self):
        curr_map = self.get_basic_map()

        # quadrant order:  II | I
        #                  -------
        #                 III | IV        

        self.assertEqual(2, curr_map.get_quadrant('n1'))
        self.assertEqual(3, curr_map.get_quadrant('n2'))
        self.assertEqual(4, curr_map.get_quadrant('e1'))        

    def test_distance(self):
        curr_map = self.get_basic_map()
        self.assertEquals(134.3, round(curr_map.get_distance('n1', 'n2'),1))
        self.assertEquals(134.3, round(curr_map.get_distance('n2', 'n1'),1))
        self.assertEquals(294.0, round(curr_map.get_distance('n2', 'e1'),1))