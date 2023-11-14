import unittest
from position.position_estimator_with_clustering import PositionEstimatorWithClustering
from position.confidence import Confidence
from position.estimator_mode import EstimatorMode
from field.field_map import FieldMap
from camera.camera_info import CameraInfo
from lidar.lidar_map import LidarMap
from navsvc.nav_json_encoder import NavJsonEncoder
import logging
import json
import numpy as np

class TestPositionEstimatorWithClusteringNewMap(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        self.__camera_config_id = 'PI2_STD_HQ_CROPPED_0'
        self.__fov_horz = CameraInfo.get_fov_horizontal(self.__camera_config_id)
        self.__fov_vert = CameraInfo.get_fov_vertical(self.__camera_config_id)
        self.__view_height = CameraInfo.get_resolution_height(self.__camera_config_id)
        self.__view_width = CameraInfo.get_resolution_width(self.__camera_config_id)

        return super().setUp()



    def Xtest_coords_and_heading (self):
        curr_map = self.get_map()

        # the second instance of n1 should not be taken into account for
        # distances, since it has lower confidence
        located_objects =  [
            {
                "sw_light": {
                    "id": "sw_light",
                    "x1": 1100.2,
                    "x2": 1151.6,
                    "y1": 141.46,
                    "y2": 274.3,
                    "time": 1699889375.1,
                    "priority": 8,
                    "confidence": 0.42,
                    "camera_heading": 122.0
                }
            },
            {
                "se_light": {
                    "id": "se_light",
                    "x1": 796.57,
                    "x2": 860.84,
                    "y1": 382.25,
                    "y2": 553.27,
                    "time": 1699889398.6,
                    "priority": 10,
                    "confidence": 0.8,
                    "camera_heading": 79.0
                }
            },
            {
                "nw_tree": {
                    "id": "nw_tree",
                    "x1": 547.89,
                    "x2": 770.31,
                    "y1": 228.66,
                    "y2": 518.64,
                    "time": 1699889414.0,
                    "priority": 7,
                    "confidence": 0.79,
                    "camera_heading": 179.0
                }
            }
        ]

        # get visual degrees for each point
        estimator = PositionEstimatorWithClustering(curr_map, horizontal_fov = self.__fov_horz, vertical_fov = self.__fov_vert, view_width=self.__view_width, view_height=self.__view_height)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, estimator_mode = EstimatorMode.PRECISE)

        logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        # actual coordinates are ~ (75, 150) with heading of around -180 (or +180. same thing)
        self.assertGreaterEqual(x,50)
        self.assertLessEqual(x,110)

        self.assertGreaterEqual(y,100)
        self.assertLessEqual(y,200)

        self.assertGreaterEqual(abs(heading),150)
        self.assertLessEqual(abs(heading),180)
        self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")

    def test_coords_and_heading_nw_pointing_s (self):
        curr_map = self.get_map()

        # entry 212 from Observer/LidarFix session
        # the second instance of n1 should not be taken into account for
        # distances, since it has lower confidence
        located_objects =  [
          {
            "nw_tree": {
                "id": "nw_tree",
                "x1": 1550.39,
                "x2": 1820.77,
                "y1": 271.96,
                "y2": 564.87,
                "time": 1699990862.3,
                "priority": 7,
                "confidence": 0.99,
                "camera_heading": 122.0 + 3 # adjusted based on visual
            }
          },
          {
            "se_light": {
                "id": "se_light",
                "x1": 343.66,
                "x2": 399.57,
                "y1": 405.28,
                "y2": 583.65,
                "time": 1699990866.4,
                "priority": 10,
                "confidence": 0.77,
                "camera_heading": 56.0 - 5# adjusted based onvisual
            }
          },
          {
            "sw_light": {
                "id": "sw_light",
                "x1": 587.3,
                "x2": 640.29,
                "y1": 225.04,
                "y2": 351.61,
                "time": 1699990880.8,
                "priority": 8,
                "confidence": 0.48,
                "camera_heading": 99.0
            }
          }
        ]

        # Add some lidar measurements
        lidar_map = self.__generate_lidar({
            46.5:6184.9, # cat tree
            49.5:6184.9, # cat tree, manually found angle

            #314.25:6492.748 # se light - actual, incorrect lidar reading
            314.25:5842.0, # corrected lidar measurement
            309.25:5842.0 # manually adjusted angle, based on visual
        })

        # get visual degrees for each point
        estimator = PositionEstimatorWithClustering(curr_map, horizontal_fov = self.__fov_horz, vertical_fov = self.__fov_vert, view_width=self.__view_width, view_height=self.__view_height)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, estimator_mode = EstimatorMode.FAST, lidar_map=lidar_map)

        logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        # actual coordinates are ~ (80, 200) with heading of around -180 (or +180. same thing)
        self.assertGreaterEqual(x,50)
        self.assertLessEqual(x,110)

        self.assertGreaterEqual(y,150)
        self.assertLessEqual(y,250)

        self.assertGreaterEqual(abs(heading),150)
        self.assertLessEqual(abs(heading),180)
        self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")

    def __generate_lidar (self, measurement_map):
        lidar_data = []
        for deg in np.arange(0, 360.25, .25):
            if deg in measurement_map:
                lidar_data.append(f"{measurement_map[deg]}")
            else:
                lidar_data.append(f"{-1.0}")
        return LidarMap(0, 0.25, '|'.join(lidar_data))

    def get_map (self):
        return FieldMap( 
            landmarks= {
                "w_windmill": {
                    "pattern":"na",
                    "type":"windmill",
                    "model":"basement",
                    "x":-134.5,
                    "y":0,
                    "height":12.0,
                    "altitude":6.0,
                    "confidence":0.6,
                    "lidar_visible":True,
                    "priority":6
                },
                "nw_tree": {
                    "pattern":"na",
                    "type":"cat_tree",
                    "model":"basement",
                    "x":-145,
                    "y":155,
                    "height":24,
                    "altitude":12,
                    "confidence":0.6,
                    "lidar_visible":True,
                    "priority":7
                },
                "se_pineapple": {
                    "pattern":"na",
                    "type":"pineapple",
                    "model":"basement",
                    "x":102,
                    "y":-87,
                    "height":14.0,
                    "altitude":7.0,
                    "confidence":0.6,
                    "lidar_visible":True,
                    "priority":2
                },   
                "nw_house": {
                    "pattern":"na",
                    "type":"house",
                    "model":"basement",
                    "x":-71,
                    "y":75,
                    "height":7,
                    "altitude":3.875,
                    "confidence":0.6,
                    "lidar_visible":False,
                    "priority":1
                },
                "sw_light": {
                    "pattern":"sideways_triangle_left",
                    "type":"light",
                    "model":"lights",
                    "x":-136,
                    "y":-80,
                    "height":15.5,
                    "altitude":25.5,
                    "confidence":0.25,
                    "lidar_visible":True,
                    "priority":8
                },	
                "ne_light": {
                    "pattern":"square",
                    "type":"light",
                    "model":"lights",
                    "x":125,
                    "y":176,
                    "height":16,
                    "altitude":15,
                    "confidence":0.25,
                    "lidar_visible":True,
                    "priority":9
                },
                "n_light": {
                    "pattern":"3",
                    "type":"light",
                    "model":"lights",
                    "x":-66,
                    "y":288,
                    "height":29,
                    "altitude":36.5,
                    "confidence":0.25,
                    "lidar_visible":True,
                    "priority":10
                },
                "se_light": {
                    "pattern":"sideways_triangle_right",
                    "type":"light",
                    "model":"lights",
                    "x":117,
                    "y":74,
                    "height":15.5,
                    "altitude":16.75,
                    "confidence":0.25,
                    "lidar_visible":True,
                    "priority":10
                }        
            },
            shape="rectangle",
            boundaries =  {
                "xmin":-150,
                "ymin":-50,
                "xmax":150,
                "ymax":250
            },
            near_boundaries = {
                "xmin":-170,
                "ymin":-100,
                "xmax":170,
                "ymax":270
            },
            obstacles= {
                "christmas_tree": {
                    "xmin":110,
                    "ymin":-17,
                    "xmax":150,
                    "ymax":25
                },
                "old_stereo": {
                    "xmin":136,
                    "ymin":25,
                    "xmax":150,
                    "ymax":180
                },
                "pool_table": {
                    "xmin":16,
                    "ymin":180,
                    "xmax":150,
                    "ymax":250
                },
                "work_area": {
                    "xmin":-150,
                    "ymin":-50,
                    "xmax":-130,
                    "ymax":135
                },
                "fp_house": {
                    "xmin":-74,
                    "ymin":65,
                    "xmax":-64,
                    "ymax":74
                },
                "fp_tree": {
                    "xmin":90,
                    "ymin":99,
                    "xmax":107,
                    "ymax":116
                },
                "post_center": {
                    "xmin":-3,
                    "ymin":-3,
                    "xmax":3,
                    "ymax":3
                },
                "post_north": {
                    "xmin":-3,
                    "ymin":143,
                    "xmax":3,
                    "ymax":150
                }
            },
            search={
                "gazing_ball": {
                        "pattern":"na",
                        "model":"basement",
                        "height":12.0,
                        "confidence":0.6,
                        "lidar_visible":False
                },
                "cone": {
                        "pattern":"na",
                        "model":"basement",
                        "height":12.0,
                        "confidence":0.6,
                        "lidar_visible":True
                },
                "speaker": {
                        "pattern":"na",
                        "model":"basement",
                        "height":8.5,
                        "confidence":0.6,
                        "lidar_visible":False
                }    
            }
        )