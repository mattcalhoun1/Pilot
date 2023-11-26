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

class TestPositionEstimatorPicam3(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        self.__camera_config_id = 'PI3_STD_HQ_CROPPED_1'
        self.__fov_horz = CameraInfo.get_fov_horizontal(self.__camera_config_id)
        self.__fov_vert = CameraInfo.get_fov_vertical(self.__camera_config_id)
        self.__view_height = CameraInfo.get_resolution_height(self.__camera_config_id)
        self.__view_width = CameraInfo.get_resolution_width(self.__camera_config_id)

        self.__estimator_mode = EstimatorMode.VERY_PRECISE
        self.__adjust_for_altitude = False

        return super().setUp()

    def __get_estimator (self, curr_map):
        return PositionEstimatorWithClustering(
            curr_map, 
            horizontal_fov = self.__fov_horz, 
            vertical_fov = self.__fov_vert, 
            view_width=self.__view_width, 
            view_height=self.__view_height, 
            estimator_mode=self.__estimator_mode,
            adjust_for_altitude=self.__adjust_for_altitude)        

    def test_dead_spot_middle (self):
        curr_map = self.get_new_map()
        located_objects = [
            {'sw_light': {'id': 'sw_light', 'time': 1701029362.9, 'x1': 801.839111328125, 'x2': 1007.0072631835938, 'y1': 314.742484331131, 'y2': 365.58671951293945, 'confidence': 0.56126183, 'camera_heading': 180.0, 'priority': 3, 'tier': 'top'}}, 
            {'n_light': {'id': 'n_light', 'time': 1701029362.9, 'x1': 594.2452239990234, 'x2': 641.7005386352539, 'y1': 230.45100831985474, 'y2': 386.6132640838623, 'confidence': 0.9958093, 'camera_heading': 0.0, 'priority': 1, 'tier': 'top'}}, 
            {'nw_house': {'id': 'nw_house', 'time': 1701029377.1, 'x1': 652.9381866455078, 'x2': 795.46875, 'y1': 459.7609233856201, 'y2': 629.6327991485596, 'confidence': 0.9478439, 'camera_heading': -51.0, 'priority': 7, 'tier': 'mid'}}
        ]

        lidar_map = None

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-60)
        self.assertLessEqual(x,-20)

        self.assertGreaterEqual(y,-20)
        self.assertLessEqual(y,20)

        self.assertGreaterEqual(heading,-33)
        self.assertLessEqual(heading,80)
        self.assertEqual(Confidence.CONFIDENCE_HIGH, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")     

    def Xtest_near_x_axis_facing_ne (self):
        curr_map = self.get_map()
        located_objects = [
            {'ne_light': {'id': 'ne_light', 'time': 1700923739.2, 'x1': 793.4196166992188, 'x2': 891.7283020019531, 'y1': 237.25842475891113, 'y2': 341.336030960083, 'confidence': 0.30915958, 'camera_heading': 56.0, 'priority': 2, 'tier': 'top'}},
            {'nw_house': {'id': 'nw_house', 'time': 1700923749.2, 'x1': 15.082077026367188, 'x2': 790.44287109375, 'y1': 9.769145965576172, 'y2': 557.5698680877686, 'confidence': 0.799291, 'camera_heading': 180.0, 'priority': 7, 'tier': 'mid'}},
            {'n_light': {'id': 'n_light', 'time': 1700923753.9, 'x1': 1076.9485473632812, 'x2': 1134.0702819824219, 'y1': 237.98599433898926, 'y2': 391.4465618133545, 'confidence': 0.6814697, 'camera_heading': 0.0, 'priority': 1, 'tier': 'top'}}, 
            {'sw_light': {'id': 'sw_light', 'time': 1700923768.2, 'x1': 1091.4876098632812, 'x2': 1241.8681335449219, 'y1': 27.39456182718277, 'y2': 285.0166025161743, 'confidence': 0.98301625, 'camera_heading': 229.0, 'priority': 3, 'tier': 'top'}}
        ]        
        lidar_map = self.__generate_lidar({
            150.5:2932.684,#115.46 inches
        })

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-60)
        self.assertLessEqual(x,-20)

        self.assertGreaterEqual(y,-20)
        self.assertLessEqual(y,20)

        self.assertGreaterEqual(heading,-33)
        self.assertLessEqual(heading,80)
        self.assertEqual(Confidence.CONFIDENCE_HIGH, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")     


    def __generate_lidar (self, measurement_map):
        lidar_data = []
        for deg in np.arange(0, 360.25, .25):
            if deg in measurement_map:
                lidar_data.append(f"{measurement_map[deg]}")
            else:
                lidar_data.append(f"{-1.0}")
        return LidarMap(0, 0.25, '|'.join(lidar_data))

    def get_new_map(self):
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
                    "priority":6,
                    "tier":"mid",
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference":165.0
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
                    "priority":8,
                    "tier":"mid",
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference":165.0
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
                    "priority":7,
                    "tier":"mid",
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference":165.0
                },
                "sw_light": {
                    "pattern":"sideways_triangle_left",
                    "type":"light",
                    "model":"lights",
                    "x":-132,
                    "y":-79,
                    "height":15.5,
                    "altitude":33.5,
                    "confidence":0.25,
                    "lidar_visible":True,
                    "priority":3,
                    "tier":"top",
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference":165.0
                },	
                "ne_light": {
                    "pattern":"square",
                    "type":"light",
                    "model":"lights",
                    "x":121.5,
                    "y":174,
                    "height":16,
                    "altitude":15,
                    "confidence":0.25,
                    "lidar_visible":False,
                    "priority":2,
                    "tier":"top",
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference":165.0
                },
                "n_light": {
                    "pattern":"3",
                    "type":"light",
                    "model":"lights",
                    "x":-150,
                    "y":280,
                    "height":25,
                    "altitude":24.25,
                    "confidence":0.25,
                    "lidar_visible":True,
                    "priority":1,
                    "tier":"top",
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference":165.0
                },
                "se_light": {
                    "pattern":"sideways_triangle_right",
                    "type":"light",
                    "model":"lights",
                    "x":111.5,
                    "y":-68.5,
                    "height":15.5,
                    "altitude":16.75,
                    "confidence":0.25,
                    "lidar_visible":False,
                    "priority":4,
                    "tier":"top",
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference":165.0
                }        
            },
            shape="rectangle",
            boundaries = {
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
            obstacles = {
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
                    "xmin":-82,
                    "ymin":75,
                    "xmax":-72,
                    "ymax":82
                },
                "fp_tree": {
                    "xmin":-150,
                    "ymin":153,
                    "xmax":-142,
                    "ymax":169
                },
                "post_center": {
                    "xmin":-4,
                    "ymin":-4,
                    "xmax":4,
                    "ymax":4
                },
                "post_north": {
                    "xmin":-4,
                    "ymin":142,
                    "xmax":4,
                    "ymax":151
                }
            },
            search = {
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
                },
                "pineapple": {
                    "pattern":"na",
                    "model":"basement",
                    "height":14.0,
                    "confidence":0.6,
                    "lidar_visible":True
                }
            }
        )

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
                    "priority":6,
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference": 165.0
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
                    "priority":7,
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference": 165.0
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
                    "priority":2,
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference": 165.0
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
                    "priority":1,
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference": 165.0
                },
                "sw_light": {
                    "pattern":"sideways_triangle_left",
                    "type":"light",
                    "model":"lights",
                    "x":-132,
                    "y":-79,
                    "height":15.5,
                    "altitude":33.5,
                    "confidence":0.25,
                    "lidar_visible":True,
                    "priority":8,
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference": 165.0
                },	
                "ne_light": {
                    "pattern":"square",
                    "type":"light",
                    "model":"lights",
                    "x":121.5,
                    "y":174,
                    "height":16,
                    "altitude":15,
                    "confidence":0.25,
                    "lidar_visible":True,
                    "priority":9,
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference": 165.0
                },
                "n_light": {
                    "pattern":"3",
                    "type":"light",
                    "model":"lights",
                    "x":-68,
                    "y":280,
                    "height":25,
                    "altitude":24.25,
                    "confidence":0.25,
                    "lidar_visible":True,
                    "priority":10,
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference": 165.0
                },
                "se_light": {
                    "pattern":"sideways_triangle_right",
                    "type":"light",
                    "model":"lights",
                    "x":111.5,
                    "y":-68.5,
                    "height":15.5,
                    "altitude":16.75,
                    "confidence":0.25,
                    "lidar_visible":True,
                    "priority":10,
                    "min_visual_angle_preference":15.0,
                    "max_visual_angle_preference": 165.0
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
