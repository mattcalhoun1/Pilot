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

class TestPositionEstimatorWithRotations(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        self.__camera_config_id = 'PI2_STD_HQ_CROPPED_0'
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

    def Xtest_below_axis_facing_nw (self):
        curr_map = self.get_map()
        located_objects = [
            {
            "n_light": {
                "id": "n_light",
                "x1": 492.97,
                "x2": 565.25,
                "y1": 201.3,
                "y2": 402.84,
                "time": 1700507692.4,
                "priority": 10,
                "confidence": 0.97,
                "camera_heading": 122.0
            }
            },
            {
            "w_windmill": {
                "id": "w_windmill",
                "x1": 792.17,
                "x2": 925.39,
                "y1": 660.77,
                "y2": 934.32,
                "time": 1700507696.8,
                "priority": 6,
                "confidence": 0.71,
                "camera_heading": 56.0
            }
            },
            {
            "nw_house": {
                "id": "nw_house",
                "x1": 770.46,
                "x2": 975.81,
                "y1": 593.44,
                "y2": 751.44,
                "time": 1700507711.5,
                "priority": 1,
                "confidence": 1.0,
                "camera_heading": 99.0
            }
            },
            {
            "se_light": {
                "id": "se_light",
                "x1": 583.17,
                "x2": 634.58,
                "y1": 240.59,
                "y2": 550.68,
                "time": 1700507748.6,
                "priority": 10,
                "confidence": 0.99,
                "camera_heading": 228.0
            }
            }            
        ]

        lidar_map = self.__generate_lidar({
            324.0:3101.34,#122.1 inches
            23.5:8557.26#336.9 inches
        })

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-40)
        self.assertLessEqual(x,10)

        self.assertGreaterEqual(y,-70)
        self.assertLessEqual(y,25)

        self.assertGreaterEqual(heading,-60)
        self.assertLessEqual(heading,-20)
        self.assertEqual(Confidence.CONFIDENCE_HIGH, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")     

    def Xtest_10_60_110 (self):
        curr_map = self.get_map()

        # reported coords off, original reported:
        #"position_x": -78.11,
        #"position_y": 105.37,
        #"heading": 91.07,
        #"entry_num": 306,        
        # actual heading somewhere near 110, actual coord near 10,60

        located_objects = [
            {
            "se_light": {
                "id": "se_light",
                "x1": 650.33,
                "x2": 724.44,
                "y1": 244.71,
                "y2": 499.7,
                "time": 1700502312.4,
                "priority": 10,
                "confidence": 0.86,
                "camera_heading": 122.0
            }
            },
            {
            "w_windmill": {
                "id": "w_windmill",
                "x1": 601.51,
                "x2": 717.7,
                "y1": 482.09,
                "y2": 676.0,
                "time": 1700502378.9,
                "priority": 6,
                "confidence": 0.92,
                "camera_heading": 228.0
            }
            },
            {
            "n_light": {
                "id": "n_light",
                "x1": 1250.23,
                "x2": 1306.51,
                "y1": 359.63,
                "y2": 637.49,
                "time": 1700502384.6,
                "priority": 10,
                "confidence": 0.98,
                "camera_heading": -51.0
            }
            }
        ]

        lidar_map = None

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-20)
        self.assertLessEqual(x,50)

        self.assertGreaterEqual(y,20)
        self.assertLessEqual(y,100)

        self.assertGreaterEqual(heading,90)
        self.assertLessEqual(heading,120)
        self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")        



    def Xtest_heading_e (self):
        curr_map = self.get_map()

        # original coord pretty good, heading is bad
        # x -84.2,y: 81.96,
        # heading: 294.64
        # actual heading somewhere betwee 45 and 90, didn't measure, but looked like probably about 70
        #entry_num: 265

        located_objects = [
            {
            "ne_light": {
                "id": "ne_light",
                "x1": 467.11,
                "x2": 539.47,
                "y1": 262.77,
                "y2": 452.27,
                "time": 1700489607.4,
                "priority": 9,
                "confidence": 0.62,
                "camera_heading": 99.0
            }
            },
            {
            "w_windmill": {
                "id": "w_windmill",
                "x1": 1251.85,
                "x2": 1399.21,
                "y1": 563.93,
                "y2": 860.93,
                "time": 1700489643.7,
                "priority": 6,
                "confidence": 0.74,
                "camera_heading": 228.0
            }
            },
            {
            "sw_light": {
                "id": "sw_light",
                "x1": 622.43,
                "x2": 703.12,
                "y1": 144.17,
                "y2": 382.35,
                "time": 1700489643.7,
                "priority": 8,
                "confidence": 1.0,
                "camera_heading": 228.0
            }
            }
        ]
        lidar_map = None

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-100)
        self.assertLessEqual(x,-50)

        self.assertGreaterEqual(y,50)
        self.assertLessEqual(y,100)

        self.assertGreaterEqual(heading,55)
        self.assertLessEqual(heading,85)
        self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")        
        
    def Xtest_heading_w (self):
        curr_map = self.get_map()

        # entry 254 from Observer/LidarFix session, incorrectly reported as
        # 8.83, 28.23 - 85 degrees
        # actual: 0, 50, -90 degrees        
        located_objects =  [
            {
                "w_windmill": {
                    "id": "w_windmill",
                    "x1": 1683.48,
                    "x2": 1806.0,
                    "y1": 616.79,
                    "y2": 834.65,
                    "time": 1700228901.9,
                    "priority": 6,
                    "confidence": 0.82,
                    "camera_heading": 56.0
                }
                },
                {
                "sw_light": {
                    "id": "sw_light",
                    "x1": 623.52,
                    "x2": 683.56,
                    "y1": 248.99,
                    "y2": 475.62,
                    "time": 1700228903.2,
                    "priority": 8,
                    "confidence": 0.98,
                    "camera_heading": 56.0
                }
                },
                {
                "n_light": {
                    "id": "n_light",
                    "x1": 115.08,
                    "x2": 200.0,
                    "y1": 169.55,
                    "y2": 459.46,
                    "time": 1700228936.5,
                    "priority": 10,
                    "confidence": 1.0,
                    "camera_heading": 179.0
                }
            }
        ]

        # Add some lidar measurements
        #lidar_map = self.__generate_lidar({
        #    46.5:6184.9, # cat tree
        #    49.5:6184.9, # cat tree, manually found angle
        #    314.25:5842.0, # corrected lidar measurement
        #    309.25:5842.0 # manually adjusted angle, based on visual
        #})
        lidar_map = None

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-100)
        self.assertLessEqual(x,-50)

        self.assertGreaterEqual(y,50)
        self.assertLessEqual(y,100)

        self.assertGreaterEqual(heading,30)
        self.assertLessEqual(heading,90)
        self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")

    def Xtest_heading_w_45 (self):
        curr_map = self.get_map()

        # entry 255 from Observer/LidarFix session, incorrectly reported as
        # 7.7, 29 - -48.24 degrees
        # actual: 0, 50, -45 degrees        
        located_objects =  [
            {
                "n_light": {
                    "id": "n_light",
                    "x1": 740.29,
                    "x2": 807.69,
                    "y1": 200.21,
                    "y2": 473.99,
                    "time": 1700229219.4,
                    "priority": 10,
                    "confidence": 1.0,
                    "camera_heading": 122.0
                }
                },
                {
                "ne_light": {
                    "id": "ne_light",
                    "x1": 957.85,
                    "x2": 1019.82,
                    "y1": 259.16,
                    "y2": 489.92,
                    "time": 1700229256.7,
                    "priority": 9,
                    "confidence": 0.93,
                    "camera_heading": 179.0
                }
                },
                {
                "sw_light": {
                    "id": "sw_light",
                    "x1": 1044.83,
                    "x2": 1092.67,
                    "y1": 294.97,
                    "y2": 519.19,
                    "time": 1700229260.9,
                    "priority": 8,
                    "confidence": 0.81,
                    "camera_heading": 0.0
                }
            }
        ]

        # Add some lidar measurements
        #lidar_map = self.__generate_lidar({
        #    46.5:6184.9, # cat tree
        #    49.5:6184.9, # cat tree, manually found angle
        #    314.25:5842.0, # corrected lidar measurement
        #    309.25:5842.0 # manually adjusted angle, based on visual
        #})
        lidar_map = None

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-15)
        self.assertLessEqual(x,15)

        self.assertGreaterEqual(y,10)
        self.assertLessEqual(y,80)

        self.assertGreaterEqual(heading,-60)
        self.assertLessEqual(heading,-30)
        self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")

    def Xtest_heading_N (self):
        curr_map = self.get_map()

        # entry 250 from Observer/LidarFix session, incorrectly reported as
        # -0.58, 92.8 : heading: 2.25
        # actual: 0, 50, 0 degrees        
        located_objects =  [
            {
                "se_light": {
                    "id": "se_light",
                    "x1": 767.02,
                    "x2": 845.9,
                    "y1": 287.98,
                    "y2": 540.24,
                    "time": 1700227185.2,
                    "priority": 10,
                    "confidence": 0.99,
                    "camera_heading": 228.0
                }
                },
                {
                "se_light": {
                    "id": "se_light",
                    "x1": 767.02,
                    "x2": 845.9,
                    "y1": 287.98,
                    "y2": 540.24,
                    "time": 1700227185.2,
                    "priority": 10,
                    "confidence": 0.99,
                    "camera_heading": 228.0
                }
                },
                {
                "ne_light": {
                    "id": "ne_light",
                    "x1": 1523.23,
                    "x2": 1356.05,
                    "y1": 239.06,
                    "y2": 479.28,
                    "time": 1700227128.3,
                    "priority": 9,
                    "confidence": 0.94,
                    "camera_heading": 122.0
                }
            }
        ]

        # Add some lidar measurements
        #lidar_map = self.__generate_lidar({
        #    46.5:6184.9, # cat tree
        #    49.5:6184.9, # cat tree, manually found angle
        #    314.25:5842.0, # corrected lidar measurement
        #    309.25:5842.0 # manually adjusted angle, based on visual
        #})
        lidar_map = None

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-15)
        self.assertLessEqual(x,15)

        self.assertGreaterEqual(y,10)
        self.assertLessEqual(y,80)

        self.assertGreaterEqual(heading,-10)
        self.assertLessEqual(heading,10)
        self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")

    def Xtest_heading_e_45 (self):
        curr_map = self.get_map()

        # entry 251 from Observer/LidarFix session, incorrectly reported as
        # 1.42, 26.26, heading: 48.75
        # actual: 0, 50, 45 degrees        
        located_objects =  [
            {
                "ne_light": {
                    "id": "ne_light",
                    "x1": 109.62,
                    "x2": 200.11,
                    "y1": 269.99,
                    "y2": 512.98,
                    "time": 1700227501.2,
                    "priority": 9,
                    "confidence": 0.96,
                    "camera_heading": 99.0
                }
                },
                {
                "se_light": {
                    "id": "se_light",
                    "x1": 793.96,
                    "x2": 870.37,
                    "y1": 263.88,
                    "y2": 519.4,
                    "time": 1700227519.9,
                    "priority": 10,
                    "confidence": 0.99,
                    "camera_heading": 179.0
                }
                },
                {
                "nw_tree": {
                    "id": "nw_tree",
                    "x1": 360.22,
                    "x2": 563.6,
                    "y1": 450.17,
                    "y2": 797.71,
                    "time": 1700227522.9,
                    "priority": 7,
                    "confidence": 0.71,
                    "camera_heading": 0.0
                }
            }
        ]

        # Add some lidar measurements
        #lidar_map = self.__generate_lidar({
        #    46.5:6184.9, # cat tree
        #    49.5:6184.9, # cat tree, manually found angle
        #    314.25:5842.0, # corrected lidar measurement
        #    309.25:5842.0 # manually adjusted angle, based on visual
        #})
        lidar_map = None

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-15)
        self.assertLessEqual(x,15)

        self.assertGreaterEqual(y,10)
        self.assertLessEqual(y,80)

        self.assertGreaterEqual(heading,35)
        self.assertLessEqual(heading,55)
        self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")

    def Xtest_heading_e (self):
        curr_map = self.get_map()

        # entry 251 from Observer/LidarFix session, incorrectly reported as
        # -0.87, 40.19 ... 86.68 degrees
        # actual: 0, 50, 90 degrees        
        located_objects =  [
            {
            "se_light": {
                "id": "se_light",
                "x1": 1585.48,
                "x2": 1661.56,
                "y1": 244.24,
                "y2": 506.42,
                "time": 1700227599.2,
                "priority": 10,
                "confidence": 0.84,
                "camera_heading": 122.0
            }
            },
            {
            "ne_light": {
                "id": "ne_light",
                "x1": 362.14,
                "x2": 664.75,
                "y1": 397.98,
                "y2": 629.91,
                "time": 1700227603.5,
                "priority": 9,
                "confidence": 0.98,
                "camera_heading": 56.0
            }
            },
            {
            "n_light": {
                "id": "n_light",
                "x1": 245.54,
                "x2": 314.38,
                "y1": 332.77,
                "y2": 615.08,
                "time": 1700227641.1,
                "priority": 10,
                "confidence": 1.0,
                "camera_heading": 0.0
            }
            }            
        ]

        # Add some lidar measurements
        #lidar_map = self.__generate_lidar({
        #    46.5:6184.9, # cat tree
        #    49.5:6184.9, # cat tree, manually found angle
        #    314.25:5842.0, # corrected lidar measurement
        #    309.25:5842.0 # manually adjusted angle, based on visual
        #})
        lidar_map = None

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-15)
        self.assertLessEqual(x,15)

        self.assertGreaterEqual(y,10)
        self.assertLessEqual(y,80)

        self.assertGreaterEqual(heading,80)
        self.assertLessEqual(heading,100)
        self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")

    def Xtest_heading_n_test_w (self):
        curr_map = self.get_map()

        # entry 262 from Observer/LidarFix session, incorrectly reported as
        # 25.11, 43.82  15.27 deg
        # actual: 100, 50ish, 15ish degrees        
        located_objects =  [
            {
            "n_light": {
                "id": "n_light",
                "x1": 1206.92,
                "x2": 1264.82,
                "y1": 348.92,
                "y2": 572.93,
                "time": 1700248936.2,
                "priority": 10,
                "confidence": 0.95,
                "camera_heading": 56.0
            }
            },
            {
            "ne_light": {
                "id": "ne_light",
                "x1": 1256.89,
                "x2": 1056.74,
                "y1": 167.73,
                "y2": 473.19,
                "time": 1700248950.7,
                "priority": 9,
                "confidence": 0.99,
                "camera_heading": 99.0
            }
            }         
        ]

        # Add some lidar measurements
        #lidar_map = self.__generate_lidar({
        #    46.5:6184.9, # cat tree
        #    49.5:6184.9, # cat tree, manually found angle
        #    314.25:5842.0, # corrected lidar measurement
        #    309.25:5842.0 # manually adjusted angle, based on visual
        #})
        lidar_map = None

        # get visual degrees for each point
        estimator = self.__get_estimator (curr_map)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 19.0, lidar_map=lidar_map)

        #logging.getLogger(__name__).info(f"Basis: {json.dumps(basis, cls=NavJsonEncoder, indent=2)}")
        logging.getLogger(__name__).info(f"X:{x}, Y:{y}, Heading: {heading}, Confidence: {confidence}")

        self.assertGreaterEqual(x,-15)
        self.assertLessEqual(x,15)

        self.assertGreaterEqual(y,10)
        self.assertLessEqual(y,80)

        self.assertGreaterEqual(heading,80)
        self.assertLessEqual(heading,100)
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
