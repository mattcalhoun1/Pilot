import unittest
from field.field_map import FieldMap
from landmarks.emitter_landmark_finder import EmitterLandmarkFinder
from camera.camera_info import CameraInfo
from camera.image_resolution import ImageResolution
import logging
import statistics

class TestEmitterLandmarkFinder(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()
        
    def testSquares (self):
        finder = self.__get_finder()
        located_objects = self.__located_objects_with_square()

        landmarks = finder.extract_landmarks_from_locations (located_objects)
        
        logging.getLogger(__name__).info(f"Landmarks (should contain a square): {landmarks}")

        self.assertEqual(len(landmarks), 3)
        

    def __get_finder (self):
        camera_config = CameraInfo.CameraConfig['PI2_STD_HQ_CROPPED_0']

        field_map = FieldMap({
                'n_square': {
                    'pattern':'square',
                    'type':'light',
                    'model':'lights',
                    'x':26,
                    'y':132,
                    'height':16.0,
                    'center_altitude':8,
                    'confidence':0.3
                },
                'n_triangle': {
                    'pattern':'sideways_triangle_left',
                    'type':'light',
                    'model':'lights',
                    'x':26,
                    'y':132,
                    'height':16.0,
                    'center_altitude':8,
                    'confidence':0.3
                },
                'n_vert': {
                    'pattern':'3',
                    'type':'light',
                    'model':'lights',
                    'x':25,
                    'y':51,
                    'height':11.5,
                    #'extension_length':25,
                    'center_altitude':12.5,
                    'confidence':0.3
                },
                # -35, -12 
            })
        return EmitterLandmarkFinder(
            camera_config=camera_config,
            field_map=field_map
        )     
                
    def __located_objects_with_square (self):
        return [
          {
            "object": "light",
            "x_center": 513.7218475341797,
            "y_center": 1008.6759209632874,
            "x_min": 479.0684509277344,
            "x_max": 548.375244140625,
            "y_min": 969.0720963478088,
            "y_max": 1048.2797455787659,
            "confidence": 0.79400504
          },
          {
            "object": "light",
            "x_center": 846.2755966186523,
            "y_center": 780.8970236778259,
            "x_min": 818.3881187438965,
            "x_max": 874.1630744934082,
            "y_min": 753.5955262184143,
            "y_max": 808.1985211372375,
            "confidence": 0.64644575
          },
          {
            "object": "light",
            "x_center": 1030.5590057373047,
            "y_center": 591.3811898231506,
            "x_min": 1003.3314514160156,
            "x_max": 1057.7865600585938,
            "y_min": 562.6425004005432,
            "y_max": 620.119879245758,
            "confidence": 0.5292161
          },
          {
            "object": "light",
            "x_center": 1031.5590057373047,
            "y_center": 781.3811898231506,
            "x_min": 1007.3314514160156,
            "x_max": 1055.7865600585938,
            "y_min": 756.6425004005432,
            "y_max": 804.119879245758,
            "confidence": 0.5292161
          },
          {
            "object": "light",
            "x_center": 843.7174415588379,
            "y_center": 589.2298436164856,
            "x_min": 819.3626976013184,
            "x_max": 868.0721855163574,
            "y_min": 567.4789094924927,
            "y_max": 610.9807777404785,
            "confidence": 0.33717632
          },
          {
            "object": "light",
            "x_center": 530.5590057373047,
            "y_center": 391.3811898231506,
            "x_min": 503.3314514160156,
            "x_max": 557.7865600585938,
            "y_min": 362.6425004005432,
            "y_max": 420.119879245758,
            "confidence": 0.5292161
          },
          {
            "object": "light",
            "x_center": 531.5590057373047,
            "y_center": 581.3811898231506,
            "x_min": 507.3314514160156,
            "x_max": 555.7865600585938,
            "y_min": 556.6425004005432,
            "y_max": 604.119879245758,
            "confidence": 0.5292161
          },
          {
            "object": "light",
            "x_center": 421.5590057373047,
            "y_center": 483.3811898231506,
            "x_min": 400.3314514160156,
            "x_max": 440.7865600585938,
            "y_min": 460.6425004005432,
            "y_max": 500.119879245758,
            "confidence": 0.5292161
          },
          
          {
            "object": "light",
            "x_center": 1246.2755966186523,
            "y_center": 780.8970236778259,
            "x_min": 1218.3881187438965,
            "x_max": 1274.1630744934082,
            "y_min": 753.5955262184143,
            "y_max": 808.1985211372375,
            "confidence": 0.64644575
          },
          {
            "object": "light",
            "x_center": 1246.2755966186523,
            "y_center": 880.8970236778259,
            "x_min": 1218.3881187438965,
            "x_max": 1274.1630744934082,
            "y_min": 853.5955262184143,
            "y_max": 908.1985211372375,
            "confidence": 0.64644575
          },
          {
            "object": "light",
            "x_center": 1246.2755966186523,
            "y_center": 980.8970236778259,
            "x_min": 1218.3881187438965,
            "x_max": 1274.1630744934082,
            "y_min": 853.5955262184143,
            "y_max": 1008.1985211372375,
            "confidence": 0.64644575
          },
          
        ]
