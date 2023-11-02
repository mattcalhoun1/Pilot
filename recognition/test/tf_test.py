import unittest
from position.position_estimator import Confidence
from recognition.test.mock_camera import MockCamera
from recognition.tf_object_locator import TFObjectLocator
import logging
import numpy as np
import json

class TestTFObjectLocator(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def __get_right_camera (self):
        return MockCamera("recognition/test/resources/images/coordinates_cam_Right.png")

    def __get_left_camera (self):
        return MockCamera("recognition/test/resources/images/coordinates_cam_Left.png")

    def __get_model_configs (self):
        model_configs = {}
        with open("recognition/test/resources/tf_models_just_basement.json") as cfg_in:
            model_configs = json.load(cfg_in)
        
        return model_configs

    def XtestLocateTree (self):
        locator =  TFObjectLocator(
            model_configs = self.__get_model_configs(), 
            keep_latest_image = True
        )
        
        logging.getLogger(__name__).info("models loaded")
        
        # locate some objects in the left camera
        located_objects = locator.find_objects_on_camera(camera = self.__get_left_camera(), object_filter = None, min_confidence = 0.3)
        
        logging.getLogger(__name__).info(f"Located {located_objects}")
