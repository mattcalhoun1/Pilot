from camera.image_resolution import ImageResolution
from field.field_map import FieldMap
from landmarks.landmark_finder import LandmarkFinder
import logging
import statistics
import time

class ObjectSearchFinder (LandmarkFinder) :
    def __init__ (self, camera_config, field_map : FieldMap, model_name : str, default_object_id_filter : list = None):
        LandmarkFinder.__init__(self, camera_config=camera_config, field_map=field_map, default_object_id_filter=default_object_id_filter)
        self.__model_name = model_name
        self.__default_confidence_threshold = 0.25


    def extract_landmarks_from_locations (self, object_locations, id_filter = None, confidence_threshold = None):
        if confidence_threshold is None:
            confidence_threshold = self.__default_confidence_threshold

        landmark_locations = []
        for i, obj in enumerate(object_locations):
            if id_filter is None or obj['object'] in id_filter:
                x1 = obj['x_min']
                x2 = obj['x_max']
                y1 = obj['y_min']
                y2 = obj['y_max']

                # if y1/y2 are flipped (weird pixel order), reverse them
                if y1 > y2:
                    tmp = y2
                    y2 = y1
                    y1 = tmp
                if x1 > x2:
                    tmp = x2
                    x2 = x1
                    x1 = tmp

                landmark_id = f"{obj['object']}.{i}"

                if obj['confidence'] >= confidence_threshold:
                    distorted_height = abs(y2-y1)
                    estimated_center_x = statistics.mean([x1, x2])
                    corrected_height = distorted_height - (distorted_height * self.get_height_distortion_multiplier_at_x(estimated_center_x))

                    landmark_locations.append({
                        landmark_id : {
                        'id':landmark_id,
                        'x1':x1,
                        'y1':y1,
                        'x2':x2,
                        'y2':y2,
                        'confidence':obj['confidence'],
                        'time':time.time(),
                        'corrected_height':corrected_height
                        }
                    })

        return landmark_locations