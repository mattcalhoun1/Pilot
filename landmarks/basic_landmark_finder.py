from camera.image_resolution import ImageResolution
from field.field_map import FieldMap
from landmarks.landmark_finder import LandmarkFinder
import logging
import statistics
import time

class BasicLandmarkFinder (LandmarkFinder) :
    def __init__ (self, camera_config, field_map : FieldMap, model_name : str, default_object_id_filter : list = None):
        LandmarkFinder.__init__(self, camera_config=camera_config, field_map=field_map, default_object_id_filter=default_object_id_filter)
        self.__model_name = model_name
        self.__default_confidence_threshold = 0.25


    def extract_landmarks_from_locations (self, object_locations, id_filter = None, confidence_threshold = None):
        landmark_locations = []
        for obj in object_locations:
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

                landmark_id = self.get_field_map().get_landmark_by_model_type_pattern (
                    self.__model_name,
                    obj['object'],
                    landmark_pattern = None)
                if landmark_id is not None:
                    confidence_threshold = confidence_threshold if confidence_threshold is not None else self.get_field_map().get_landmark_confidence_threshold(landmark_id)

                    if obj['confidence'] >= confidence_threshold:
                        landmark_locations.append({
                            landmark_id : {
                            'id':landmark_id,
                            'x1':x1,
                            'y1':y1,
                            'x2':x2,
                            'y2':y2,
                            'confidence':obj['confidence'],
                            'time':time.time()
                            }
                        })
                else:
                    logging.getLogger(__name__).debug(f"Object type : {obj['object']} is not on map as a landmark.")

        return landmark_locations
