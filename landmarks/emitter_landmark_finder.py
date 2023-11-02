from camera.image_resolution import ImageResolution
from field.field_map import FieldMap
from landmarks.emitter_group import EmitterGroup
from landmarks.emitter_viewed_location import EmitterViewedLocation
from landmarks.landmark_finder import LandmarkFinder
import logging
import statistics
import time
import math

class EmitterLandmarkFinder (LandmarkFinder) :
    def __init__ (self, camera_config, field_map : FieldMap):
        LandmarkFinder.__init__(self, camera_config=camera_config, field_map=field_map)
        self.__emitter_model = 'lights'
        self.__landmark_type = 'light'
        self.__default_confidence_threshold = 0.25
        #self.__camera_config = camera_config
        #self.__field_layout = field_layout
        #self.__image_resolution = image_resolution

    #def get_landmark_config (self, landmark_id):
    #    return self.__field_layout.get_landmark_position(landmark_id)

    #def get_image_resolution (self):
    #    return self.__image_resolution

    def extract_landmarks_from_locations (self, object_locations, id_filter = ['light'], confidence_threshold = None):
        emitter_locations = []
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

                emitter_locations.append(EmitterViewedLocation(x1=x1, y1=y1, x2=x2, y2=y2, confidence=obj['confidence']))
        group_locations = self.__extract_landmarks(emitter_locations)
        landmark_locations = []
        
        for g in group_locations:
            top_center_x, top_center_y = g.get_top().get_center()
            bottom_center_x, bottom_center_y = g.get_bottom().get_center()

            # bump the x's apart by the group's width
            top_left_x = top_center_x - (int)(g.get_top().get_width() / 2)
            bottom_right_x = bottom_center_x + (int)(g.get_bottom().get_width() / 2)
            
            # make sure this group of emitters is a known group.
            # otherwise, we will ignore it. it may just be a stray light
            group_id = self.get_emitter_group_identity(g)
            if group_id is not None:
                confidence_threshold = confidence_threshold if confidence_threshold is not None else self.get_field_map().get_landmark_confidence_threshold(group_id)
                if obj['confidence'] >= confidence_threshold:
                    landmark_locations.append({
                        group_id : {
                        'id':group_id,
                        'x1':top_left_x,
                        'y1':top_center_y,
                        'x2':bottom_right_x,
                        'y2':bottom_center_y,
                        'confidence':g.get_confidence(),
                        'time':time.time()
                        }
                    })
        
        return landmark_locations
    
    # for all identified objects, extracts emitter groups
    def __extract_landmarks (self, emitter_locations):
        # percent of the side to side movement a point is allowed, to be
        # considered part ofthe same emitter
        horizontal_leeway = .05 * self.get_image_resolution().get_width()
        groups = []

        for loc in emitter_locations:
            center_x, center_y = loc.get_center()
            
            # see if this emitter location has been used already

            matching_group = None
            for g in groups:
                g_center_x, g_center_y = g.get_group_center()
                if abs(g_center_x - center_x) <= horizontal_leeway:
                    matching_group = g
                    break
            
            if matching_group is None:
                matching_group = EmitterGroup()
                groups.append(matching_group)
            
            matching_group.add_emitter(loc)

        # calculate confidence
        for g in groups:
            all_conf = []
            for e in g.get_emitters():
                all_conf.append(e.confidence)
            g.set_confidence(statistics.mean(all_conf))

        known_groups = []
        for g in groups:
            group_id = self.get_emitter_group_identity(g)
            if group_id is None:
                logging.getLogger(__name__).debug(f'found group of size: {len(g.get_emitters())} centered at: {g.get_group_center()}, height: {g.get_height()}')
            else:
                g.set_identity(group_id)
                known_groups.append(g)
                logging.getLogger(__name__).debug(f'Found {group_id} - size: {len(g.get_emitters())} centered at: {g.get_group_center()}, height: {g.get_height()}')

        return known_groups

    def get_emitter_group_identity (self, emitter_group):
        cached_id = emitter_group.get_identity()
        if cached_id is None:
            cached_id = self.get_field_map().get_landmark_by_model_type_pattern (
                self.__emitter_model,
                self.__landmark_type,
                landmark_pattern = str(len(emitter_group.get_emitters())))
            emitter_group.set_identity(cached_id)
        return cached_id

    def __get_distance (self, x1, y1, x2, y2):
        return math.sqrt(((x2 - x1)**2) + ((y2 - y1) **2))
