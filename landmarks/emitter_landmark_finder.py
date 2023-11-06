from camera.image_resolution import ImageResolution
from field.field_map import FieldMap
from landmarks.emitter_group import EmitterGroup, EmitterGroupPattern
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


    def extract_landmarks_from_locations (self, object_locations, id_filter = ['light'], confidence_threshold = None):
        emitter_locations = []
        #logging.getLogger(__name__).info(f"Full object locations: {object_locations}")
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
    def __extract_landmarks (self, emitter_locations, min_vert_lights = 3):
        groups = self.__extract_vertical_lines (emitter_locations)
        known_groups = []
        
        squares, matched_emitter_keys = self.__extract_squares_search(groups)
        
        # now once again, find all vertically aligned sets (and loners) that were not previously matched as a square
        remaining_emitter_locations = []
        for e in emitter_locations:
            if e.get_key() not in matched_emitter_keys:
                remaining_emitter_locations.append(e)
        remaining_groups = self.__extract_vertical_lines(remaining_emitter_locations)

        # search for triangles
        triangles, matched_emitter_keys = self.__extract_triangles_search (remaining_groups, remaining_emitter_locations)
        last_emitter_locations = []
        for e in remaining_emitter_locations:
            if e.get_key() not in matched_emitter_keys:
                last_emitter_locations.append(e)
        last_groups = self.__extract_vertical_lines(last_emitter_locations)

        # finally, search for vertical lines from the leftovers
        vertical_lines = []
        for g in last_groups:
            if len(g.get_emitters()) >= min_vert_lights:
                vertical_lines.append(g)
        
        for g in vertical_lines + triangles + squares:
            group_id = self.get_emitter_group_identity(g)
            if group_id is None:
                logging.getLogger(__name__).debug(f'found unmarked group, pattern: {g.get_pattern()} of size: {len(g.get_emitters())} centered at: {g.get_group_center()}, height: {g.get_height()}')
            else:
                g.set_identity(group_id)
                known_groups.append(g)
                all_conf = []
                for e in g.get_emitters():
                    all_conf.append(e.confidence)
                g.set_confidence(statistics.mean(all_conf))

                logging.getLogger(__name__).debug(f'Found {group_id} - size: {len(g.get_emitters())} centered at: {g.get_group_center()}, height: {g.get_height()}')
                
        return known_groups

    def __extract_vertical_lines (self, emitter_locations):
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
        return groups

    def __get_group_key (self, emitter_group):
        center_x, center_y = emitter_group.get_group_center()
        return f"{center_x},{center_y}"

    def __get_potential_pairs (self, vertical_set, set_filter):
        pairs = []
        set_key = self.__get_group_key(vertical_set)
        if set_key not in set_filter and len(vertical_set.get_emitters()) > 1:
            # add each neighboring pair within the set as a potential pair
            for i, emitter in enumerate(vertical_set.get_emitters()):
                if i < len(vertical_set.get_emitters()) - 1:
                    trial_group = EmitterGroup()
                    trial_group.add_emitter(emitter)
                    trial_group.add_emitter(vertical_set.get_emitters()[i+1])
                    pairs.append(trial_group)
        
        return pairs

    def __extract_squares_search (self, vertical_sets, vertical_leeway = 0.05, max_height_diff_pct = 0.05, max_horz_dist_pct = 0.2):
        squares = []
        matched_groups = []
        matched_emitter_keys = []
        for s1 in vertical_sets:
            s1_pairs = self.__get_potential_pairs(s1, matched_groups)
            this_key = self.__get_group_key(s1)
            
            for s1_pair in s1_pairs:
                # Loop over all other unmatched vertical sets
                for s2 in vertical_sets:
                    if this_key not in matched_groups:
                        s2_pairs = self.__get_potential_pairs(s2, matched_groups + [this_key,]) # filter already matched groups, plus the first comparison set
                        for s2_pair in s2_pairs:
                            if self.__are_vertically_aligned (s1_pair, s2_pair, vertical_leeway, max_height_diff_pct, max_horz_dist_pct):
                                matched_groups.append(this_key)
                                matched_groups.append(self.__get_group_key(s2))
                                
                                # Add the p2 line to p1, and change its type to square
                                s1_pair.set_pattern(EmitterGroupPattern.SQUARE)
                                for e in s2_pair.get_emitters():
                                    s1_pair.add_emitter(e)
                                self.__set_emitters_as_matched(s1_pair, matched_emitter_keys)
                                squares.append(s1_pair)
                                break
        return squares, matched_emitter_keys

    def __extract_triangles_search (self, vertical_sets, all_emitters, vertical_leeway = 0.05, max_horz_dist_pct = 0.2, min_width_pct = 0.04):
        triangles = []
        matched_groups = []
        matched_emitter_keys = []
        for s1 in vertical_sets:
            s1_pairs = self.__get_potential_pairs(s1, matched_groups)
            this_key = self.__get_group_key(s1)
            
            for s1_pair in s1_pairs:
                e1_key = s1_pair.get_emitters()[0].get_key()
                e2_key = s1_pair.get_emitters()[1].get_key()
                if e1_key not in matched_emitter_keys and e2_key not in matched_emitter_keys:
                    # Loop over all other unmatched emitters
                    for e in all_emitters:
                        this_e_key = e.get_key()
                        if this_e_key not in matched_emitter_keys:
                            # check for traingle-type match between the pair and this loner
                            if self.__are_group_and_emitter_vertically_aligned (s1_pair, e, vertical_leeway, max_horz_dist_pct):
                                if self.__is_triangle_wide_enough (s1_pair, e, min_width_pct):
                                    matched_groups.append(this_key)
                                        
                                    # Add the loner point to the group, and change its type to triangle
                                    s1_pair.set_pattern(EmitterGroupPattern.SIDEWAYS_TRIANGLE)
                                    s1_pair.add_emitter(e)
                                    self.__set_emitters_as_matched(s1_pair, matched_emitter_keys)
                                    triangles.append(s1_pair)
                                    break
        return triangles, matched_emitter_keys


    def __set_emitters_as_matched (self, group, matched_emitter_keys):
        for e in group.get_emitters():
            matched_emitter_keys.append(e.get_key())

    def __is_triangle_wide_enough (self, group, emitter, min_width_pct):
        g_center_x, g_center_y = group.get_group_center()
        e_center_x, e_center_y = emitter.get_center()
        
        if (abs(g_center_x - e_center_x) / self.get_image_resolution().get_width()) >= min_width_pct:
            return True
        return False
 
    def __are_group_and_emitter_vertically_aligned (self, group, emitter, vertical_leeway, max_horz_dist_pct):
        g_center_x, g_center_y = group.get_group_center()
        e_center_x, e_center_y = emitter.get_center()
        
        if (abs(g_center_y - e_center_y) / self.get_image_resolution().get_height()) <= vertical_leeway:
            if (abs(g_center_x - e_center_x) / self.get_image_resolution().get_width()) <= max_horz_dist_pct:
                return True
        return False
        
    def __are_vertically_aligned (self, group_1, group_2, vertical_leeway, max_height_diff_pct, max_horz_dist_pct):
        g1_center_x, g1_center_y = group_1.get_group_center()
        g2_center_x, g2_center_y = group_2.get_group_center()
        
        if (abs(g1_center_y - g2_center_y) / self.get_image_resolution().get_height()) <= vertical_leeway:
            if (abs(group_1.get_height() - group_2.get_height()) / self.get_image_resolution().get_height()) <= max_height_diff_pct:
                if (abs(g1_center_x - g2_center_x) / self.get_image_resolution().get_width()) <= max_horz_dist_pct:
                    return True
        return False


    def get_emitter_group_identity (self, emitter_group):
        cached_id = emitter_group.get_identity()
        if cached_id is None:
            pattern = str(len(emitter_group.get_emitters()))
            if emitter_group.get_pattern() == EmitterGroupPattern.SQUARE:
                pattern = 'square'
            elif emitter_group.get_pattern() == EmitterGroupPattern.SIDEWAYS_TRIANGLE:
                pattern = 'sideways_triangle'
                
            cached_id = self.get_field_map().get_landmark_by_model_type_pattern (
                self.__emitter_model,
                self.__landmark_type,
                landmark_pattern = pattern)
            emitter_group.set_identity(cached_id)
        return cached_id

    def __get_distance (self, x1, y1, x2, y2):
        return math.sqrt(((x2 - x1)**2) + ((y2 - y1) **2))
