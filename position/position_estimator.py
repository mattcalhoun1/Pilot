from field.field_map import FieldMap
from visual.visual_distance import VisualDistanceCalculator
from visual.visual_degrees import VisualDegreesCalculator
from trig.trig import BasicTrigCalc
from trig.genetic_length_finder import BaseTopLengthFinder
from position.position_thread_manager import PositionThreadManager
from position.confidence import Confidence
from position.estimator_mode import EstimatorMode
import math
import statistics
import logging

class PositionEstimator:
    def __init__(self, field_map : FieldMap, horizontal_fov, vertical_fov, view_width, view_height, base_front=90.0, use_multithreading=True, estimator_mode = EstimatorMode.VERY_PRECISE, max_lidar_drift_deg = 1.5, max_lidar_visual_variance_pct = 0.33):
        self.__field_map = field_map
        self.__visual_dist_calc = VisualDistanceCalculator(horizontal_fov = horizontal_fov, vertical_fov = vertical_fov, view_width=view_width, view_height=view_height)
        self.__visual_degrees_calc = VisualDegreesCalculator(horizontal_fov = horizontal_fov, vertical_fov = vertical_fov, view_width=view_width, view_height=view_height)
        self.__horizontal_fov = horizontal_fov
        self.__vertical_fov = vertical_fov
        self.__view_width = view_width
        self.__view_height = view_height
        self.__base_front = base_front
        self.__use_multithreading = use_multithreading
        self.__estimator_mode = estimator_mode
        self.__max_lidar_drift = max_lidar_drift_deg
        self.__max_lidar_visual_variance = max_lidar_visual_variance_pct

        self.__log_configuration()

    def __log_configuration (self):
        logging.getLogger(__name__).info(f"== Position Estimator configuration ==")
        printable_settings = {
            'Field Map': self.__field_map.get_name(), 
            'Horizontal FOV': self.__horizontal_fov,
            'Vertical FOV': self.__vertical_fov,
            'View Size':f'{self.__view_width} x {self.__view_height}',
            'Multithreading':self.__use_multithreading,
            'Estimator Mode':self.__estimator_mode
        }
        for k in printable_settings:
            logging.getLogger(__name__).info(f"{k:>15}: {printable_settings[k]}")

    def extract_object_view_angles (self, located_objects, add_relative_angles = True):
        angles = {}

        # located objects expected like:
        #[
        #    {'n1':{'x1':1162.5, 'y1':321.76, 'x2':1209.53, 'y2':516.0, 'confidence':0.4}},
        #    {'n2':{'x1':308.05, 'y1':135.06, 'x2':352.7, 'y2':545.3, 'confidence':0.4}},
        #]        

        # go through the objects located, and get distance to each
        for located_group in located_objects:
            for landmark_id in located_group:
                obj_pix = located_group[landmark_id]
                height_pixels = abs(obj_pix['y2'] - obj_pix['y1'])
                width_pixels = abs(obj_pix['x2'] - obj_pix['x1'])
                height_degrees = self.__visual_degrees_calc.caclulate_vertical_visual_degrees_given_height_pixels(height_pixels=height_pixels)
                width_degrees = self.__visual_degrees_calc.caclulate_horizontal_visual_degrees_given_width_pixels(width_pixels=width_pixels)

                center_x=obj_pix['x1'] + ((obj_pix['x2'] - obj_pix['x1'])/2)
                center_y=obj_pix['y1'] + ((obj_pix['y2'] - obj_pix['y1'])/2)


                vehicle_rel_camera_heading = obj_pix['camera_heading'] - self.__base_front
                img_pixels_off_center = center_x - (self.__view_width / 2.0)
                image_relative_deg = self.__visual_degrees_calc.caclulate_horizontal_visual_degrees_given_width_pixels(img_pixels_off_center)

                if landmark_id not in angles:
                    angles[landmark_id] = []
                this_angle_set = angles[landmark_id]
                this_angle_set.append({
                    'center_x':center_x,
                    'center_y':center_y,
                    'height_pix':height_pixels,
                    'height_deg':height_degrees,
                    'width_pix':width_pixels,
                    'width_deg':width_degrees,
                    'confidence':obj_pix['confidence'],
                    'image_heading':obj_pix['camera_heading'],
                    'image_rel_deg':image_relative_deg
                })

        # add the observed degrees of each marker relative to others
        if add_relative_angles:
            self.__add_relative_angles(angles=angles)

        return angles
    
    def __add_relative_angles (self, angles):
        # for each angle, figure out a 0-360 angle difference from each other angle
        for landmark_id in angles:
            if self.__field_map.is_landmark_known(landmark_id):
                for curr_angle in angles[landmark_id]:
                    curr_angle['relative_deg'] = {}
                    curr_angle['relative_px'] = {}
                    for other_landmark_id in angles:
                        if other_landmark_id != landmark_id:
                            # Get the most accurate view we have for that landmark
                            selected_other_angle = max(angles[other_landmark_id], key=lambda x:x['confidence'])

                            # find the x distance between the two
                            this_x = curr_angle['center_x']
                            other_x = selected_other_angle['center_x']

                            # figure the adjusted x for both, if we stretch the image out to 360 degrees,
                            # assuming the center of curr obj is as zero degrees
                            # so the base object is at x pixel zero
                            heading_degree_diff = selected_other_angle['image_heading'] - curr_angle['image_heading']
                            heading_degree_diff_px = self.__visual_degrees_calc.calculate_pixels_given_horizontal_degree (heading_degree_diff)

                            distance_between_px = other_x - this_x
                            relative_x = distance_between_px # assuming this x is zero, what is the other objects pixel x related to this

                            if heading_degree_diff > 0:
                                distance_between_px = abs(heading_degree_diff_px + other_x - this_x)
                                relative_x = distance_between_px
                            elif heading_degree_diff < 0:
                                distance_between_px = this_x - heading_degree_diff_px - other_x
                                relative_x = -1 * distance_between_px

                            #logging.getLogger(__name__).info(f'{landmark_id}/{other_landmark_id} px diff: {distance_between_px}, degree diff: {distance_between_deg}')

                            curr_angle['relative_px'][other_landmark_id] = relative_x
                            curr_angle['relative_deg'][other_landmark_id] = self.__visual_degrees_calc.caclulate_horizontal_visual_degrees_given_width_pixels(curr_angle['relative_px'][other_landmark_id])

                            #logging.getLogger(__name__).info(f"{landmark_id}/{other_landmark_id}, relative pixels: {curr_angle['relative_px'][other_landmark_id]}, relative deg: {curr_angle['relative_deg'][other_landmark_id]}")

    def __get_vehicle_relative_heading (self, cam_oriented_heading):
        new_heading = cam_oriented_heading - 90
        if new_heading > 180:
            new_heading = -1 * (180 - (new_heading - 180))

        return new_heading

    def __mm_to_in (self, measurement):
        return measurement / 25.4


    def extract_distances (self, view_angles, view_altitude, filter_unmapped_objects = True, lidar_map = None):
        # loop through all the visible angles and get our distance to them
        distances = {}
        for landmark_id in view_angles:
            add_lidar = False
            selected_angle = None
            if self.__field_map.is_landmark_known(landmark_id):
                # if > 1 reported, choose the one with highest confidence
                selected_angle = max(view_angles[landmark_id], key=lambda x:x['confidence'])
                ground_distance, top_distance, bottom_distance = self.__visual_dist_calc.estimate_distances_given_viewed_height (
                            view_altitude = view_altitude, 
                            obj_height_degrees = selected_angle['height_deg'], 
                            obj_known_height = self.__field_map.get_landmark_height(landmark_id=landmark_id), 
                            obj_center_altitude = self.__field_map.get_landmark_altitude(landmark_id=landmark_id))

                distances[landmark_id] = {
                    'ground':ground_distance,
                    'top':top_distance,
                    'bottom':bottom_distance,
                    'islidar':False
                }

                # see if we should check lida for a hit on distance
                if lidar_map is not None and self.__field_map.is_landmark_lidar_visible(landmark_id):
                    add_lidar = True


            elif filter_unmapped_objects == False:
                # if > 1 reported, choose the one with highest confidence
                selected_angle = max(view_angles[landmark_id], key=lambda x:x['confidence'])
                ground_distance, top_distance, bottom_distance = self.__visual_dist_calc.estimate_distances_given_viewed_height (
                            view_altitude = view_altitude, 
                            obj_height_degrees = selected_angle['height_deg'], 
                            obj_known_height = self.__field_map.get_search_obj_height(object_type=landmark_id.split('.')[0]), 
                            obj_center_altitude = view_altitude # assume same altitude
                )

                distances[landmark_id] = {
                    'ground':ground_distance,
                    'top':top_distance,
                    'bottom':bottom_distance,
                    'islidar':False
                }

                # see if we should check lida for a hit on distance
                if lidar_map is not None and self.__field_map.is_search_object_lidar_visible(landmark_id):
                    add_lidar = True

            if add_lidar:
                obj_rel_deg = selected_angle['image_heading'] + selected_angle['image_rel_deg']
                obj_veh_relative_heading = self.__get_vehicle_relative_heading (obj_rel_deg)  

                # now convert to lidar heading, where 0 is front, 270 is left
                lidar_rel_heading = obj_veh_relative_heading if obj_veh_relative_heading > 0 else (360 + obj_veh_relative_heading)

                logging.getLogger(__name__).info(f"Checking lidar for {landmark_id} at {obj_veh_relative_heading} degrees (lidar heading: {lidar_rel_heading})")
                lidar_reading = lidar_map.get_measurement (lidar_rel_heading, max_allowed_drift = self.__max_lidar_drift)
                lidar_reading_in = self.__mm_to_in(lidar_reading)
                if lidar_reading > 0 and abs(lidar_reading_in - distances[landmark_id]['ground'])/lidar_reading_in <= self.__max_lidar_visual_variance:
                    logging.getLogger(__name__).info(f"Substituting Lidar reading of {lidar_reading_in} in place of visual estimate of {distances[landmark_id]['ground']}.")
                    distances[landmark_id]['visual_ground'] = distances[landmark_id]['ground']
                    distances[landmark_id]['lidar'] = lidar_reading_in
                    distances[landmark_id]['ground'] = lidar_reading_in
                    distances[landmark_id]['islidar'] = True


        return distances
    

    def __get_target_accuracy_and_time (self):        
        target_accuracy = 0.002
        allowed_time = 0.5

        if self.__estimator_mode == EstimatorMode.FAST:
            target_accuracy = 0.04
            allowed_time = 0.2
        elif self.__estimator_mode == EstimatorMode.VERY_PRECISE:
            target_accuracy = 0.001
        
        return target_accuracy, allowed_time

    def __get_possible_coordinates (self, viz_angle, landmark_id, other_landmark_id, distances, view_angles):
        actual_field_dist = self.__field_map.get_distance(landmark_id_1=landmark_id,landmark_id_2=other_landmark_id)
        possible = []

        # imagine a line betwee the two landmarks. we have to
        # calculate what the coordinates would be with us on either side of the line
        # because at this point we don't know which side of hte line we're on.
        # after having all possible coordinates, another pair should hopefully tell us
        # which. otherwise there are two possible coordinates, and we have
        # no way to know which, unless we have bounds around our coordinates, and one
        # of the points is out of bounds

        # the far side is always opposite the viz_angle, the viz_angle is the far angle
        for flip_calculation in [True,False]:        
            # left off here
            base_side = 0
            top_side = 0
            base_landmark_id = None
            top_landmark_id = None

            if flip_calculation:
                # base side is other landmark, top is this
                top_side = distances[landmark_id]['ground']
                top_landmark_id = landmark_id
                base_side = distances[other_landmark_id]['ground']
                base_landmark_id = other_landmark_id
                #logging.getLogger(__name__).info(f"{landmark_id}/{other_landmark_id} FLIPPED, base: {landmark_id}, top: {other_landmark_id}")
            else:
                # base side is this landmark, top is other
                top_side = distances[other_landmark_id]['ground']
                top_landmark_id = other_landmark_id
                base_side = distances[landmark_id]['ground']
                base_landmark_id = landmark_id
                #logging.getLogger(__name__).info(f"{landmark_id}/{other_landmark_id} NOT flipped, base: {other_landmark_id}, top: {landmark_id}")

            # far_angle = viz_angle (angle from my perspective)
            # so i have far angle = HIGH confidence, far dist = FACT, other 2 sides MED confidence
            #
            # need other 2 angles so I can calculate slope to find position
            # base_side = right side (relative to perspective) looking down (see readme)
            trig_calc = BasicTrigCalc()
            #logging.getLogger(__name__).info(f"{landmark_id}/{other_landmark_id} - Actual Field Dist: {actual_field_dist}, Base Side: {base_side}, Top Side: {top_side}")

            # the base and top distances are estimates. We need to adjust them until the view angle looks close
            
            # whichever is taller gets higher confidence
            base_height = max(view_angles[base_landmark_id], key=lambda x:x['confidence'])['height_deg']
            top_height = max(view_angles[top_landmark_id], key=lambda x:x['confidence'])['height_deg']

            base_confidence=Confidence.CONFIDENCE_MEDIUM
            top_confidence=Confidence.CONFIDENCE_MEDIUM

            # if lidar was substituted, raise confidence in the lidar measurement
            if distances[base_landmark_id]['islidar'] == True:
                base_confidence = Confidence.CONFIDENCE_HIGH
            if distances[top_landmark_id]['islidar'] == True:
                top_confidence = Confidence.CONFIDENCE_HIGH

            # if both are visual, lower confidence in the shorter object
            if distances[base_landmark_id]['islidar'] == False and distances[top_landmark_id]['islidar'] == False:
                if base_height > top_height:
                    top_confidence -= 0.1
                    base_confidence += 0.025
                else:
                    base_confidence -= 0.1
                    top_confidence += 0.025


            length_finder = BaseTopLengthFinder(
                far_angle=abs(viz_angle),
                far_side=actual_field_dist, 
                est_top=top_side,
                est_base=base_side,
                base_confidence=base_confidence,
                top_confidence=top_confidence
            )

            target_accuracy, allowed_time = self.__get_target_accuracy_and_time()

            possible_lengths = length_finder.find_lengths(max_num_solutions=10, target_accuracy=target_accuracy, allowed_time=allowed_time)

            for found_base_side, found_top_side, found_far_angle_diff in possible_lengths:
                #logging.getLogger(__name__).info(f"Trying out lengths - base: {found_base_side}, top: {found_top_side}")

                base_angle = trig_calc.calc_base_angle(
                    far_side=actual_field_dist, 
                    base_side=found_base_side, 
                    top_side=found_top_side)
                        
                
                top_angle = 180 - base_angle - abs(viz_angle)
                #logging.getLogger(__name__).info(f"{landmark_id}/{other_landmark_id} For calculation - Top Angle: {top_angle}, Base Angle: {base_angle}, Viz Angle: {abs(viz_angle)}")

                # now get this landmark's slope, relative to a line that goes from this landmark to the y axis
                # to do so, make a right triangle between the 2 landmarks, to get the landmark's far angle
                landmark_x,landmark_y = self.__field_map.get_landmark_position(landmark_id)
                other_landmark_x, other_landmark_y = self.__field_map.get_landmark_position(other_landmark_id)
                landmarks_xdist = abs(other_landmark_x - landmark_x)
                landmarks_ydist = abs(other_landmark_y - landmark_y)
                # for this right triangle, base angle is the 90, base side is the right side of the landmark, 
                # far side is the side opposite the landmark, top side is the side ot hte left of hte landmark
                # what I WANT is the far angle
                landmark_to_me_slope_deg = 0
                if landmarks_ydist != 0:
                    #logging.getLogger(__name__).info(f"field dist: {actual_field_dist}, top_side: {landmarks_ydist}, far_side: {landmarks_xdist}")
                    landmark_far_angle = self.calc_far_angle(far_side=landmarks_xdist, base_side=actual_field_dist, top_side=landmarks_ydist)

                    # the top_angle we have so far is just part of it, since the right triangle is rotated,
                    # we have to adjust the top_angle by  however much the right triangle is rotated
                    slope_adjust_for_right_triangle_rotation = 180 - landmark_far_angle - 90

                    #logging.getLogger(__name__).info(f"{landmark_id} - landmark far angle: {landmark_far_angle}, slope adjust deg: {slope_adjust_for_right_triangle_rotation}, top angle: {top_angle}")

                    # subtract the landmark-calculated far angle from the base_angle of the main triangle
                    # which will return the slope (in degrees) of my position relative to the landmark
                    #unadjusted_landmark_to_me_slope_deg = top_angle - landmark_far_angle
                    landmark_to_me_slope_deg = slope_adjust_for_right_triangle_rotation + top_angle
                    
                landmark_to_me_slope = math.tan(math.radians(landmark_to_me_slope_deg))
                #logging.getLogger(__name__).info(f"{landmark_id} - to my y = mx + b slope: {landmark_to_me_slope}")


                #not it: y_dist_from_point = (x_mean - known_point_x) * (1/slope)
                #logging.getLogger(__name__).info(f"Landmark {landmark_id}: ({landmark_x},{landmark_y} - Slope To Me: {landmark_to_me_slope}), est dist: {distances[landmark_id]['ground']})")
                # for nw light, should be close to -1.4
                #landmark_to_me_slope = -1.4

                # add both positive and negative slope, just in case
                # can i improve this?

                possible_variants = self.__get_line_point_variations (landmark_x, landmark_y, slope=landmark_to_me_slope, dist=distances[landmark_id]['ground'])
                for p in possible_variants:
                    possible.append(p)

        return possible

    # return heading in degrees vs the x axis. 0 means pointed perpendicular to x axis,
    # negative means pointed x degrees to left of perpendicular to x axis, positive means to the right
    # this method assume the x,y you are providing are relatively accurate.
    # none means not enough info or maybe  problem with angles
    def get_heading(self, x, y, angles):
        all_headings = self.get_possible_headings(x,y,angles)
        #logging.getLogger(__name__).info(f"All possible headings: {all_headings}")
        heading = None

        if len(all_headings) > 1:
            return statistics.mean(all_headings)
        elif len(all_headings) == 1:
            return all_headings[0]
        return heading 
    
    def get_possible_headings (self, x, y, view_angles):
        #logging.getLogger(__name__).info(f"Perspective x:{x}, y:{y}")
        #logging.getLogger(__name__).info(f"Perspective x:{x}, y:{y}, View angles: {view_angles}")
        headings = []

        for base_landmark_id in view_angles:
            base_landmark = max(view_angles[base_landmark_id], key=lambda x:x['confidence'])
            base_x, base_y = self.__field_map.get_landmark_position(base_landmark_id)
            
            # Relative North: Based on the vehicle's assumed position, if it were facing the landmark
            # how much and which direction (degrees) would it need to turn to be facing north (up) on the plane
            relative_north = self.__visual_degrees_calc.calculate_relative_north(
                perspective_x=x,
                perspective_y=y,
                point_x=base_x,
                point_y=base_y
            )
            #logging.getLogger(__name__).info(f"Landmark {base_landmark_id}, x: {base_x}, y: {base_y}, Relative N: {relative_north}")

            # how many degrees, relative to vehicle, was camera pointed
            # for instance, -20 means the camera is pointed 20 degrees left of where vehicle is facing
            base_relative_to_facing_direction = base_landmark['image_heading'] - self.__base_front
            
            img_pixels_off_center = base_landmark['center_x'] - (self.__view_width / 2.0)
            image_relative_deg = self.__visual_degrees_calc.caclulate_horizontal_visual_degrees_given_width_pixels(img_pixels_off_center)

            #logging.getLogger(__name__).info(f"{base_landmark_id} : Camera vs vehicle front {base_relative_to_facing_direction} deg")
            #logging.getLogger(__name__).info(f"{base_landmark_id} : image vs center : {image_relative_deg} deg")

            # turn the map-specified amount from the base landmark
            # assume vehicle at 0, camera at -20, image relative degrees at 18 (right of center)
            # to make the vehicle face the object, the vehicle needs to turn:
            # -20 (match the camera) + 18 (adjust for centering) = -2

            # assume vehicle at 0, camera at +90, image relative degrees at -10
            # vehicle +90 to match camera, then -10 to center image = +80

            landmark_deg_relative_to_vehicle_facing = base_relative_to_facing_direction + image_relative_deg
            #logging.getLogger(__name__).info(f"{base_landmark_id} Landmark relative to vehicle: {landmark_deg_relative_to_vehicle_facing}")

            # to the landmark deg relative, do the opposite of relative north to see where vehicle is facing
            # Note: This calculation is probably more complicated. 
            visible_heading = landmark_deg_relative_to_vehicle_facing + relative_north
            
            # turn the vehicle toward the landmark, then from there turn toward north (according to landmark).
            # We will likely overshoot north, multiply by -1 to calculate what it would take to get us back to N.
            # this is our heading
            # If relative north is >0degrees from us, we are facing left of north (- degrees)

            # if our perspective is below the landmark, we need to flip the heading sign
            if self.__is_heading_negative(landmark_id=base_landmark_id, position_x=x, landmark_x=base_x, relative_north=relative_north, visual_landmark_angle=landmark_deg_relative_to_vehicle_facing):
                visible_heading = -1 * abs(visible_heading)
            else:
                visible_heading = abs(visible_heading)

            #if x < 20 and x > -20 and y < 70 and y > 30:
            #    logging.getLogger(__name__).info(f"Heading: {visible_heading}, Landmark: {base_landmark_id}, Relative N: {relative_north}, Visual Angle: {landmark_deg_relative_to_vehicle_facing}")

            headings.append(visible_heading)

        return headings

    def __is_heading_negative (self, landmark_id, position_x, landmark_x, relative_north, visual_landmark_angle):
        if abs(relative_north) > 180 or abs(visual_landmark_angle) > 180:
            logging.getLogger(__name__).error(f"Bad angle reading, neither of these should ever be > 180 - N:{relative_north}, V:{visual_landmark_angle}")

        # if landmark is left of this position (on map)
        if position_x > landmark_x:
            # if landmark is visually to the right
            if visual_landmark_angle > 0:
                # if combined relative N and visual angle < 180. we are facing westish (negative heading)
                if relative_north + visual_landmark_angle < 180:
                    return True
            elif visual_landmark_angle < 0: # landmark visually on left
                if abs(visual_landmark_angle) < relative_north:
                    return True
        elif position_x < landmark_x: # landmark is to the right, mapwise
            #logging.getLogger(__name__).info(f"landmark: {landmark_id}, visual: {visual_landmark_angle}, relative north: {relative_north}")
            # if landmark is visually to the right
            if visual_landmark_angle > 0:
                # if we are not looking between the landmark and N
                if visual_landmark_angle > abs(relative_north):
                    return True
            elif visual_landmark_angle < 0:
                # both visual landmark angle and relative north are < 0. If we add them
                # together and the total is >= -180, we are facing eastern (positive heading)
                # we will look for opposite situation, since this function is looking for negative heading
                #logging.getLogger(__name__).info(f"visual: {visual_landmark_angle}, relative north: {relative_north}, sum: {visual_landmark_angle + relative_north}")
                if visual_landmark_angle + relative_north < -180:
                    return True
        
        return False


    def is_possible (self, x, y, view_angles, allowed_variance = 0.2, allowed_heading_variance = 0.3):
        base_landmark_id = None
        base_landmark = None
        processed_pairs = []

        total_degrees = 0.0
        degrees_diff = 0.0
        headings = self.get_possible_headings(x,y,view_angles)

        #logging.getLogger(__name__).info(f"Testing Coord Set: {x},{y}. Possible headings: {headings}")
        for base_landmark_id in view_angles:
            base_landmark = max(view_angles[base_landmark_id], key=lambda x:x['confidence'])
            base_x, base_y = self.__field_map.get_landmark_position(base_landmark_id)

            # figure out our heading, based on this landmark
            #relative_north = self.__visual_degrees_calc.calculate_relative_north(
            #    perspective_x=x,
            #    perspective_y=y,
            #    point_x=base_x,
            #    point_y=base_y
            #)

            # how many degrees, relative to vehicle, was camera pointed
            # for instance, -20 means the camera is pointed 20 degrees left of where vehicle is facing
            #base_relative_to_facing_direction = base_landmark['image_heading'] - self.__base_front
            #img_pixels_off_center = base_landmark['center_x'] - (self.__view_width / 2.0)
            #image_relative_deg = self.__visual_degrees_calc.caclulate_horizontal_visual_degrees_given_width_pixels(img_pixels_off_center)


            #landmark_deg_relative_to_vehicle_facing = base_relative_to_facing_direction + image_relative_deg
            #logging.getLogger(__name__).debug(f"{base_landmark_id} Landmark relative to vehicle: {landmark_deg_relative_to_vehicle_facing}")

            # to the landmrk deg relaative, do the opposite of relative north to see where vehicle is facing
            #visible_heading = landmark_deg_relative_to_vehicle_facing + relative_north
            #logging.getLogger(__name__).debug(f"{base_landmark_id} Vehicle heading: {visible_heading}, Landmark relative to vehicle: {landmark_deg_relative_to_vehicle_facing}")
            #headings.append(visible_heading)


            for landmark_id in view_angles[base_landmark_id][0]['relative_deg']:
                if f"{base_landmark_id}|{landmark_id}" not in processed_pairs:
                    processed_pairs.append(f"{base_landmark_id}|{landmark_id}")
                    processed_pairs.append(f"{landmark_id}|{base_landmark_id}")

                    #curr_angle = max(view_angles[landmark_id], key=lambda x:x['confidence'])

                    landmark_x, landmark_y = self.__field_map.get_landmark_position(landmark_id)
                    #horizontal_angle = curr_angle['image_heading']

                    # make a triangle between the 3 points, and get my expected far angle
                    # it should match up with the visible angle
                    expected_angle = self.__visual_degrees_calc.calculate_degrees_given_point_and_perspective(
                        perspective_x=x,
                        perspective_y=y,
                        base_x=base_x,
                        base_y=base_y,
                        point_x=landmark_x,
                        point_y=landmark_y
                    )

                    # 
                    if expected_angle is None:
                        logging.getLogger(__name__).info(f"Position ({x},{y}) leads to weird calculation, not considering this one possible")
                        return False

                    visible_degrees_diff = base_landmark['relative_deg'][landmark_id]
                    #logging.getLogger(__name__).debug(f"{base_landmark_id}/{landmark_id} - Expected far angle: {round(expected_angle,1)}, Visible angle: {round(visible_degrees_diff,1)}")

                    total_degrees += abs(expected_angle)
                    degrees_diff += abs(abs(expected_angle) - abs(visible_degrees_diff))

                    #logging.getLogger(__name__).info(f"Diff: {abs(expected_angle - visible_degrees_diff)}")



                #if angle > 90 and angle < 270:
                #    logging.getLogger(__name__).info(f"{landmark_id} is to my right")

                #else:
                #    logging.getLogger(__name__).info(f"{landmark_id} is to my left")

                #landmark_x, landmark_y = self.__field_map.get_landmark_position(landmark_id=landmark_id)
                #xdist = x - landmark_x
                #ydist = y - landmark_x

                # +x means i'm to the right of it
                # +y means i'm below it

        headings_good = True
        max_heading_variance = 0
        if len(headings) > 1:
            avg_heading = statistics.mean(headings)
            for heading in headings:
                heading_var = abs(heading - avg_heading) / 360.0 
                if heading_var > allowed_heading_variance:
                    headings_good = False
                    #logging.getLogger(__name__).info(f"{x,y} - Heading of {heading} looks bogus compared to avg of {avg_heading}")
                    #logging.getLogger(__name__).info(f"{x,y} - All Headings: {headings}")
                if heading_var > max_heading_variance:
                    max_heading_variance = heading_var

        #logging.getLogger(__name__).info(f"{x,y} - Angle Variance: {degrees_diff / total_degrees}, Heading Variance: {max_heading_variance}")
        acceptable = headings_good and total_degrees > 0 and degrees_diff / total_degrees <= allowed_variance

        #if acceptable:
        #    logging.getLogger(__name__).info(f"{x,y} - Within variance, this coordinate is ok")
        #else:
        #    logging.getLogger(__name__).info(f"{x,y} - Headings good: {headings_good}")

        return acceptable

    def __get_allowed_heading_variance (self):
        allowed_heading_variance = 0.09
        if self.__estimator_mode == EstimatorMode.PRECISE:
            allowed_heading_variance = 0.07
        if self.__estimator_mode == EstimatorMode.VERY_PRECISE:
            allowed_heading_variance = 0.06
        return allowed_heading_variance
    
    # gets estimated coordinates and heading, given located objects and altitude
    # returns x,y,heading,basis
    # heading of 0 means vehicle is pointed north on the axis.
    # + heading means it's pointed to the right of north
    # - heading means it's pointed to the left of north
    def get_coords_and_heading (self, located_objects, view_altitude, lidar_map = None):
        angles = self.extract_object_view_angles(located_objects=located_objects)
        #logging.getLogger(__name__).info(f"Estimated Angles: {angles}")

        distances =self.extract_distances(view_angles=angles, view_altitude=view_altitude, lidar_map = lidar_map)
        #logging.getLogger(__name__).info(f"Estimated distances: {distances}")


        conf = Confidence.CONFIDENCE_VERY_LOW
        coords = []
        if len(located_objects) > 1:
            conf = Confidence.CONFIDENCE_LOW
            coords = self.find_possible_coordinates(
                view_angles=angles, 
                distances=distances, 
                allowed_variance=0.4, # we want to be able to adjust the estimated distances quite a bit. this isnt for accuracy
                allowed_heading_variance = self.__get_allowed_heading_variance())

        # if we got some back, get the heading and return the centroid
        heading = None
        centroid_x = None
        centroid_y = None

        if len(located_objects) > 3:
            conf = Confidence.CONFIDENCE_HIGH
        elif len(located_objects) > 2:
            conf = Confidence.CONFIDENCE_MEDIUM

        # calculate the mean
        if len(coords) > 0:
            # avg coordinates. could possibly do better than this
            x = [p[0] for p in coords]
            y = [p[1] for p in coords]
            centroid_x, centroid_y = (sum(x) / len(coords), sum(y) / len(coords))

            heading = self.get_heading(centroid_x, centroid_y, angles)

        basis = {
            'angles':angles,
            'distances':distances,
            'landmarks':located_objects
        }

        return centroid_x, centroid_y, heading, conf, basis
    
    def get_possible_coords_isolated (self, landmark_id, other_landmark_id, distances, view_angles, filter_out_of_bounds, allowed_variance, allowed_heading_variance, enforce_landmark_preferred_angles = True):
        # this is the angle of the other landmark id relative to this one.
        # positive means th OTHER is to the right. negative means OTHEr is to the left
        selected_this_angle = max(view_angles[landmark_id], key=lambda x:x['confidence'])
        #selected_other_angle = max(view_angles[other_landmark_id], key=lambda x:x['confidence'])
        filtered_coords = []

        viz_angle = selected_this_angle['relative_deg'][other_landmark_id]
        lm_min_preferred = self.__max_or_none(self.__field_map.get_landmark_min_angle_preference(landmark_id), self.__field_map.get_landmark_min_angle_preference(other_landmark_id))
        lm_max_preferred = self.__min_or_none(self.__field_map.get_landmark_max_angle_preference(landmark_id), self.__field_map.get_landmark_max_angle_preference(other_landmark_id))

        if ((lm_max_preferred is None or abs(viz_angle) <= lm_max_preferred) and (lm_min_preferred is None or abs(viz_angle) >= lm_min_preferred)) or enforce_landmark_preferred_angles == False:
            if (lm_min_preferred is not None and abs(viz_angle) < lm_min_preferred) or (lm_max_preferred is not None and abs(viz_angle) > lm_max_preferred):
                logging.getLogger(__name__).info(f"Visual angle of {round(viz_angle,2)} between {landmark_id} and {other_landmark_id} is outside preferred, but using it anyway")

            curr_possibilities = self.__get_possible_coordinates(
                viz_angle=viz_angle, 
                landmark_id=landmark_id, 
                other_landmark_id=other_landmark_id, 
                distances=distances,
                view_angles=view_angles)

            in_bounds_coords = []
            if filter_out_of_bounds:        
                # filter any that are out of bounds. Allow near_bounds to be used, as the robot could wander a little out
                for poss_x, poss_y in curr_possibilities:
                    if self.__field_map.is_near_bounds(x=poss_x, y=poss_y):
                        in_bounds_coords.append((poss_x, poss_y))
            else:
                in_bounds_coords = curr_possibilities

            # filter any impossible coordinates
            for poss_x, poss_y in in_bounds_coords:
                if self.is_possible(poss_x, poss_y, view_angles=view_angles, allowed_variance=allowed_variance, allowed_heading_variance=allowed_heading_variance):
                    filtered_coords.append((poss_x, poss_y))
        else:
            logging.getLogger(__name__).info(f"Visual angle of {round(viz_angle, 2)} between {landmark_id} and {other_landmark_id} is outside preferred range, not using that one")

        return filtered_coords


    def find_possible_coordinates (self, view_angles, distances, filter_out_of_bounds = True, allowed_variance = 0.3, allowed_heading_variance = 0.1, enforce_landmark_preferred_angles = True):
        possible_coords = []
        computed = []
        thread_params = []

        coord_sets = []

        # if we only have one coordinate, we can not compute that.
        if len(distances) > 1:
            # given each pair
            for landmark_id in distances:
                for other_landmark_id in distances:
                    pair_key = f"{landmark_id}|{other_landmark_id}"
                    if other_landmark_id != landmark_id and pair_key not in computed:
                        computed.append(pair_key) # don't recalculate for exact same info

                        if self.__use_multithreading:
                            thread_params.append((
                                self,
                                landmark_id,
                                other_landmark_id,
                                distances,
                                view_angles,
                                filter_out_of_bounds,
                                allowed_variance,
                                allowed_heading_variance,
                                enforce_landmark_preferred_angles
                            ))
                        else:
                            coord_sets.append(
                                self.get_possible_coords_isolated(
                                    landmark_id=landmark_id,
                                    other_landmark_id=other_landmark_id,
                                    distances=distances,
                                    view_angles=view_angles,
                                    filter_out_of_bounds=filter_out_of_bounds,
                                    allowed_variance=allowed_variance,
                                    allowed_heading_variance=allowed_heading_variance,
                                    enforce_landmark_preferred_angles=enforce_landmark_preferred_angles
                                )
                            )

        # if multithreading, we need to wait for results to come in
        if self.__use_multithreading:
            pool = PositionThreadManager.get_thread_pool()
            async_results = pool.starmap_async(external_get_possible_coords_isolated, thread_params)
            logging.getLogger(__name__).debug("Waiting for threads to finish getting coords")
            
            try:
                for thread_result in async_results.get():
                    coord_sets.append(thread_result)
            except TimeoutError as te:
                # is the thread pool corrupt in this case? Does it have a zombie
                logging.getLogger(__name__).info("Timed out, some or all coord sets may not be included")


        
        final_possible_coords = []
        for cs in coord_sets:
            final_possible_coords = final_possible_coords + cs

        return final_possible_coords
    
    def calc_far_angle (self, far_side, base_side, top_side):
        part_1 = base_side ** 2 + top_side ** 2 - far_side ** 2
        part_2 = 2 * base_side * top_side
        far_angle_rad = math.acos(part_1 / part_2)
        return math.degrees(far_angle_rad)

    # go different directions from xy
    def __get_line_point_variations (self, x, y, slope, dist):
        variants = []
        variants.append(self.__get_line_point(x,y,slope=slope, dist=dist, step_size=1))
        variants.append(self.__get_line_point(x,y,slope=-1*slope, dist=dist, step_size=1))
        variants.append(self.__get_line_point(x,y,slope=slope, dist=dist, step_size=-1))
        variants.append(self.__get_line_point(x,y,slope=-1*slope, dist=dist, step_size=-1))

        return variants


    # gives point that is given distance from x,y at the given slope
    # this is a hack and should be replaced with an actual formula
    def __get_line_point(self, x, y, slope, dist, step_size = 1):
        x_coord = x
        y_coord = y

        while self.get_distance(x_coord, y_coord, x, y) < dist:
            x_coord += step_size
            y_coord += step_size * slope

        return x_coord, y_coord


    def get_distance (self, x1, y1, x2, y2):
        if y2 == y1:
            return abs(x2 - x1)

        return math.sqrt(((x2 - x1)**2) + ((y2 - y1) **2))

    def __min_or_none (self, val1, val2):
        if val1 is None and val2 is None:
            return None
        elif val1 is None:
            return val2
        elif val2 is None:
            return val1
        
        return min(val1, val2)

    def __max_or_none (self, val1, val2):
        if val1 is None and val2 is None:
            return None
        elif val1 is None:
            return val2
        elif val2 is None:
            return val1
        
        return max(val1, val2)

def external_get_possible_coords_isolated (estimator_inst, landmark_id, other_landmark_id, distances, view_angles, filter_out_of_bounds, allowed_variance, allowed_heading_variance, enforce_landmark_preferred_angles):
    return estimator_inst.get_possible_coords_isolated (landmark_id, other_landmark_id, distances, view_angles, filter_out_of_bounds, allowed_variance, allowed_heading_variance, enforce_landmark_preferred_angles)

if __name__ == "__main__":
    print("houdy")
