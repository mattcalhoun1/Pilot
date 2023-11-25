import logging
import math
from lidar.lidar_map import LidarMap
from field.field_map import FieldMap
import numpy as np

class PathFinder:
    def __init__(self, field_map: FieldMap = None, vehicle_width = 13.0, vehicle_length = 22.0, vehicle_height = 16.0):
        self.__field_map = field_map
        self.__vehicle_width = vehicle_width
        self.__vehicle_height = vehicle_height
        self.__vehicle_length = vehicle_length

    def is_close_enough (self, start_x, start_y, end_x, end_y, max_distance):
        return self.__get_distance(start_x, start_y, end_x, end_y) <= max_distance
    
    def find_rotation (self, current_heading, target_heading):
        # there will be two ways to get there. calculate them both and pick the shortest one
        
        # add 180 to both 
        adjusted_current = current_heading + 180
        adjusted_target = target_heading + 180
        
        # now we have 0 to 360, where 0 is north, 1 is one degree right of north, 359 is one degree west of north
        option1 = adjusted_target - adjusted_current
        option2 = option1 - 360
        if option1 < 0:
            option2 = 360 - abs(option1)
        
        logging.getLogger(__name__).info(f"From {current_heading} to {target_heading}, option1: {option1}, option2: {option2})")
        
        return option1 if abs(option1) < abs(option2) else option2
            

    # this method currently only allows a max of two legs of a trip, meaning it will only make a single turn 
    # to go around a pre-identified obstacle
    def find_potential_paths (self, start_x : float, start_y : float, end_x : float, end_y : float, lidar_map : LidarMap, current_heading : float):
        # finds a couple of ways to get to the given location
        direct_heading, direct_dist = self.find_direct_path (start_x=start_x, start_y=start_y, end_x=end_x, end_y=end_y)
        if lidar_map is None or self.is_path_clear_including_vehicle_width(desired_heading=direct_heading, distance=direct_dist, lidar_map=lidar_map, current_heading=current_heading):
            # if we don't have a field map, or the field map indicates the path is open
            if self.__is_path_map_plausible (start_x = start_x, start_y = start_y, end_x = end_x, end_y = end_y):
                return [
                    [(direct_heading, direct_dist, end_x, end_y),]
                ]
        

        logging.getLogger(__name__).info("direct path is not open, finding potential paths")

        # if the path was not clear, look for a way around
        angle_increment = 0.5
        max_adjust = 45.0

        first_leg_starting_fraction = 0.99 # what percentage of the path shoudl we cover in the first leg, we will loop and lower this as needed
        first_leg_adjust_increment = 0.1 # we will drop by 10% until we can find a solution
        min_first_leg_fraction = 0.3

        desired_paths = 2 # how many paths until we quit looping

        paths = [] # array of arrays of tuples [ [(heading,dist),(leg2_heading, leg2_dist),...],[...] ]
        distances = [] # distance of each path, so the can be sorted

        for first_leg_fraction in np.arange(first_leg_starting_fraction, min_first_leg_fraction - 0.01, -1*first_leg_adjust_increment):
            for heading_adjust_amount in np.arange(0.5, max_adjust + 0.1, angle_increment):
                # check the positive and negative path around
                potential_left_heading = self.__rescale_heading(direct_heading - heading_adjust_amount)
                potential_right_heading = self.__rescale_heading(direct_heading + heading_adjust_amount)

                for potential_heading in [potential_left_heading, potential_right_heading]:
                    
                    # heading 0 == north. heading -90 = west, 90 = east

                    # what would the x,y point be if we went the first leg distance in the potential heading direction
                    trial_dist = direct_dist * first_leg_fraction

                    # convert to an x,y coordinate
                    first_leg_x, first_leg_y = self.find_point(start_x=start_x, start_y=start_y, degrees=potential_heading, distance=trial_dist)

                    # if not in bounds, ignore it
                    if self.__field_map is None or self.__field_map.is_in_bounds(first_leg_x, first_leg_y):
                        # if map indicates it's blocked, ignore it
                        if self.__is_path_map_plausible (start_x = start_x, start_y = start_y, end_x = first_leg_x, end_y = first_leg_y):
                            # if the map indicates or that there's no clear path from there to the end goal                            
                            if self.__is_path_map_plausible (start_x = first_leg_x, start_y = first_leg_y, end_x = end_x, end_y = end_y):
                                # see if the lidar shows blocked
                                if lidar_map is None or self.is_path_clear_including_vehicle_width (
                                    desired_heading = potential_heading,
                                    distance = trial_dist, 
                                    lidar_map = lidar_map,
                                    current_heading = current_heading):

                                    last_leg_heading, last_leg_dist = self.find_direct_path(first_leg_x, first_leg_y, end_x, end_y)
                                    paths.append([
                                        (potential_heading, trial_dist, first_leg_x, first_leg_y),(last_leg_heading, last_leg_dist, end_x, end_y)
                                    ])
                                    distances.append(trial_dist + last_leg_dist)

                                    if len(paths) >= desired_paths:
                                        break

                if len(paths) >= desired_paths:
                    break
            if len(paths) >= desired_paths:
                break
        
        # sort by distance
        sorted_paths = []
        sorted_distances = []
        for i, dist in enumerate(distances):
            insert_pos = -1
            
            # find a place to insert this one, given its dist
            for sorted_i, sorted_dist in enumerate(sorted_distances):
                if dist < sorted_dist:
                    insert_pos = sorted_i
                    break

            if insert_pos >= 0:
                sorted_paths.insert(insert_pos, paths[i])
                sorted_distances.insert(insert_pos, dist)
            else:
                sorted_paths.append(paths[i])
                sorted_distances.append(dist)
        
        return sorted_paths

    # returns true if the given leg is shown to be plausible (unblocked) by the map
    # this does not actually look at lidar for live obstructions. it is just based on map
    # if map is unavailable, all paths are considered plausible
    def __is_path_map_plausible (self, start_x, start_y, end_x, end_y):
        plausible = True
        if self.__field_map is not None:
            blocked, obstacle_id = self.__field_map.is_path_blocked(
                x1 = start_x, y1 = start_y, 
                x2 = end_x, y2 = end_y,
                path_width = max(self.__vehicle_width, self.__vehicle_length))
            plausible = not blocked
        
        return plausible

    # finds a point at the given distance
    def find_point (self, start_x, start_y, degrees, distance):
        # the easy ones, parallel to either axis
        if round(degrees) == 0:
            return start_x, start_y + distance
        if round(abs(degrees)) == 180:
            return start_x, start_y - distance
        if round(degrees) == 90:
            return start_x + distance, start_y
        if round(degrees) == -90:
            return start_x - distance, start_y

        # make a right triangle to get the x, y difference
        # then use knowledge of the degrees to decide which differences to add/subtract

        # we want degrees difference from the x axis , since slope  and x/y position is relative to that axis
        angle = abs(abs(degrees) - 90)
        y_diff = distance * math.sin(math.radians(angle))
        x_diff = math.sqrt(distance**2 - y_diff**2)

        if degrees > 0 and degrees < 90.0:
            return start_x + x_diff, start_y + y_diff
        if degrees > 90.0:
            return start_x + x_diff, start_y - y_diff
        
        if degrees < 0 and degrees > -90.0:
            return start_x - x_diff, start_y + y_diff
        if degrees < -90.0:
            return start_x - x_diff, start_y - y_diff 


    def is_path_clear_including_vehicle_width (self, desired_heading : float, distance: float, lidar_map : LidarMap, current_heading : float):
        # right triangle, where direct_dist is the 90 degree back, 1/2 vehicle width is the 90 deg bottom
        # find the bottom angle
        hypotenuse = math.hypot((self.__vehicle_width/2), distance)
        side_angle = math.degrees(math.asin((self.__vehicle_width/2)/hypotenuse))

        # desired heading is relative to north, need to get vehicle-relative, which is what the lidar is
        desired_relative_heading = self.find_rotation (current_heading, desired_heading)

        # check whether the given angle is open according to lidar
        left_heading = self.__rescale_heading(desired_relative_heading - side_angle)
        right_heading = self.__rescale_heading(desired_relative_heading + side_angle)

        return  self.is_path_clear(left_heading, distance=distance, lidar_map=lidar_map) and \
            self.is_path_clear(desired_relative_heading, distance=distance, lidar_map=lidar_map) and \
            self.is_path_clear(right_heading, distance=distance, lidar_map=lidar_map)



    # if the heading has moved outside of range, it adjusts
    def __rescale_heading (self, heading):
        if heading > 180:
            return -1 * (360 - heading)
        if heading < -180:
            return 360 + heading
        return heading

    def is_path_clear (self, heading : float, distance : float, lidar_map : LidarMap):

        # convert the heading from 0 to 180 / -180 to 0 
        # instead to 0 to 360, matching lidar map
        converted_heading = heading
        if heading < 0:
            converted_heading = 180 + (180-abs(heading))

        lidar_dist = lidar_map.get_measurement (converted_heading, 0.5) # allowed drift should be calculated somehow, not hardcoded

        return lidar_dist <= 0 or lidar_dist > distance


    # returns a heading and distance for a straight line
    # not taking into account known obstacles
    def find_direct_path (self, start_x, start_y, end_x, end_y):
        
        # do distance first
        dist = self.__get_distance(start_x, start_y, end_x, end_y)
        
        is_east = end_x > start_x
        is_north = end_y > start_y
        
        # create a right triangle using my X as the side parallel to x axis
        x_parallel_side = abs(end_y - start_y)
        y_parallel_side = abs(end_x - start_x)
        
        # we have all sides, calculate the angle of the x_parallel_side
        # that will be the abs value of degrees off NORTH we have to face to be facing the new coordinates
        # we want the y parallel angle
        # arcsin (yp / hyp)
        degrees = math.degrees(math.asin(y_parallel_side / dist))
        
        if not is_north:
            degrees += 90

        # if the west of us, the degrees must be negative
        if not is_east:
            degrees *= -1
        
        
        return degrees,dist

    def __get_distance (self, x1, y1, x2, y2):
        return math.sqrt(((x2 - x1)**2) + ((y2 - y1) **2))
