import logging
import random
import math
import statistics
from trig.trig import BasicTrigCalc

# translates between an LVPS field and a scaled field
# this helps to visualize or simulate a field
class FieldScaler:
    def __init__(self, field_map, scaled_width, scaled_height, scale_factor, invert_x_axis = False, invert_y_axis = False):
        self.__field_map = field_map
        self.__scaled_height = scaled_height
        self.__scaled_width = scaled_width
        self.__map_scaler = scale_factor

        self.__invert_y_axis = invert_y_axis
        self.__invert_x_axis = invert_x_axis

        bound_x_min, bound_y_min, bound_x_max, bound_y_max = field_map.get_boundaries()
        self.__x_scaler = scale_factor * (scaled_height / (bound_x_max - bound_x_min))
        self.__y_scaler = scale_factor * (scaled_width / (bound_y_max - bound_y_min))
        # this isn't very good, and only works accurately for square shape.
        # the dist scaling really needs to be done by calculating distance differently
        self.__dist_scaler = statistics.mean([self.__x_scaler, self.__y_scaler])

        bound_center_x = bound_x_min + ((bound_x_max - bound_x_min) / 2)
        bound_center_y = bound_y_min + ((bound_y_max - bound_y_min) / 2)
        scaled_down_center_x, scaled_down_center_y = self.__scale_coords_down(bound_center_x, bound_center_y)


        scaled_field_center_x = self.__scaled_width / 2
        scaled_field_center_y = self.__scaled_height / 2

        self.__shift_x = scaled_field_center_x - scaled_down_center_x
        self.__shift_y = scaled_field_center_y - scaled_down_center_y

        #logging.getLogger(__name__).info(f"LVPS Map center: ({bound_center_x},{bound_center_y}), scaled to ({scaled_down_center_x},{scaled_down_center_y})")
        #logging.getLogger(__name__).info(f"Scaled field center: ({scaled_field_center_x},{scaled_field_center_y})")
        #logging.getLogger(__name__).info(f"After scaling, x coords will be shifted: {self.__shift_x}, y coords will be shifted {self.__shift_y}")

        self.__trig_calc = BasicTrigCalc()

    def get_scaled_height (self):
        return self.__scaled_height
    
    def get_scaled_width (self):
        return self.__scaled_width

    def get_map (self):
        return self.__field_map
    
    def scale_lvps_distance_to_sim (self, dist):
        return dist * self.__dist_scaler
    
    def scale_sim_to_lvps_distance (self, dist):
        return dist / self.__dist_scaler

    def get_lvps_coords (self, sim_x, sim_y):
        if self.__invert_x_axis:
            sim_x = self.__scaled_width - sim_x

        if self.__invert_y_axis:
            sim_y = self.__scaled_height - sim_y

        return self.__scale_coords_up(*self.__get_unshifted_coords(sim_x, sim_y))

    def get_scaled_coords (self, lvps_x, lvps_y):
        scaled_x, scaled_y = self.__get_shifted_coords(*(self.__scale_coords_down(lvps_x,lvps_y)))

        if self.__invert_y_axis:
            scaled_y = self.__scaled_height - scaled_y
        if self.__invert_x_axis:
            scaled_x = self.__scaled_width - scaled_x

        return scaled_x, scaled_y
    
    def is_scaled_coord_visible (self, sim_perspective_x, sim_perspective_y, sim_target_x, sim_target_y, sight_range_sim):
        if self.__get_distance(sim_perspective_x, sim_perspective_y, sim_target_x, sim_target_y) <= sight_range_sim:
            #logging.getLogger(__name__).info("coord is within dist")
            lvps_p_x, lvps_p_y = self.get_lvps_coords(sim_perspective_x, sim_perspective_y)
            lvps_target_x, lvps_target_y = self.get_lvps_coords(sim_target_x, sim_target_y)

            blocked, obstacle_id = self.__field_map.is_path_blocked(lvps_p_x, lvps_p_y, lvps_target_x, lvps_target_y)
            return not blocked
        return False

    def is_lvps_coord_visible (self, lvps_perspective_x, lvps_perspective_y, lvps_target_x, lvps_target_y, lvps_sight_range):
        if self.__get_distance(lvps_perspective_x, lvps_perspective_y, lvps_target_x, lvps_target_y) <= lvps_sight_range:
            blocked, obstacle_id = self.__field_map.is_path_blocked(lvps_perspective_x, lvps_perspective_y, lvps_target_x, lvps_target_y)
            return not blocked
        return False

    # checks whether given coord is blocked based on lvps coords
    def __is_lvps_coord_blocked (self, lvps_starting_x, lvps_starting_y, lvps_x, lvps_y):
        if lvps_starting_y == lvps_y and lvps_starting_x == lvps_x:
            return False

        #logging.getLogger(__name__).info("IN lvps check")
        bound_x_min, bound_y_min, bound_x_max, bound_y_max = self.__field_map.get_boundaries()
        if lvps_x <= bound_x_min or lvps_x >= bound_x_max or lvps_y <= bound_y_min or lvps_y >= bound_y_max:
            #logging.getLogger(__name__).info(f"{lvps_x},{lvps_y} is out of bounds")
            return True

        is_blocked, obstacle_id = self.__field_map.is_blocked(lvps_x, lvps_y)
        if is_blocked:
            #logging.getLogger(__name__).info(f"{lvps_x},{lvps_y} is blocked by {obstacle_id}")
            return True

        path_blocked, obstacle_id = self.__field_map.is_path_blocked (lvps_starting_x, lvps_starting_y, lvps_x, lvps_y)
        #if not path_blocked:
        #    logging.getLogger(__name__).info(f"lvps:LVPS Path from {lvps_starting_x},{lvps_starting_y} to {lvps_x},{lvps_y} is OPEN")
        #else:
        #    logging.getLogger(__name__).info(f"Lvps path from {lvps_starting_x},{lvps_starting_y} to {lvps_x},{lvps_y} is blockedQ!")

        return path_blocked
    
    def __is_sim_coord_blocked (self, sim_starting_x, sim_starting_y, sim_x, sim_y):
        if sim_starting_y == sim_y and sim_starting_x == sim_x:
            return False

        if self.is_in_bounds(sim_x, sim_y) == False or self.is_obstacle(sim_x, sim_y):
            return True
        
        # check map blockage
        lvps_start_x, lvps_start_y = self.get_lvps_coords(sim_starting_x, sim_starting_y)
        lvps_target_x, lvps_target_y = self.get_lvps_coords(sim_x, sim_y)

        path_blocked, obstacle_id = self.__field_map.is_path_blocked (lvps_start_x, lvps_start_y, lvps_target_x, lvps_target_y)
        #if not path_blocked:
        #    logging.getLogger(__name__).info(f"sim:LVPS Path from {lvps_start_x},{lvps_start_y} to {lvps_target_x},{lvps_target_y} is OPEN")
        return path_blocked

        
    def get_nearest_travelable_lvps_coords (self, starting_x, starting_y, target_x, target_y, max_dist):
        return self.__get_nearest_travelable_coords (starting_x, starting_y, target_x, target_y, max_dist, self.__is_lvps_coord_blocked)

    def get_nearest_travelable_sim_coords (self, starting_x, starting_y, target_x, target_y, max_dist):
        return self.__get_nearest_travelable_coords (starting_x, starting_y, target_x, target_y, max_dist, self.__is_sim_coord_blocked)


    def __get_nearest_travelable_coords (self, starting_x, starting_y, target_x, target_y, max_dist, block_check_method):
        # Get the point on the line between here and the target that is no farther than max dist
        # im sure there is a simple math calculation to do this, but i'm skipping that for now
        traveled = 0
        curr_x = starting_x
        curr_y = starting_y
        last_ok_x = curr_x
        last_ok_y = curr_y

        full_dist = self.__get_distance(curr_x, curr_y, target_x, target_y)

        slope = (target_y - starting_y) / (target_x - starting_x)
        dist_increment = 0.05 * full_dist

        # convert the slope to degrees
        cartesian_degrees = math.degrees(math.atan(slope))
        
        while (traveled < max_dist and full_dist > self.__get_distance(starting_x, starting_y, curr_x, curr_y)):
            traveled += dist_increment
            curr_x, curr_y = self.__trig_calc.get_coords_for_angle_and_distance (
                heading=cartesian_degrees,
                x = starting_x,
                y = starting_y,
                distance = traveled,
                is_forward = target_x > starting_x)

            traveled = self.__get_distance(starting_x, starting_y, curr_x, curr_y)

            if self.__get_distance(starting_x, starting_y, curr_x, curr_y) <= max_dist:
                if block_check_method(starting_x, starting_y, curr_x, curr_y) == False:
                    last_ok_x = curr_x
                    last_ok_y = curr_y

        #logging.getLogger(__name__).info(f"Position {starting_x},{starting_y} wants to go up to {max_dist} toward {target_x},{target_y} returning nearest coords: {last_ok_x},{last_ok_y}")

        return last_ok_x, last_ok_y

    def is_in_bounds (self, sim_x, sim_y):
        lvps_x,lvps_y = self.get_lvps_coords(sim_x, sim_y)
        bound_x_min, bound_y_min, bound_x_max, bound_y_max = self.__field_map.get_boundaries()
        if lvps_x <= bound_x_min or lvps_x >= bound_x_max or lvps_y <= bound_y_min or lvps_y >= bound_y_max:
            return False
        return True

    def is_obstacle (self, sim_x, sim_y):
        lvps_x,lvps_y = self.get_lvps_coords(sim_x, sim_y)
        is_blocked, obstacle_id = self.__field_map.is_blocked(lvps_x, lvps_y)
        return is_blocked

    def get_random_traversable_coords (self):
        max_attempts = 1000
        curr_attempt = 0

        min_x, min_y, max_x, max_y = self.get_map().get_boundaries()

        while curr_attempt < max_attempts:
            curr_attempt += 1
            lvps_x = random.randrange(int(min_x*10),int(max_x)*10)/10
            lvps_y = random.randrange(int(min_y*10),int(max_y)*10)/10

            if self.__field_map.is_in_bounds(lvps_x, lvps_y):
                blocked, obstacle_id = self.__field_map.is_blocked(lvps_x, lvps_y)
                if not blocked:
                    return lvps_x,lvps_y

        logging.getLogger(__name__).error("Unable to find traversable coords!")
        return None,None

    def __get_shifted_coords (self, x, y):
        return (x + self.__shift_x, y + self.__shift_y)

    def __get_unshifted_coords (self, x, y):
        return (x - self.__shift_x, y - self.__shift_y)

    def __scale_coords_up (self, x, y):
        scaled_x = x / self.__x_scaler
        scaled_y = y / self.__y_scaler

        scaled_x += (1 / self.__x_scaler) / 2
        scaled_y += (1 / self.__y_scaler) / 2

        #logging.getLogger(__name__).info(f"Simulation coord {(x,y)} scales up to LVPS {(scaled_x,scaled_y)}")

        return scaled_x, scaled_y

    def __scale_coords_down (self, x, y):
        scaled_x = x * self.__x_scaler
        scaled_y = y * self.__y_scaler

        # round or floor?
        scaled_x = round(scaled_x)
        scaled_y = round(scaled_y)

        #logging.getLogger(__name__).info(f"LVPS coord {(x,y)} scales down to {(scaled_x,scaled_y)}")

        return scaled_x, scaled_y    

    def __get_distance(self, x1, y1, x2, y2):
        dx = x1 - x2
        dy = y1 - y2
        return math.sqrt(dx**2 + dy**2)