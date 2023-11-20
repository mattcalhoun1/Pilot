import math
import logging

# helps determine distance of object, given how it appears
# versus the known size (height or width)
class VisualDistanceCalculator:
    def __init__(self, horizontal_fov, vertical_fov, view_width, view_height):
        self.__fov_h = horizontal_fov
        self.__fov_v = vertical_fov
        self.__view_w = view_width
        self.__view_h = view_height
    

    # distance given position above/below perspective vs object?
    # this assumes height of object is same no matter which side we are viewing it from
    # altitude is vs center, not vs top or bottom

    # returns ground_distance, distance to object top, distance to object bottom
    def estimate_distances_given_viewed_height (self, view_altitude, obj_height_degrees, obj_known_height, obj_center_altitude, adjust_for_altitude = True):
        # We know our perspective's altitude, the objects height and altitude
        # so we can draw a horizontally level line between us and our altitude below or above the object
        # now we have a right triangle
        obj_top = (obj_known_height / 2) + obj_center_altitude
        obj_bottom = obj_center_altitude - (obj_known_height / 2)
        top_distance = 0
        ground_distance = 0
        bottom_distance = 0

        if adjust_for_altitude == False or (view_altitude < obj_top and view_altitude > obj_bottom):
            # if we are beside the object (not above or below), we draw a line over, and we have 2 right triangles
            # instead of just one
            ground_distance, top_distance, bottom_distance = self.__est_dist_given_viewed_height_beside (view_altitude, obj_height_degrees, obj_known_height, obj_top, obj_bottom)
        else:
            ground_distance, top_distance, bottom_distance = self.__est_dist_given_viewed_height_not_beside (view_altitude, obj_height_degrees, obj_known_height, obj_top, obj_bottom)


        return ground_distance, top_distance, bottom_distance

    # given a certain far side and far angle, along with estimated top/base sides,
    # return some other top/base possibilities that will match up with the far side    
    #def get_possible_top_base_distances (self, far_side, far_angle, estimated_base, estimated_top, max_adjustment = 0.5):
    #    #

    #    adjusted_base_angle = False
    #    while base_angle is None:
    #        try:
    #            base_angle = trig_calc.calc_base_angle(far_side=actual_field_dist, base_side=base_side, top_side=top_side)
    #        except Exception:
    ##            # our estimated distances may not add up to a 180 degree agle,
    #            # bump them until they do
    #            adjusted_base_angle = True 
    #            base_side += base_side * .01
    #            top_side += top_side * .01
    #    if adjusted_base_angle:
    #        logging.getLogger(__name__).info(f"{landmark_id}/{other_landmark_id} *ADJUSTED* - Actual Field Dist: {actual_field_dist}, Base Side: {base_side}, Top Side: {top_side}")
    #    
    #    top_angle = 180 - base_angle - abs(viz_angle)


    def __est_dist_given_viewed_height_beside (self, view_altitude, obj_height_degrees, obj_known_height, obj_top, obj_bottom):
        top_distance = 0
        bottom_distance = 0
        ground_distance = 0

        triangle_base_pos = view_altitude
        height_degree_ratio = obj_height_degrees / obj_known_height


        # triangle looking up, with us at the base
        top_tri_far_side = obj_top - triangle_base_pos
        top_tri_shortened_amount = obj_known_height - top_tri_far_side

        # figure out the degree height of top portion of triangle far side
        top_tri_shortened_degrees = top_tri_shortened_amount * height_degree_ratio
        top_tri_height_degrees = obj_height_degrees - top_tri_shortened_degrees

        # long side is the distance to top
        top_distance = self.__calculate_hypotenuse(visual_degrees=top_tri_height_degrees, far_side=top_tri_far_side)
        ground_distance = self.__calculate_non_hypotenuse_side (top_tri_far_side, top_distance)

        # now calculate distance to the bottom
        # draw triangle looking down, with us at the upside-down-base
        bottom_tri_far_side = obj_known_height - top_tri_far_side
        bottom_tri_height_degrees = obj_height_degrees - top_tri_height_degrees
        bottom_distance = self.__calculate_hypotenuse(visual_degrees=bottom_tri_height_degrees, far_side=bottom_tri_far_side)

        return ground_distance, top_distance, bottom_distance

    def __est_dist_given_viewed_height_not_beside (self, view_altitude, obj_height_degrees, obj_known_height, obj_top, obj_bottom):
        top_distance = 0
        bottom_distance = 0

        triangle_base_pos = view_altitude
        far_side = obj_top - triangle_base_pos
        obj_lengthened_amount = obj_bottom - triangle_base_pos
        view_from_below = triangle_base_pos <= obj_bottom
        if view_from_below == False:
            far_side = triangle_base_pos - obj_bottom
            obj_lengthened_amount = triangle_base_pos - obj_top
            
        # figure out how many degrees were just added by increasing the height of the triangle
        height_degree_ratio = obj_height_degrees / obj_known_height
        added_degrees = obj_lengthened_amount * height_degree_ratio
        full_height_degrees = obj_height_degrees + added_degrees

        # long side is the distance to top or bottom
        long_side = self.__calculate_hypotenuse(visual_degrees=full_height_degrees, far_side=far_side)

        ground_distance = self.__calculate_non_hypotenuse_side (far_side, long_side)
        added_side_dist = math.hypot(ground_distance, obj_lengthened_amount)

        if view_from_below:
            top_distance = long_side
            bottom_distance = added_side_dist
        else:
            bottom_distance = long_side
            top_distance = added_side_dist


        return ground_distance, top_distance, bottom_distance
            
    def __calculate_hypotenuse (self, visual_degrees, far_side):
        visual_rad = math.radians(visual_degrees)
        return far_side / (math.sin(visual_rad))


    ### Do i need these -- ###


    def calculate_vertical_hypotenuse (self, visual_vertical_degrees):
        return self.calculate_hypotenuse(visual_degrees=visual_vertical_degrees, adjacent_length=self.__emitter_height)


    # calculates the non-hypotenuse side,
    # given the hypotenuse plus one side.
    def __calculate_non_hypotenuse_side (self, side_length, hypotenuse_length):
        return math.sqrt((hypotenuse_length**2) - (side_length**2))
    
    # using the horizontal visual angle and hypotenuse (calculated in previous step),
    # returns opposite length
    def calculate_horizontal_opposite (self, horizontal_hypotenuse_length, visual_horizontal_degrees):
        # if there is no visible angle (emitter is centered), the opposite length is 0
        if visual_horizontal_degrees == 0.0:
            return 0.0
        return abs(horizontal_hypotenuse_length * math.sin(math.radians(visual_horizontal_degrees)))

    # given a set of visual vertical and horizontal degrees, returns relative x and y positions
    # for horizontal degrees and vertical degrees, 0 is straight out
    # negative degrees means emitter appears below or left of the sensor
    # positive degrees means emitter appears above or right of the sensor
    def calculate_relative_coordinates (self, visual_vertical_degrees, visual_horizontal_degrees):

        # 1) Find distance between emitter and current position
        emitter_distance = self.calculate_emitter_distance(visual_vertical_degrees=visual_vertical_degrees)

        # 2) For an imagined right triangle where the knowns are hypotenuse (distance between emitter/sensor),
        #    visual horizontal angle (and obviously the 90 degree point), calculate the other 2 sides
        opposite = self.calculate_horizontal_opposite(
            horizontal_hypotenuse_length=emitter_distance,
            visual_horizontal_degrees=visual_horizontal_degrees)

        adjacent = emitter_distance
        if opposite > 0:
            adjacent = self.calculate_non_hypotenuse_side(side_length=opposite, hypotenuse_length=emitter_distance)

        # relative y will be adjacent + emitter y
        relative_y = adjacent + self.__emitter_y

        # x will be emitter_x +- opposite, depending on whether the horizontal angle goes left or right
        relative_x = self.__emitter_x
        if visual_horizontal_degrees > 0:
            relative_x += opposite
        elif visual_horizontal_degrees < 0:
            relative_x -= opposite

        return relative_x, relative_y
