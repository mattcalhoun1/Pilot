import logging
import math
from trig.trig import BasicTrigCalc

class VisualDegreesCalculator:
    def __init__(self, horizontal_fov, vertical_fov, view_width, view_height):
        self.__fov_h = horizontal_fov
        self.__fov_v = vertical_fov
        self.__view_w = view_width
        self.__view_h = view_height
        self.__fov_vertical_adjust_factor = 1

    # returns visual degrees between 2 points, based on field of view/img height
    # returns the ration of distance between the two points to the view size, times the camera's fov
    def caclulate_vertical_visual_degrees (self, x1, y1, x2, y2):
        dist = abs(y2-y1)

        return self.__adjust_vertical_degrees(self.__fov_v * (dist / self.__view_h))

    def caclulate_vertical_visual_degrees_given_height_pixels (self, height_pixels):
        return self.__adjust_vertical_degrees(self.__fov_v * (height_pixels / self.__view_h))

    def __adjust_vertical_degrees (self, raw_degrees):
        # adjust if necessary
        return raw_degrees * self.__fov_vertical_adjust_factor

    # returns visual degrees between 2 points, based on field of view/img height
    # Assumes the object is being viewed from a perfectly level viewpoint
    # returns the ration of distance between the two points to the view size, times the camera's fov
    def caclulate_horizontal_visual_degrees (self, x1, y1, x2, y2):
        dist = abs(x2-x1)

        return self.__fov_h * (dist / self.__view_w)

    def caclulate_horizontal_visual_degrees_given_width_pixels (self, width_pixels):
        return self.__fov_h * (width_pixels / self.__view_w)
    
    def calculate_pixels_given_horizontal_degree (self, horz_degrees):
        # fov is 120 deg horz, 66 deg vert
        # image res is 1280 x 720

        # horizontal = 10.666 pixels per degree
        # vertical = 10.9 pixes per degree
        return (self.__view_w / self.__fov_h) * horz_degrees

    def calculate_degrees_from_slope (self, x1, y1, x2, y2):
        slope = (y2 - y1) / (x2 - x1)

        return math.degrees(math.atan(slope))
    
    def calculate_degrees_from_slope_given_offset (self, x1, y1, x2, y2, offset):
        return offset - self.calculate_degrees_from_slope(x1, y1, x2, y2)

    def calculate_degrees_given_point_and_perspective (self, perspective_x, perspective_y, base_x, base_y, point_x, point_y):
        # sitting at perspective, how many degrees difference is seen from base to point?
        return BasicTrigCalc().calc_far_angle(
            far_side=self.__dist((base_x, base_y),(point_x,point_y)),
            base_side=self.__dist((perspective_x, perspective_y),(base_x,base_y)),
            top_side=self.__dist((perspective_x, perspective_y),(point_x,point_y))
        )
    
    # returns degrees to the left or right of the point that aims at the x axis
    # for instance, -90 means look to the left of point x,y by 90 degrees to be looking at north
    def calculate_relative_north (self, perspective_x, perspective_y, point_x, point_y):

        #far_side = math.dist((point_x,point_y),(point2_x,point2_y))
        new_point_x = perspective_x
        new_point_y = point_y
        
        # make a right triangle and get the expected far (perspective) angle
        angled_side = self.__dist((perspective_x,perspective_y),(point_x,point_y))
        vertical_side = self.__dist((perspective_x,perspective_y),(new_point_x,new_point_y))
        north_deg_diff = math.degrees(math.asin(vertical_side/angled_side))

        relative_degrees_north = 0
        if perspective_y > point_y:
            relative_degrees_north = north_deg_diff + 90
            if perspective_x < point_x:
                relative_degrees_north *= -1            
        elif perspective_y < point_y:
            relative_degrees_north = 90 - north_deg_diff
            if perspective_x < point_x:
                relative_degrees_north *= -1

        # if it's to the right, ??

        # subtract the expected angle from 90

        # does it matter which side of the axis im on

        return relative_degrees_north
    
    def __dist(self, p1, p2):
        return math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
