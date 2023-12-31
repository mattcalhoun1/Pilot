from landmarks.emitter_viewed_location import EmitterViewedLocation
import logging
import time
import statistics

class EmitterGroupPattern:
    VERTICAL_LINE = 0
    SQUARE = 1
    SIDEWAYS_TRIANGLE_LEFT = 2
    SIDEWAYS_TRIANGLE_RIGHT = 3

class EmitterGroup:
    def __init__(self, barrel_distortion_at_edge = 0.0, image_width = 0.0):
        self.__emitters = []
        self.__identity = None
        self.__time = time.time()
        self.__confidence = 0
        self.__pattern = EmitterGroupPattern.VERTICAL_LINE # default
        self.__barrel_distortion_at_edge = barrel_distortion_at_edge
        self.__image_width = image_width
        
    def set_pattern(self, pattern):
        self.__pattern = pattern
        
    def get_pattern(self):
        return self.__pattern

    def set_confidence (self, confidence):
        self.__confidence = confidence
    
    def get_confidence (self):
        return self.__confidence

    def set_identity (self, identity):
        self.__identity = identity
    
    def get_identity (self):
        return self.__identity

    def set_time (self, seconds):
        self.__time = seconds
    
    def get_time(self):
        return self.__time

    def add_emitter (self, emitter : EmitterViewedLocation):
        # insert the emitter so that lowest y value is at the top
        insert_location = None
        for i,e in enumerate(self.__emitters):
            if e.y1 > emitter.y1:
                insert_location = i
                break

        if insert_location is not None:
            self.__emitters.insert(insert_location, emitter)
        else:
            self.__emitters.append(emitter)

    def get_group_center (self):
        if len(self.__emitters) == 0:
            return (0,0)

        if self.__pattern == EmitterGroupPattern.VERTICAL_LINE:
            center_x, center_y = self.__emitters[0].get_center()
            center_y += int(self.get_height()/2)
        elif self.__pattern == EmitterGroupPattern.SQUARE:
            center_x1, center_y1 = self.__emitters[0].get_center()
            center_x2, center_y2 = self.__emitters[1].get_center()
            center_x_last, center_y_last = self.__emitters[-1].get_center()
            center_x = min(center_x1, center_x2) + int(abs(center_x1 - center_x2)/2)
            center_y = statistics.mean([center_y1, center_y2]) + (self.get_height()/2)
        elif self.__pattern in [EmitterGroupPattern.SIDEWAYS_TRIANGLE_LEFT, EmitterGroupPattern.SIDEWAYS_TRIANGLE_RIGHT]:
            center_x1, center_y1 = self.__emitters[0].get_center()
            #center_x2, center_y2 = self.__emitters[1].get_center()
            center_x_last, center_y_last = self.__emitters[-1].get_center()
            #center_x = min(center_x1, center_x2) + int(abs(center_x1 - center_x2)/2)
            center_x = min(center_x1, center_x_last) + int(abs(center_x1 - center_x_last)/2)
            center_y = statistics.mean([center_y1, center_y_last])
            
            
        return center_x, center_y
       
    def get_emitters (self):
        return self.__emitters
    
    def get_height (self):
        distorted_height = None
        corrected_height = None
        estimated_center_x = None # for calculating distortion

        if self.__pattern in [EmitterGroupPattern.VERTICAL_LINE, EmitterGroupPattern.SIDEWAYS_TRIANGLE_LEFT, EmitterGroupPattern.SIDEWAYS_TRIANGLE_RIGHT]:
            top_center_x, top_center_y = self.__emitters[0].get_center()
            bottom_center_x, bottom_center_y = self.__emitters[-1].get_center()
            distorted_height = abs(bottom_center_y - top_center_y)
            estimated_center_x = statistics.mean([top_center_x, bottom_center_x])

        elif self.__pattern == EmitterGroupPattern.SQUARE:
            # height is abs value of ( avg of top two - avg bottom two )
            top_x1, top_y1 = self.__emitters[0].get_center()
            top_x2, top_y2 = self.__emitters[1].get_center()
            mid_top_y = statistics.mean([top_y1, top_y2])

            bottom_x1, bottom_y1 = self.__emitters[2].get_center()
            bottom_x2, bottom_y2 = self.__emitters[3].get_center()
            mid_bottom_y = statistics.mean([bottom_y1, bottom_y2])

            estimated_center_x = statistics.mean([top_x1, top_x2, bottom_x1, bottom_x2])
            
            distorted_height = int(abs(mid_top_y - mid_bottom_y))

        if distorted_height is not None:
            # get distance from center
            #logging.getLogger(__name__).info(f"Original height: {distorted_height}")
            corrected_height = distorted_height - (distorted_height * self.get_height_distortion_multiplier_at_x(estimated_center_x))
            #logging.getLogger(__name__).info(f"Corrected height: {corrected_height}")

        return corrected_height

    def get_height_distortion_multiplier_at_x (self, x):
        # get the center's distance from y center of image
        image_x_center = self.__image_width / 2
        dist_from_image_center = abs(x - image_x_center)
        distortion_multiplier = (dist_from_image_center / image_x_center) * self.__barrel_distortion_at_edge
        #logging.getLogger(__name__).info(f"Applying edge size correction of {distortion_multiplier} percent")
        return distortion_multiplier


    def get_width (self):
        # get the center of each point. furthest right minus furthest left is the width
        farthest_left, farthest_right = self.__emitters[0].get_center()
        for emitter in self.__emitters:
            e_center_x, e_center_y = emitter.get_center()
            if e_center_x < farthest_left:
                farthest_left = e_center_x
            if e_center_x > farthest_right:
                farthest_right = e_center_x
        
        return abs(farthest_right - farthest_left)

    # returns the farthest left and farthest right emitters in the group
    def get_left_and_right (self):
        farthest_left = self.__emitters[0]
        farthest_left_center_x, farthest_left_y = farthest_left.get_center()
        farthest_right = self.__emitters[0]
        farthest_right_center_x, farthest_right_y = farthest_right.get_center()

        for emitter in self.__emitters:
            e_center_x, e_center_y = emitter.get_center()
            if e_center_x < farthest_left_center_x:
                farthest_left = emitter
                farthest_left_center_x, farthest_left_y = farthest_left.get_center()
            elif e_center_x > farthest_right_center_x:
                farthest_right = emitter
                farthest_right_center_x, farthest_right_y = farthest_right.get_center()
        
        return farthest_left, farthest_right

    def get_right (self):
        pass

    def get_top (self):
        return self.__emitters[0]

    def get_bottom (self):
        return self.__emitters[-1]

    def log_emitters (self):
        logging.getLogger(__name__).info(f"Emitters For: {self.get_identity()}")
        for i,e in enumerate(self.__emitters):
            logging.getLogger(__name__).info(f"E{i}: ({e.x1},{e.y1}) - ({e.x2},{e.y2})")
