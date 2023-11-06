from landmarks.emitter_viewed_location import EmitterViewedLocation
import logging
import time
import statistics

class EmitterGroupPattern:
    VERTICAL_LINE = 0
    SQUARE = 1
    SIDEWAYS_TRIANGLE = 2

class EmitterGroup:
    def __init__(self):
        self.__emitters = []
        self.__identity = None
        self.__time = time.time()
        self.__confidence = 0
        self.__pattern = EmitterGroupPattern.VERTICAL_LINE # default
        
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
        elif self.__pattern == EmitterGroupPattern.SIDEWAYS_TRIANGLE:
            center_x1, center_y1 = self.__emitters[0].get_center()
            center_x2, center_y2 = self.__emitters[1].get_center()
            center_x_last, center_y_last = self.__emitters[-1].get_center()
            center_x = min(center_x1, center_x2) + int(abs(center_x1 - center_x2)/2)
            center_y = statistics.mean([center_y1, center_y_last])
            
            
        return center_x, center_y
       
    def get_emitters (self):
        return self.__emitters
    
    def get_height (self):
        if self.__pattern == EmitterGroupPattern.VERTICAL_LINE or self.__pattern == EmitterGroupPattern.SIDEWAYS_TRIANGLE:
            top_center_x, top_center_y = self.__emitters[0].get_center()
            bottom_center_x, bottom_center_y = self.__emitters[-1].get_center()
            return abs(bottom_center_y - top_center_y)
        elif self.__pattern == EmitterGroupPattern.SQUARE:
            # height is abs value of ( avg of top two - avg bottom two )
            top_x1, top_y1 = self.__emitters[0].get_center()
            top_x2, top_y2 = self.__emitters[1].get_center()
            mid_top_y = statistics.mean([top_y1, top_y2])

            bottom_x1, bottom_y1 = self.__emitters[2].get_center()
            bottom_x2, bottom_y2 = self.__emitters[3].get_center()
            mid_bottom_y = statistics.mean([bottom_y1, bottom_y2])
            
            return int(abs(mid_top_y - mid_bottom_y))
            
        return None

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
