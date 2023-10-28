from landmarks.emitter_viewed_location import EmitterViewedLocation
import logging
import time

class EmitterGroup:
    def __init__(self):
        self.__emitters = []
        self.__identity = None
        self.__time = time.time()
        self.__confidence = 0

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

        center_x, center_y = self.__emitters[0].get_center()
        center_y += self.get_height()
        return center_x, center_y

        # returns the average of top and bottom points
        # center means center of the vertically aligned points
        # is this accurate enough measure of center?
        #x_total = 0
        #y_total = 0

        #for i in [0,-1]:
        #    e_center_x, e_center_y = self.__emitters[i].get_center()
        #    x_total += e_center_x
        #    y_total += e_center_y
        
        #return ((x_total / 2), (y_total / 2))
        
    def get_emitters (self):
        return self.__emitters
    
    def get_height (self):
        top_center_x, top_center_y = self.__emitters[0].get_center()
        bottom_center_x, bottom_center_y = self.__emitters[-1].get_center()
        return abs(bottom_center_y - top_center_y)

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
