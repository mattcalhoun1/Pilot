class EmitterViewedLocation:
    def __init__(self, x1, y1, x2, y2, confidence):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.confidence = confidence

    def get_height (self):
        return abs(self.y2 - self.y1)
    
    def get_width (self):
        return abs(self.x2 - self.x1)
    
    def get_center (self):
        return (self.x1 + (self.get_width()/2),self.y1 + (self.get_height()/2))

