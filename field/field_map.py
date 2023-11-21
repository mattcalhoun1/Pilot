from field.field_point import Point
import math
import logging

class FieldMap:
    def __init__(self, landmarks, shape='rectangle', boundaries = None, obstacles = None, search = None, near_boundaries = None, name = None):
        if shape != 'rectangle':
            raise Exception("Only rectangle supported")
        
        self.__landmarks = landmarks
        self.__relative_distances = self.__calculate_relative_distances()
        self.__boundaries = boundaries
        self.__obstacles = obstacles
        self.__search = search
        self.__near_boundaries = near_boundaries
        self.__name = name
        
        if self.__boundaries is None:
            logging.getLogger(__name__).warning("No boundaries given on map, all points will be considered in bounds")
        else:
            logging.getLogger(__name__).debug(f"Boundaries: {self.__boundaries}")

        if self.__obstacles is None:
            logging.getLogger(__name__).warning("No obstacles given on map, all points will be considered open")
        else:
            logging.getLogger(__name__).debug(f"Obstacles: {self.__obstacles}")

        if self.__near_boundaries is None:
            logging.getLogger(__name__).warning("No near boundaries given on map, boundaries will be replicated for near boundaries")
        else:
            logging.getLogger(__name__).debug(f"Near Boundaries: {self.__near_boundaries}")

    def get_name (self):
        return self.__name

    def get_searchable_objects (self):
        return self.__search if self.__search is not None else []

    def get_boundaries(self):
        if self.__boundaries is None:
            return None, None, None, None
        
        return self.__boundaries['xmin'], self.__boundaries['ymin'], self.__boundaries['xmax'], self.__boundaries['ymax']

    def get_width (self):
        if self.__boundaries is None:
            return -1
        return abs(self.__boundaries['xmax'] - self.__boundaries['xmin'])

    def get_length (self):
        if self.__boundaries is None:
            return -1
        return abs(self.__boundaries['ymax'] - self.__boundaries['ymin'])

    # tells whether a given point is in bounds
    def is_in_bounds (self, x, y):
        if self.__boundaries is None:
            return True

        return x >= self.__boundaries['xmin'] and x <= self.__boundaries['xmax'] and y >= self.__boundaries['ymin'] and y <= self.__boundaries['ymax']

    # tells whether a given point is in bounds or close to it
    def is_near_bounds (self, x, y):
        if self.is_in_bounds(x, y):
            return True

        # check the near-bounds
        return x >= self.__near_boundaries['xmin'] and x <= self.__near_boundaries['xmax'] and y >= self.__near_boundaries['ymin'] and y <= self.__near_boundaries['ymax']

    # tells whether a given point is blocked by an obstacle, as well as the obstacle id
    def is_blocked (self, x, y):
        if self.__obstacles is None:
            return False, None

        for o in self.__obstacles:
            o_bounds = self.__obstacles[o]
            if x >= o_bounds['xmin'] and x <= o_bounds['xmax'] and y >= o_bounds['ymin'] and y <= o_bounds['ymax']:
                return True, o
        
        return False, None

    # tells whether direct path between two points is blocked by a mapped obstacle
    def is_path_blocked (self, x1, y1, x2, y2, path_width = 0):
        if self.__obstacles is None:
            return False, None

        # if width is to be taken into account, we need to do this check 4 times with a different path segment
        path_segments = [
            (Point(x1, y1),Point(x2, y2)),
        ]

        if path_width > 0:
            # add additional segments to account for width of the path
            # this isn't perfect, and assumes the object (width) is square and moving in a fixed orientation
            path_segments = [
                (Point(x1, y1),Point(x2, y2)),
                (Point(x1+(path_width/2), y1),Point(x2+(path_width/2), y2)),
                (Point(x1-(path_width/2), y1),Point(x2-(path_width/2), y2)),
                (Point(x1, y1+(path_width/2)),Point(x2, y2+(path_width/2))),
                (Point(x1, y1-(path_width/2)),Point(x2, y2-(path_width/2))),
            ]

        for path_segment_p, path_segment_q in path_segments:
            for o in self.__obstacles:
                segments = [
                    (Point(self.__obstacles[o]['xmin'],self.__obstacles[o]['ymin']),Point(self.__obstacles[o]['xmax'],self.__obstacles[o]['ymin'])),
                    (Point(self.__obstacles[o]['xmin'],self.__obstacles[o]['ymin']),Point(self.__obstacles[o]['xmin'],self.__obstacles[o]['ymax'])),
                    (Point(self.__obstacles[o]['xmin'],self.__obstacles[o]['ymax']),Point(self.__obstacles[o]['xmax'],self.__obstacles[o]['ymax'])),
                    (Point(self.__obstacles[o]['xmax'],self.__obstacles[o]['ymin']),Point(self.__obstacles[o]['xmax'],self.__obstacles[o]['ymax'])),
                ]
                for obst_seg_p, obst_seg_q in segments:
                    if self.__do_intersect(path_segment_p, path_segment_q, obst_seg_p, obst_seg_q):
                        #logging.getLogger(__name__).info("".join([
                        #    "The following segments intersect: ",
                        #    f"({obst_seg_p.x},{obst_seg_p.y}) ... ",
                        #    f"({obst_seg_q.x},{obst_seg_q.y}) ... ",
                        #]))
                        return True, o

        return False, None
    
    # returns xmin, ymin, xmax, ymax for given obstacle
    def get_obstacle_bounds (self, obstacle_id):
        if self.__obstacles is not None and obstacle_id in self.__obstacles:
            return self.__obstacles[obstacle_id]['xmin'], self.__obstacles['ymin'], self.__obstacles[obstacle_id]['xmax'], self.__obstacles['ymax']
        
        return None, None, None, None

    def is_landmark_known (self, landmark_id):
        return landmark_id in self.__landmarks
    
    def get_landmark_confidence_threshold (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return landmark['confidence']

    def get_landmark_priority (self, landmark_id):
        if landmark_id in self.__landmarks and 'priority' in self.__landmarks[landmark_id]:
            landmark = self.__landmarks[landmark_id]
            return landmark['priority']
        return 0

    def get_landmark_tier (self, landmark_id):
        if landmark_id in self.__landmarks and 'tier' in self.__landmarks[landmark_id]:
            landmark = self.__landmarks[landmark_id]
            return landmark['tier']
        return "unknown"

    def is_landmark_lidar_visible (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return 'lidar_visible' in landmark.keys() and landmark['lidar_visible']

    def get_landmark_min_angle_preference (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return landmark['min_visual_angle_preference'] if 'min_visual_angle_preference' in landmark else None

    def get_landmark_max_angle_preference (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return landmark['max_visual_angle_preference'] if 'max_visual_angle_preference' in landmark else None

    def get_landmark_position (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return landmark['x'], landmark['y']

    def get_landmark_altitude (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return landmark['altitude']

    def get_landmark_height (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return landmark['height']
    
    def get_landmark_width (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return landmark['width']

    def get_landmark_type (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return landmark['type']

    def get_landmark_pattern (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return landmark['pattern']

    def get_landmark_model (self, landmark_id):
        landmark = self.__landmarks[landmark_id]
        return landmark['model']

    def get_search_obj_height (self, object_type):
        searchable = self.__search[object_type]
        return searchable['height']
    
    def get_search_width (self, object_type):
        searchable = self.__search[object_type]
        return searchable['width']

    def is_search_object_lidar_visible (self, object_type):
        searchable = self.__search[object_type]
        return 'lidar_visible' in searchable.keys() and searchable['lidar_visible']

    # returns IDs of all models required for this map
    def get_required_models (self):
        required_models = []
        for l in self.__landmarks:
            if self.__landmarks[l]['model'] not in required_models:
                required_models.append(self.__landmarks[l]['model'])
        
        if self.__search is not None:
            for s in self.__search:
                if self.__search[s]['model'] not in required_models:
                    required_models.append(self.__search[s]['model'])

        return required_models

    def get_landmark_by_model_type_pattern (self, landmark_model, landmark_type, landmark_pattern = None):
        for lid in self.__landmarks:
            curr_landmark = self.__landmarks[lid]
            if 'model' in curr_landmark and curr_landmark['model'] == landmark_model:
                if 'type' in curr_landmark and curr_landmark['type'] == landmark_type:
                    if landmark_pattern is None or 'pattern' in curr_landmark and curr_landmark['pattern'] == landmark_pattern:
                        return lid
        return None
        
    def get_slope (self, landmark_id, landmark_id_2):
        slope = None
        if self.is_landmark_known(landmark_id=landmark_id) and self.is_landmark_known(landmark_id=landmark_id_2):
            landmark_x, landmark_y = self.get_landmark_position(landmark_id=landmark_id)
            landmark2_x, landmark2_y = self.get_landmark_position(landmark_id=landmark_id_2)
            slope = (landmark2_y - landmark_y) / (landmark2_x - landmark_x)
        return slope
    
    def get_quadrant (self, landmark_id):
        # quadrant order:  II | I
        #                  -------
        #                 III | IV
        quadrant = None
        if self.is_landmark_known(landmark_id=landmark_id):
            x,y = self.get_landmark_position(landmark_id=landmark_id)
            if x >= 0 and y >= 0:
                return 1
            elif x < 0 and y >= 0:
                return 2
            elif x >= 0 and y < 0:
                return 4
            else:
                return 3
            
        return quadrant


    def get_distance (self, landmark_id_1, landmark_id_2):
        if landmark_id_1 != landmark_id_2 and self.is_landmark_known(landmark_id_1) and self.is_landmark_known(landmark_id_2):
            return self.__relative_distances[landmark_id_1][landmark_id_2]
        
        return 0
    
    def __calculate_relative_distances (self):
        # go through each landmark and cache its distances to every other
        relative = {}

        for landmark_id in self.__landmarks:
            if landmark_id not in relative:
                relative[landmark_id] = {}

            for landmark_2_id in self.__landmarks:
                if landmark_2_id != landmark_id and landmark_2_id not in relative[landmark_id]:
                    x1,y1 = self.get_landmark_position(landmark_id=landmark_id)
                    x2,y2 = self.get_landmark_position(landmark_id=landmark_2_id)
                    dist = self.__dist((x1, y1), (x2, y2))

                    relative[landmark_id][landmark_2_id] = dist
                    if landmark_2_id not in relative:
                        relative[landmark_2_id] = {}
                        relative[landmark_2_id][landmark_id] = dist
                    elif landmark_id not in relative[landmark_2_id]:
                        relative[landmark_2_id][landmark_id] = dist

        return relative
    
    def __dist(self, p1, p2):
        return math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )

    # The following code is adapted from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/#
    # Given three collinear points p, q, r, the function checks if  
    # point q lies on line segment 'pr'  
    def __on_segment(self, p, q, r): 
        if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and 
            (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))): 
            return True
        return False
    
    def __orientation(self, p, q, r): 
        # to find the orientation of an ordered triplet (p,q,r) 
        # function returns the following values: 
        # 0 : Collinear points 
        # 1 : Clockwise points 
        # 2 : Counterclockwise 
        
        # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/  
        # for details of below formula.  
        
        val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y)) 
        if (val > 0): 
            
            # Clockwise orientation 
            return 1
        elif (val < 0): 
            
            # Counterclockwise orientation 
            return 2
        else: 
            
            # Collinear orientation 
            return 0
    
    # The main function that returns true if  
    # the line segment 'p1q1' and 'p2q2' intersect. 
    def __do_intersect(self, p1,q1,p2,q2): 
        
        # Find the 4 orientations required for  
        # the general and special cases 
        o1 = self.__orientation(p1, q1, p2) 
        o2 = self.__orientation(p1, q1, q2) 
        o3 = self.__orientation(p2, q2, p1) 
        o4 = self.__orientation(p2, q2, q1) 
    
        # General case 
        if ((o1 != o2) and (o3 != o4)): 
            return True
    
        # Special Cases 
    
        # p1 , q1 and p2 are collinear and p2 lies on segment p1q1 
        if ((o1 == 0) and self.__on_segment(p1, p2, q1)): 
            return True
    
        # p1 , q1 and q2 are collinear and q2 lies on segment p1q1 
        if ((o2 == 0) and self.__on_segment(p1, q2, q1)): 
            return True
    
        # p2 , q2 and p1 are collinear and p1 lies on segment p2q2 
        if ((o3 == 0) and self.__on_segment(p2, p1, q2)): 
            return True
    
        # p2 , q2 and q1 are collinear and q1 lies on segment p2q2 
        if ((o4 == 0) and self.__on_segment(p2, q1, q2)): 
            return True
    
        # If none of the cases 
        return False