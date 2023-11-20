from position.position_estimator import PositionEstimator
from position.confidence import Confidence
from position.estimator_mode import EstimatorMode
from field.field_map import FieldMap
import numpy as np
from sklearn.cluster import KMeans
import statistics
import logging

class PositionEstimatorWithClustering (PositionEstimator):
    def __init__(self, field_map : FieldMap, horizontal_fov, vertical_fov, view_width, view_height, base_front=90.0, use_multithreading = True, estimator_mode = EstimatorMode.VERY_PRECISE, max_lidar_drift_deg = 1.5, max_lidar_visual_variance_pct = 0.33):
        PositionEstimator.__init__(self, field_map, horizontal_fov, vertical_fov, view_width, view_height, base_front, use_multithreading, estimator_mode, max_lidar_drift_deg, max_lidar_visual_variance_pct)
        logging.getLogger(__name__).info("Position Clustering is enabled, for greater accuracy.")
        self.__default_heading_clusters = 2
        self.__heading_std_dev = 2
        self.__default_coord_clusters = 2
        self.__coord_std_dev = 2
        self.__estimator_mode = estimator_mode

    def get_possible_headings (self, x, y, view_angles):
        poss = PositionEstimator.get_possible_headings(self, x, y, view_angles)
        #logging.getLogger(__name__).info(f"Possible headings, before clustering cleanup: {poss}")

        cleaned = self.__remove_heading_outliers(poss)
        
        return cleaned
    
    def count_lidar_hits (self, distances):
        hits = 0
        for d in distances:
            if distances[d]['islidar'] == True:
                hits += 1
        
        return hits

    def get_coords_and_heading (self, located_objects, view_altitude, lidar_map = None, enforce_landmark_preferred_angles = True):
        angles = self.extract_object_view_angles(located_objects=located_objects)
        #logging.getLogger(__name__).info(f"Estimated Angles: {angles}")

        distances =self.extract_distances(view_angles=angles, view_altitude=view_altitude, lidar_map = lidar_map)
        lidar_hits = self.count_lidar_hits(distances)

        #logging.getLogger(__name__).info(f"Estimated distances: {distances}")

        allowed_heading_variance = 0.07
        if self.__estimator_mode == EstimatorMode.PRECISE:
            allowed_heading_variance = 0.05
        if self.__estimator_mode == EstimatorMode.VERY_PRECISE:
            allowed_heading_variance = 0.03

        conf = Confidence.CONFIDENCE_VERY_LOW
        coords = []
        if len(located_objects) > 1:
            conf = Confidence.CONFIDENCE_LOW
            coords = self.find_possible_coordinates(
                view_angles=angles, 
                distances=distances, 
                allowed_variance=0.5, # we want to be able to adjust the estimated distances quite a bit. this isnt for accuracy
                allowed_heading_variance = allowed_heading_variance,
                enforce_landmark_preferred_angles=enforce_landmark_preferred_angles)
            #logging.getLogger(__name__).info(f"All possible: {coords}")

        # if we got some back, get the heading and return the centroid
        heading = None
        centroid_x = None
        centroid_y = None

        if len(located_objects) > 3 or (len(located_objects) > 2 and lidar_hits >= 2):
            conf = Confidence.CONFIDENCE_HIGH
        elif len(located_objects) >= 2 and lidar_hits > 0:
            conf = Confidence.CONFIDENCE_MEDIUM
        elif len(located_objects) > 2:
            conf = Confidence.CONFIDENCE_MEDIUM

        

        # use clustering to remove outliers if enough samples
        if len(coords) > 2:
            #logging.getLogger(__name__).info("Removing outliers")
            coords = self.__remove_coordinates_outliers(coords)
            #logging.getLogger(__name__).info(f"{len(coords)} Removed outliers")

        # calculate the mean of whatever is left
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

    def __remove_heading_outliers (self, headings):
        cleaned_headings = []
        # only trigger clustering if there is a lot of deviation
        if len(headings) > 2 and statistics.stdev(headings) > self.__heading_std_dev:
            # try a couple of differnt cluster sizes, and if there
            # is a large disparity, leave off the outliers
            two_clusters = self.__get_largest_headings_cluster(headings, self.__default_heading_clusters)
            if len(two_clusters) != len(headings):
                cleaned_headings = two_clusters
            else:
                cleaned_headings = headings
        else:
            # not enough to determine
            cleaned_headings = headings
        
        return cleaned_headings
        
    
    def __get_largest_headings_cluster (self, headings, n_clusters):
        largest = []
        
        np_headings = np.array(headings).reshape(-1, 1)
        model = KMeans(n_clusters=n_clusters, random_state=0, n_init=2).fit(np_headings)
        pred = model.labels_
        
        groups = {}
        for i,p in enumerate(pred):
            #logging.getLogger(__name__).info(f"{headings[i]} : {p}")
            if p not in groups:
                groups[p] = [headings[i],]
            else:
                groups[p].append(headings[i])
        
        # find the largest groups
        largest_cluster = None
        largest_amount = 0
        for g in groups:
            if len(groups[g]) > largest_amount:
                largest_amount = len(groups[g])
                largest_cluster = g
        
        return groups[largest_cluster]
                
    def __remove_coordinates_outliers (self, all_coords):
        cleaned_coords = []
        if len(all_coords) > 2:
            # try a couple of differnt cluster sizes, and if there
            # is a large disparity, leave off the outliers
            two_clusters = self.__get_largest_coordinates_cluster(all_coords, self.__default_coord_clusters)
            if len(two_clusters) != len(all_coords):
                cleaned_coords = two_clusters
            else:
                cleaned_coords = all_coords
        else:
            # not enough to determine
            cleaned_coords = all_coords
        
        #logging.getLogger(__name__).info(f"Cleaned: {cleaned_coords}")
        return cleaned_coords
        
    
    def __get_largest_coordinates_cluster (self, all_coords, n_clusters):
        largest = []
        distances = []
        
        for i, c in enumerate(all_coords):
            # calc distance from 0,0, maybe not the best choice
            distances.append(self.get_distance(c[0],c[1],0,0))

        # only trigger clustering if there is a lot of deviation
        if statistics.stdev(distances) > self.__coord_std_dev:
            np_distances = np.array(distances).reshape(-1, 1)
            model = KMeans(n_clusters=n_clusters, random_state=0, n_init=2).fit(np_distances)
            pred = model.labels_
            
            groups = {}
            for i,p in enumerate(pred):
                #logging.getLogger(__name__).info(f"{all_coords[i]} : {p}")
                if p not in groups:
                    groups[p] = [all_coords[i],]
                else:
                    groups[p].append(all_coords[i])
            
            # find the largest groups
            largest_cluster = None
            largest_amount = 0
            for g in groups:
                if len(groups[g]) > largest_amount:
                    largest_amount = len(groups[g])
                    largest_cluster = g
            
            return groups[largest_cluster]
        
        return all_coords

def external_get_possible_coords_isolated (estimator_inst, landmark_id, other_landmark_id, distances, view_angles, filter_out_of_bounds, allowed_variance, allowed_heading_variance):
    return estimator_inst.get_possible_coords_isolated (landmark_id, other_landmark_id, distances, view_angles, filter_out_of_bounds, allowed_variance, allowed_heading_variance)

if __name__ == "__main__":
    print("in main")