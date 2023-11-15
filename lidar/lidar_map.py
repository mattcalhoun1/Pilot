import logging
import math
import json
class LidarMap:
    def __init__(self, offset : int, granularity : float, lidar_data : str):
        lidar_raw = lidar_data.split('|')
        self.__measurement_map = {}
        for i,measurement in enumerate(lidar_raw):
            measurement = float(measurement)
            if measurement > 0.0:
                adjustedAngle = (i * granularity) + offset
                if adjustedAngle > 360.0:
                    adjustedAngle -= 360.0

                self.__measurement_map[adjustedAngle] = measurement
        
        self.__available_angles = sorted(self.__measurement_map.keys())
        #logging.getLogger(__name__).info(f"Available angles: {self.__available_angles}")

    def get_available_angles (self):
        return self.__available_angles

    def get_lidar_data (self):
        return self.__measurement_map

    def get_measurement (self, vehicle_relative_angle : float, max_allowed_drift : float):
        #logging.getLogger(__name__).info(f"Pulling Lidar for angle {vehicle_relative_angle}")
        # find the closest available
        closest_available = self.get_closest_available_angle(vehicle_relative_angle)

        logging.getLogger(__name__).info(f"Closest available to {vehicle_relative_angle} is {closest_available}")

        # if it's close enough 0 or 360, also check the opposite
        if vehicle_relative_angle - max_allowed_drift < 0:
            alternative = self.get_closest_available_angle(360.0)
            if abs(alternative - 360.0 - vehicle_relative_angle) < abs(closest_available - vehicle_relative_angle):
                closest_available = alternative
        elif vehicle_relative_angle + max_allowed_drift > 360.0:
            alternative = self.get_closest_available_angle(0.0)
            if abs((360.0 + alternative) - vehicle_relative_angle) < abs(closest_available - vehicle_relative_angle):
                closest_available = alternative

        if abs(vehicle_relative_angle - closest_available) <= max_allowed_drift:
            if closest_available in self.__measurement_map:
                return self.__measurement_map[closest_available]
            else:
                logging.getLogger(__name__).error(f"Lidar measurement was found at {closest_available} but is now gone!")
        return -1.0

    def get_closest_available_angle (self, desired_angle: float):
        # if there's a direct hit, return that
        if desired_angle in self.__available_angles:
            return self.__available_angles[self.__available_angles.index(desired_angle)]

        lowest = min(self.__available_angles)
        highest = max(self.__available_angles)

        if desired_angle < lowest:
            return lowest
        elif desired_angle > highest:
            return highest

        # do a binary search to find nearest
        sub_center = math.floor(len(self.__available_angles) / 2)
        sub_length = len(self.__available_angles)
        closest_so_far = self.__available_angles[sub_center]
        sub_offset = 0
        while sub_center > 0 and sub_center < (sub_center + sub_length):
            if round(desired_angle,2) == round(self.__available_angles[sub_center],2):
                return round(desired_angle,2)
            elif abs(desired_angle - self.__available_angles[sub_center]) < abs(desired_angle - closest_so_far):
                closest_so_far = self.__available_angles[sub_center]
            
            if sub_length == 1:
                if abs(self.__available_angles[sub_center] - desired_angle) < abs(closest_so_far - desired_angle):
                    return self.__available_angles[sub_center]
                return closest_so_far
            elif desired_angle > self.__available_angles[sub_center]:
                sub_offset = sub_center
                sub_length /= 2
                sub_length = round(sub_length)
                sub_center += math.floor(sub_length / 2)
            elif desired_angle < self.__available_angles[sub_center]:
                sub_offset = sub_center
                sub_length /= 2
                sub_length = round(sub_length)
                sub_center -= math.floor(sub_length / 2)
        
        if abs(self.__available_angles[0] - desired_angle) < abs(closest_so_far - desired_angle):
            return self.__available_angles[0]
        elif abs(self.__available_angles[len(self.__available_angles)-1] - desired_angle) < abs(closest_so_far - desired_angle):
            return self.__available_angles[len(self.__available_angles)-1]


        return closest_so_far
