from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from camera.camera_info import CameraInfo
from pilot.actions.action_base import ActionBase
import logging
import time
import math

class SearchAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        super().__init__(vehicle=vehicle, pilot_nav=pilot_nav, pilot_logger=pilot_logger, pilot_config=pilot_config, pilot=pilot)

        self.__lidar_enabled_search = self.get_pilot_config()['Lidar']['EnabledSearch']
        self.__lidar_max_age = self.get_pilot_config()['Lidar']['MaxAge']
        self.__lidar_drift_tolerance = self.get_pilot_config()['Lidar']['MaxDriftDegrees']
        self.__lidar_visual_variance_pct = self.get_pilot_config()['Lidar']['MaxVisualDistVariancePct']

        self.__camera_horz_fov = [
            CameraInfo.get_fov_horizontal(self.get_pilot_config()['Cameras']['Left']['Config']),
            CameraInfo.get_fov_horizontal(self.get_pilot_config()['Cameras']['Right']['Config'])
        ]

    def get_name (self):
        return "Search"

    def execute (self, params):
        # should include either single object or multiple objects to search for
        if 'object' in params:
            return self.search([params['object'],]
        
        refresh_position = False if 'position' not in params else params['position']

        return self.search(params['objects'], refresh_position)

    def search (self, objects, refresh_position = False):
        if self.get_vehicle().wait_for_ready() and self.get_vehicle().get_all_configurations():
            logging.getLogger(__name__).info(f"Searching for {objects}")

            if refresh_position:
                self.find_new_position()

            located_objects = self.get_pilot_nav().locate_objects_multi_angles(objects=objects)
            if len(located_objects) > 0:
                logging.getLogger(__name__).info(f"Located {len(located_objects)} Objects In This Space!")

                # determine actual position/heading so we can record good info about this
                # assume location was record prior to beginning this search
                x, y, heading, confidence, coord_age = self.get_pilot_nav().get_last_coords_and_heading()

                for cam in located_objects:
                    angles = located_objects[cam]['angles']
                    distances = located_objects[cam]['distances']

                    for obj_id in angles:
                        obj_type = obj_id.split('.')[0]
                        obj_rel_deg = angles[obj_id][0]['image_heading'] + angles[obj_id][0]['image_rel_deg']
                        obj_est_dist = distances[obj_id]['ground']
                        logging.getLogger(__name__).info(f"Current Location ({x},{y}) Heading {heading} ==> Obj:{obj_type} : Relative: {obj_rel_deg}, Dist: {obj_est_dist}")

                        obj_veh_relative_heading = self.__get_vehicle_relative_heading (obj_rel_deg)
                        logging.getLogger(__name__).info(f"Object Vehicle Relative (vehicle centric): {obj_veh_relative_heading}")

                        logging.getLogger(__name__).info(f"Angles: {angles[obj_id][0]}")

                        est_x, est_y = self.get_object_x_y(heading, x, y, obj_dist=obj_est_dist, obj_degrees = obj_rel_deg)
                        logging.getLogger(__name__).info(f"Object estimated position: {(est_x, est_y)}")

                        obj_est_dist_mm = self.__in_to_mm(obj_est_dist)

                        # wait as long as necessary to allow lidar to update
                        lidar_dist = -1.0
                        if self.__lidar_enabled_search:
                            lidar_angle, lidar_dist = self.get_lidar_measurement(
                                degrees = obj_veh_relative_heading,
                                angle_tolerance = self.__lidar_drift_tolerance,
                                expected_distance_mm = obj_est_dist_mm,
                                distance_tolerance=obj_est_dist_mm * self.__lidar_visual_variance_pct,
                                max_age_millis = int(self.__lidar_max_age * 1000)
                            )

                        self.get_pilot_logger().log_search_hit (
                            map_id = self.get_pilot_nav().get_map_id(), 
                            object_type = obj_type,
                            est_visual_distance = obj_est_dist,
                            est_lidar_dist = lidar_dist, 
                            vehicle_relative_heading = obj_veh_relative_heading,
                            est_x = est_x,
                            est_y = est_y,
                            vehicle_x = x,
                            vehicle_y = y,
                            vehicle_heading = heading, 
                            confidence = confidence,
                            camera_id = cam,
                            camera_angle = self.__get_vehicle_relative_heading(self.get_pilot_nav().get_camera_heading(cam)),
                            image_file = f"{self.get_pilot_config()['CacheLocations']['Images']}/search_cam_{cam}.png",
                            image_format = 'png'
                        )
            return True # search is successful, even if we didnt find anything
        return False

    def __in_to_mm (self, measurement):
        return measurement * 25.4
    
    def __mm_to_in (self, measurement):
        return measurement / 25.4

    def __get_vehicle_relative_heading (self, cam_oriented_heading):
        new_heading = cam_oriented_heading - 90
        if new_heading > 180:
            new_heading = -1 * (180 - (new_heading - 180))

        return new_heading

    # give a vehicle centric measurement (-90 is left, 90 is right, etc), get back lidar-centric (0/360 is front , 180 is back)
    def __get_lidar_relative_heading (self, vehicle_oriented_heading):
        new_heading = vehicle_oriented_heading
        if new_heading < 0:
            new_heading = 360 + new_heading

        return new_heading

    def get_lidar_measurement (self, degrees : float, angle_tolerance : float, expected_distance_mm : float, distance_tolerance : float, max_age_millis : int):
        lidar_heading = self.__get_lidar_relative_heading(degrees)
        logging.getLogger(__name__).info(f"Getting lidar measurement for vehicle relative heading: {degrees} (lidar heading: {lidar_heading})")

        found_angle, found_measurement = self.get_vehicle().find_measurement (
            degrees = lidar_heading, 
            angle_tolerance = angle_tolerance,
            expected_distance  = expected_distance_mm,
            distance_tolerance = distance_tolerance, 
            max_age = max_age_millis,
            timeout = 10)
        
        return found_angle, found_measurement

    def get_object_x_y (self, heading, x, y, obj_dist, obj_degrees):
        # rotate degrees so zero is east and 180 is west
        #x = r X cos( θ )
        #y = r X sin( θ )
        cartesian_angle_degrees = 180 - (obj_degrees - heading)
        if cartesian_angle_degrees < 0:
            cartesian_angle_degrees += 360

        logging.getLogger(__name__).info(f"Finding object position using vehicle heading: {heading}, x: {x}, y:{y}, cartesian coord angle: {cartesian_angle_degrees}")

        est_x = x + obj_dist * math.cos(math.radians(cartesian_angle_degrees))
        est_y = y + obj_dist * math.sin(math.radians(cartesian_angle_degrees))
        return est_x, est_y
