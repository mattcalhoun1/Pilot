from pilot.pilot_navigation import PilotNavigation
from pilot.pilot_logger import PilotLogger
from camera.camera_info import CameraInfo
from pilot.actions.action_base import ActionBase
import logging
import time
import math

class SearchAction(ActionBase):
    def __init__(self, vehicle, pilot_nav : PilotNavigation, pilot_logger : PilotLogger, pilot_config, pilot) :
        self.__vehicle = vehicle
        self.__pilot = pilot
        self.__pilot_nav = pilot_nav
        self.__pilot_logger = pilot_logger
        self.__pilot_config = pilot_config
        self.__lidar_enabled_search = pilot_config['LidarEnabledSearch']

        self.__camera_horz_fov = [
            CameraInfo.get_fov_horizontal(self.__pilot_config['Cameras']['Left']['Config']),
            CameraInfo.get_fov_horizontal(self.__pilot_config['Cameras']['Right']['Config'])
        ]

    def get_name (self):
        return "Search"

    def execute (self, params):
        # should include either single object or multiple objects to search for
        if 'object' in params:
            return self.search([params['object'],]
                               )
        return self.search(params['objects'])

    def search (self, objects, start_heading = 1.0, end_heading = -1.0):
        if self.__vehicle.wait_for_ready() and self.__vehicle.get_all_configurations():
            logging.getLogger(__name__).info(f"Searching for {objects}")
            rotation_amount = 0

            last_move_time = time.time() # so we know how long lidar has been collecting at this orientation
            lidar_max_age = 5.0

            # get the camera starting positions so we can set them back when done
            self.__pilot_nav.read_and_update_camera_positions(self.__vehicle)
            cam_left_orig = self.__get_vehicle_relative_heading(self.__pilot_nav.get_camera_heading('Left'))
            cam_right_orig = self.__get_vehicle_relative_heading(self.__pilot_nav.get_camera_heading('Right'))
            
            # assuming 2 cameras on vehicle can simultaneously look in 90 and -90 vehicle relative headings
            self.__vehicle.look_multi([(90, 90),(-90, 90)], wait_for_result=True)
            self.__pilot_nav.read_and_update_camera_positions(self.__vehicle)
            rotation_amount = 0

            while rotation_amount <= 0:#180.0:
                located_objects = self.__pilot_nav.locate_objects(objects=objects)
                if len(located_objects) > 0:
                    logging.getLogger(__name__).info(f"Located {len(located_objects)} Objects In This Space!")

                    # determine actual position/heading so we can record good info about this
                    x, y, heading, confidence, coord_age = self.__pilot_nav.get_last_coords_and_heading()
                    if rotation_amount > 0: # if we have rotated since last position read, get fresh readings
                        x, y, heading, confidence = self.__pilot_nav.get_coords_and_heading() 

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
                                lidar_age = time.time() - last_move_time
                                if lidar_age < lidar_max_age:
                                    logging.getLogger(__name__).info(f"Allowing lidar to collect measurments")
                                    time.sleep(lidar_max_age - lidar_age)

                                lidar_angle, lidar_dist = self.get_lidar_measurement(
                                    degrees = obj_veh_relative_heading,
                                    angle_tolerance = 1.0,
                                    expected_distance_mm = obj_est_dist_mm,
                                    distance_tolerance=obj_est_dist_mm * 0.2,
                                    max_age_millis = int(lidar_max_age * 1000)
                                )

                            self.__pilot_logger.log_search_hit (
                                map_id = self.__pilot_nav.get_map_id(), 
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
                                camera_angle = self.__get_vehicle_relative_heading(self.__pilot_nav.get_camera_heading(cam)),
                                image_file = f"{self.__pilot_config['CacheLocations']['Images']}/search_cam_{cam}.png",
                                image_format = 'png'
                            )


                # Rotate the smallest fov of the two cameras, so we capture the entire 360 view
                self.__vehicle.rotate(degrees=min(self.__camera_horz_fov[0],self.__camera_horz_fov[1]), wait_for_result = True)
                self.__pilot_nav.invalidate_position()

                rotation_amount = rotation_amount + self.__camera_horz_fov[0]
                last_move_time = time.time()

            # set cameras back to how they were before the search
            logging.getLogger(__name__).info(f"Returning cameras to original positions of Left: {cam_left_orig}, Right: {cam_right_orig}")
            self.__vehicle.look_multi([(cam_right_orig, 90),(cam_left_orig, 90)], wait_for_result=True)


            return True
        
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
        logging.getLogger(__name__).info(f"Getting lidar measuremnt for vehicle relative heading: {degrees} (lidar heading: {lidar_heading})")

        found_angle, found_measurement = self.__vehicle.find_measurement (
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
