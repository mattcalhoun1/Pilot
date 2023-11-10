from landmarks.emitter_landmark_finder import EmitterLandmarkFinder
from landmarks.basic_landmark_finder import BasicLandmarkFinder
from landmarks.object_search_finder import ObjectSearchFinder
from position.position_estimator_with_clustering import PositionEstimatorWithClustering
from position.confidence import Confidence
from position.estimator_mode import EstimatorMode
from landmarks.landmark_labeler import LandmarkLabeler
from landmarks.object_search_labeler import ObjectSearchLabeler
import time
import logging
import statistics
from camera.camera_info import CameraInfo
from camera.camera_manager import CameraManager
from recognition.tflite_object_locator import TFLiteObjectLocator
from field.field_map import FieldMap
from navsvc.nav_service import NavService
from pilot.pilot_resources import PilotResources
from pilot.pilot_logger import PilotLogger
import json
import copy
import os

class PilotNavigation:
    def __init__ (self, pilot_resources : PilotResources, pilot_logger : PilotLogger, map_id : str, field_map : FieldMap, starting_altitude : float, vehicle):
        self.__resources = pilot_resources
        self.__pilot_logger = pilot_logger
        self.__map_id = map_id
        self.__field_map = field_map
        self.__config = pilot_resources.get_config()
        self.__camera_manager = CameraManager()
        self.__enabled_cameras = {}
        self.__camera_headings = {}
        self.__last_location_update = time.time()
        self.__curr_altitude = starting_altitude
        self.__vehicle = vehicle

        self.__lidar_enabled_positioning = self.__config['LidarEnabledPositioning']
        self.__smoothing_cycles_per_image = self.__config['SmoothingCyclesPerImage']
        self.__lidar_map = None
        self.__lidar_time = 0
        self.__lidar_max_age = 600 # max seconds old lidar data can be. As long as the vehicle doesn't move the lidar should be good indefinitely, as long as objects don't move around it

        self.__locator = TFLiteObjectLocator(model_configs = pilot_resources.get_model_configs())
        self.__estimator_mode = PilotNavigation.__get_estimator_mode(self.__config['EstimatorMode'])
        self.__default_preferred_num_landmarks = 3
        if self.__estimator_mode == EstimatorMode.FAST:
            self.__default_preferred_num_landmarks = 2
        elif self.__estimator_mode == EstimatorMode.VERY_PRECISE:
            self.__default_preferred_num_landmarks = 4
                                                                     

        self.__delay_between_location_attempts = .2 # how long to wait between location attemps
        self.__max_location_attempts = 2 # how many times to try retrieve location before skipping
        self.__min_location_success = 3 # how many locations to receive before considering the coordinates valid
        self.__min_location_confidence = Confidence.CONFIDENCE_LOW # minimum location confidence before it can be considered valid
        self.__min_object_confidence = 0.25 if 'MinObjectConfidence' not in self.__config else self.__config['MinObjectConfidence']
        
        self.__save_images = self.__config['SavePositioningImages']
        self.__save_empty_images = self.__config['SaveEmptyPositioningImages']

        self.__newest_images = {} # one per camera, these are just the image locations

        # keep copy of last known coords
        self.__last_x = None
        self.__last_y = None
        self.__last_heading = None
        self.__last_confidence = None
        self.__last_coord_time = None
        
        self.__alt_camera_positions = {}
        self.__default_camera_positions = []
        for c in self.__config['Cameras']:
            if self.__config['Cameras'][c]['Enabled'] == True:
                self.__enabled_cameras[c] = self.__config['Cameras'][c]['Config']
                self.__camera_headings[c] = self.__config['Cameras'][c]['DefaultHeading']

                # these are vehicle-centric values, for controlling servo positions, they need to be 0-front, rather than 90-front
                self.__default_camera_positions.append(self.__config['Cameras'][c]['DefaultHeading'] - 90)
                self.__alt_camera_positions[c] = []
                if 'AlternateHeadings' in self.__config['Cameras'][c]:
                    for pos in self.__config['Cameras'][c]['AlternateHeadings']:
                        self.__alt_camera_positions[c].append(pos - 90)

        self.__finders = {} # for navigation only
        self.__obj_search_finders = {} # for search
        for c in self.__enabled_cameras:
            # for each model in the map, add a finder
            self.__finders[c] = []
            self.__obj_search_finders[c] = []
            model_configs = self.__resources.get_model_configs()
            for m in model_configs:
                if model_configs[m]['LandmarkType'] == 'emitter':
                    self.__finders[c].append(EmitterLandmarkFinder(
                        camera_config=CameraInfo.CameraConfig[self.__enabled_cameras[c]],
                        field_map=field_map
                    ))

                elif model_configs[m]['LandmarkType'] == 'basic':
                    self.__finders[c].append(BasicLandmarkFinder(
                        camera_config=CameraInfo.CameraConfig[self.__enabled_cameras[c]],
                        field_map=field_map,
                        model_name = m
                    ))
                    self.__obj_search_finders[c].append(ObjectSearchFinder(
                        camera_config=CameraInfo.CameraConfig[self.__enabled_cameras[c]],
                        field_map=field_map,
                        model_name = m
                    ))

        self.__position_est = {}
        for c in self.__enabled_cameras:
            self.__position_est[c] = PositionEstimatorWithClustering(
                field_map = field_map,
                horizontal_fov = CameraInfo.get_fov_horizontal(config_id=self.__enabled_cameras[c]),
                vertical_fov=CameraInfo.get_fov_vertical(config_id=self.__enabled_cameras[c]),
                view_width=CameraInfo.get_resolution_width(config_id=self.__enabled_cameras[c]),
                view_height=CameraInfo.get_resolution_height(config_id=self.__enabled_cameras[c]),
                base_front=90.0,
                use_multithreading=self.__config['MultithreadedPositioning'],
                estimator_mode = PilotNavigation.__get_estimator_mode(self.__estimator_mode),
                max_lidar_drift_deg = self.__config['LidarMaxDriftDegrees'],
                max_lidar_visual_variance_pct = self.__config['LidarMaxVisualDistVariancePct']
            )        

    def __get_estimator_mode (estimator_mode_str):
        if estimator_mode_str.lower() == "fast":
            return EstimatorMode.FAST
        elif estimator_mode_str.lower() == "very_precise":
            return EstimatorMode.VERY_PRECISE
        else:
            return EstimatorMode.PRECISE
            
    def __get_camera (self, c):
        return self.__camera_manager.get_camera(c, camera_config=self.__enabled_cameras[c], auto_optimize_object_locator=self.__locator)
    
    def get_map_id (self):
        return self.__map_id

    def get_field_map (self):
        return self.__field_map

    def get_camera_heading (self, c):
        return self.__camera_headings[c]

    # returns max value for alternative camera headings
    def __get_num_alt_camera_headings (self):
        max_headings = 0
        for c in self.__alt_camera_positions:
            if len(self.__alt_camera_positions[c]) > max_headings:
                max_headings = len(self.__alt_camera_positions[c])
        return max_headings

    # repositions up to 2 cameras to default or alternate rotations
    def reposition_cameras (self, alt_heading_num = None):
        rotation_1 = None
        rotation_2 = None
        if alt_heading_num is None:
            for rotation in self.__default_camera_positions:
                if rotation_1 is None:
                    rotation_1 = rotation
                elif rotation_2 is None:
                    rotation_2 = rotation
        else:
            for c in self.__alt_camera_positions:
                if len(self.__alt_camera_positions[c]) > alt_heading_num:
                    if rotation_1 is None:
                        rotation_1 = self.__alt_camera_positions[c][alt_heading_num]
                    elif rotation_2 is None:
                        rotation_2 = self.__alt_camera_positions[c][alt_heading_num]
        if rotation_1 is not None and rotation_2 is not None:
            self.__vehicle.look_multi([(rotation_1, 90),(rotation_2, 90)], wait_for_result=True)
        elif rotation_1 is not None:
            self.__vehicle.look(rotation = rotation_1, tilt = 90, wait_for_result=True)

        return self.read_and_update_camera_positions(self.__vehicle)

    def read_and_update_camera_positions (self, vehicle):
        cameras = vehicle.get_cameras (timeout = 10.0, force_refresh = True)
        if cameras is not None and len(cameras) > 0:
            for c in cameras:
                self.__update_camera_heading(vehicle_cam_id=int(c), relative_heading = float(cameras[c]['rotation']))
            return True
        else:
            logging.getLogger(__name__).error("Unable to update camera position readings from vehicle!")
        return False

    def __update_camera_heading(self, vehicle_cam_id, relative_heading):
        for cam_config in self.__config['Cameras']:
            if self.__config['Cameras'][cam_config]['VehicleBaseId'] == int(vehicle_cam_id):
                # get the actual heading
                # for purposes of navigation, 90 is considered the front of the vehicle, not 0.
                self.__camera_headings[cam_config] = 90 + relative_heading
                logging.getLogger(__name__).debug(f"Camera {vehicle_cam_id} / {cam_config} New Heading: {self.__camera_headings[cam_config]}, based on vehicle-reported: {relative_heading}")
                return True
        logging.getLogger(__name__).warning(f"Unable to match vehicle camera: {vehicle_cam_id} to any camera config. Headings may be off!!")
        return False

    def get_altitude (self):
        return self.__curr_altitude
    
    def locate_landmarks (self, display_on_vehicle = False):
        located_landmarks = {}
        for c in self.__enabled_cameras:
            try:
                # go through location cycle multiple times so smoothing can work
                c_located_objects = None
                for locate_cycle in range(self.__smoothing_cycles_per_image):
                    c_located_objects = self.__locator.find_objects_on_camera(camera=self.__get_camera(c), min_confidence = self.__min_object_confidence)
                    logging.getLogger(__name__).debug(f"Camera {c} found {len(c_located_objects)} objects: {c_located_objects}")
                    located_landmarks[c] = {}
                    for f in self.__finders[c]:
                        f_located = f.locate_landmarks(object_locations=c_located_objects)
                        #logging.getLogger(__name__).info(f"Camera {c} Landmarks: {f_located}")
                        for lid in f_located:
                            located_landmarks[c][lid] = f_located[lid]
                        #located_landmarks[c] = located_landmarks[c] + f.locate_landmarks(object_locations=c_located_objects)#, id_filter=['light'])
            
                angles = None
                distances = None
                if self.__save_images or display_on_vehicle:
                    angles = self.__position_est[c].extract_object_view_angles(
                        located_objects = self.__format_landmarks_for_position (located_objects = located_landmarks[c], camera_heading = self.__camera_headings[c]), 
                        add_relative_angles = True)
                    distances = self.__position_est[c].extract_distances (view_angles=angles, view_altitude=self.get_altitude())

                if display_on_vehicle:
                    self.__vehicle.display_objects (obj_dist_map = distances, wait_for_result = True)

                if self.__save_images:
                    # save any images where landmarks were spotted, or all images if configured
                    if len(located_landmarks[c]) > 0 or self.__save_empty_images:
                        # save the image with bound boxes for inspection. the distance calculation can be slow!
                        latest_img = self.__locator.get_latest_image()
                        image_file = f"{self.__config['CacheLocations']['Images']}/coordinates_cam_{c}_{round(self.__camera_headings[c])}.png"
                        LandmarkLabeler().export_labeled_image(
                            image = latest_img,
                            landmarks=located_landmarks[c],
                            distances=distances,
                            angles=angles,
                            file_name=image_file)
                        self.__newest_images[f"{c}_{round(self.__camera_headings[c])}"] = image_file
            except Exception as e:
                logging.getLogger(__name__).warning(f"Locate landmarks failed. Possibly camera glitch: {e}")
        
        return located_landmarks

    def locate_objects (self, objects, display_on_vehicle = False):
        combined_located_objects = {}

        all_objects = {}

        for c in self.__enabled_cameras:
            try:
                c_located_objects = self.__locator.find_objects_on_camera(camera=self.__get_camera(c), min_confidence = self.__min_object_confidence)
                #logging.getLogger(__name__).info(f"Camera {c} found {len(c_located_objects)} objects: {c_located_objects}")
                combined_located_objects[c] = {}
                for f in self.__obj_search_finders[c]:
                    f_located = f.locate_landmarks(object_locations=c_located_objects, id_filter=objects)
                    #logging.getLogger(__name__).info(f"Camera {c} Landmarks: {f_located}")
                    for lid in f_located:
                        combined_located_objects[c][lid] = f_located[lid]
                    #located_landmarks[c] = located_landmarks[c] + f.locate_landmarks(object_locations=c_located_objects)#, id_filter=['light'])
            

                angles = None
                distances = None
                if self.__save_images or display_on_vehicle:
                    angles = self.__position_est[c].extract_object_view_angles(
                        located_objects = self.__format_landmarks_for_position (located_objects = combined_located_objects[c], camera_heading = self.__camera_headings[c]), 
                        add_relative_angles = True)
                    distances = self.__position_est[c].extract_distances (view_angles=angles, view_altitude=self.get_altitude(), filter_unmapped_objects = False)

                if display_on_vehicle:
                    self.__vehicle.display_objects (obj_dist_map = distances, wait_for_result = True)

                if self.__save_images:
                    all_objects[c] = {
                        'angles':angles,
                        'distances':distances
                    }

                    # save the image with bound boxes for inspection
                    latest_img = self.__locator.get_latest_image()
                    image_file = f"{self.__config['CacheLocations']['Images']}/search_cam_{c}.png"
                    ObjectSearchLabeler().export_labeled_image(
                        image = latest_img,
                        landmarks=combined_located_objects[c],
                        distances=distances,
                        angles=angles,
                        file_name=image_file)
            except Exception as e:
                logging.getLogger(__name__).warning(f"Locate objects failed. Possibly camera glitch: {e}")
        
        return all_objects
    
    def invalidate_position (self):
        self.__lidar_map = None

    def __get_lidar_map (self):
        if self.__lidar_enabled_positioning and (self.__lidar_map is None or time.time() - self.__lidar_time > self.__lidar_max_age):
            self.__lidar_map = self.__vehicle.get_live_lidar_map(timeout=15.0)
            self.__lidar_time = time.time()
        return self.__lidar_map

    def get_coords_and_heading (self, min_num_landmarks = 2, allow_camera_reposition = True, cam_start_default_position = True, display_landmarks_on_vehicle= False):
        if cam_start_default_position:
            # ensure cameras are at correct heading
            self.reposition_cameras()

        num_repositions_allowed = self.__get_num_alt_camera_headings() if allow_camera_reposition else 0

        successes = 0
        attempts = 0

        x = None
        y = None
        heading = None
        confidence = Confidence.CONFIDENCE_LOW
        basis = None
        
        while confidence < Confidence.CONFIDENCE_MEDIUM and successes < self.__min_location_success and attempts < self.__max_location_attempts:
            if attempts > 0:
                time.sleep(self.__delay_between_location_attempts)

            attempts += 1

            num_repositions_used = 0
            unique_landmark_ids = []
            combined_landmarks = []

            while confidence < Confidence.CONFIDENCE_MEDIUM and successes < self.__min_location_success and num_repositions_used <= num_repositions_allowed and len(unique_landmark_ids) < self.__default_preferred_num_landmarks:
                landmarks = self.locate_landmarks(display_on_vehicle=display_landmarks_on_vehicle)
                
                for c in landmarks:
                    camera_heading = self.get_camera_heading(c)
                    newly_found_landmarks = {}
                    #logging.getLogger(__name__).info(self.__format_landmarks_for_position(landmarks, camera_heading))
                    for lid in landmarks[c]:
                        if lid not in unique_landmark_ids:
                            unique_landmark_ids.append(lid)
                            l = landmarks[c][lid]
                            logging.getLogger(__name__).debug(f"Cam {c} (@{camera_heading}) Located: {lid} @ {l['x1'],l['y1']} - {l['x2'],l['y2']}")
                            newly_found_landmarks[lid] = l
                        
                    if len(newly_found_landmarks) > 0:
                        combined_landmarks = combined_landmarks + self.__format_landmarks_for_position(located_objects=newly_found_landmarks,camera_heading=camera_heading)
                
                # if we prefer to have more and there are repositions left, do that now
                if len(unique_landmark_ids) < self.__default_preferred_num_landmarks and num_repositions_used < num_repositions_allowed:
                    logging.getLogger(__name__).info(f"Prefer more landmarks, only ({len(unique_landmark_ids)}) found. Repositioning to find more.")
                    logging.getLogger(__name__).info(f"Combined Landmarks: {combined_landmarks}")
                    # rotate the cameras to the next position
                    self.reposition_cameras(num_repositions_used)
                    num_repositions_used += 1
                elif len(combined_landmarks) >= min_num_landmarks:
                    logging.getLogger(__name__).info(f"Combined Landmarks: {combined_landmarks}")
                    x, y, heading, confidence, basis = self.get_coords_and_heading_for_landmarks (combined_landmarks=combined_landmarks, allow_lidar = True)
                    logging.getLogger(__name__).info(f"=== Coords: ({x} , {y})  Heading: {heading}, Confidence: {confidence}, Successes: {successes} ===")
                    if x is not None and y is not None and heading is not None and confidence is not None and confidence >= self.__min_location_confidence:
                        # display on vehicle, if configured
                        self.__vehicle.display_position(x=x, y=y, heading=heading, wait_for_result = True) # if we dont wait for result, the repositino can fail

                        successes += 1
                else:
                    num_repositions_used += 1

            # if we had to move the cameras, set them back to their default location
            if num_repositions_used > 0:
                self.reposition_cameras()
        
        if successes >= self.__min_location_success or confidence >= Confidence.CONFIDENCE_MEDIUM:
            if self.__pilot_logger is not None and x is not None and y is not None and heading is not None:
                if self.__save_images:
                    views = []
                    for c in self.__newest_images:
                        image_location = self.__newest_images[c]
                        views.append({
                            'image_file':image_location,
                            'image_format':'png',
                            'camera_id':c.split('_')[0],
                            'camera_heading':c.split('_')[1]
                        })
                    self.__pilot_logger.log_coordinates_and_images(map_id = self.__map_id, x = x, y = y, heading = heading, images=views, basis=basis)
                else:
                    self.__pilot_logger.log_coordinates(map_id = self.__map_id, x = x, y = y, heading = heading, basis=basis)
            
            self.__last_x = x
            self.__last_y = y
            self.__last_heading = heading
            self.__last_confidence = confidence
            self.__last_coord_time = time.time()

            # clear image buffer
            self.__newest_images = {}

            # display on vehicle, if configured
            if x is not None and y is not None and heading is not None:
                self.__vehicle.display_position(x=x, y=y, heading=heading)

            return x,y,heading,confidence

        # clear image buffer
        self.__newest_images = {}

        # not enough successes to report a location
        return None,None,None,None

    def __select_top_landmarks (self, combined_landmarks, max_landmarks = 3):
        if len(combined_landmarks) <= max_landmarks:
            return combined_landmarks
        
        top_landmarks = []
        found_priorities = []
        prioritized_map = {}

        for lm in combined_landmarks:
            for lid in lm:
                this_priority = 0
                if 'priority' in lm[lid]:
                    this_priority = lm[lid]['priority']
                else:
                    this_priority = self.__field_map.get_landmark_priority(lid)

                found_priorities.append(this_priority)
                prioritized_map[this_priority] = lm
        
        found_priorities.sort(reverse=True)
        for priority in found_priorities[:max_landmarks]:
            top_landmarks.append(prioritized_map[priority])
        
        return top_landmarks


    def get_coords_and_heading_for_landmarks (self, combined_landmarks, allow_lidar = True, lidar_map = None):
        if allow_lidar and lidar_map is None:
            lidar_map = self.__get_lidar_map()
        else:
            # if we were given a lidar map and told not to use it, null the reference
            lidar_map = None

        #logging.getLogger(__name__).info(f"*** Unfiltered Landmarks: {combined_landmarks}")
        filtered_landmarks = self.__select_top_landmarks (combined_landmarks, max_landmarks = 3)
        #logging.getLogger(__name__).info(f"*** Filtered Landmarks: {filtered_landmarks}")

        x, y, heading, confidence, basis = self.__position_est[[*self.__position_est.keys()][0]].get_coords_and_heading(
            located_objects=filtered_landmarks,
            view_altitude=self.get_altitude(), # on the tank, on the floor
            lidar_map=lidar_map
        )
        return x, y, heading, confidence, basis
    
    def get_last_coords_and_heading (self):
        return self.__last_x, self.__last_y, self.__last_heading, self.__last_confidence, self.__last_coord_time
        

    def cleanup (self):
        logging.getLogger(__name__).debug("Cleaning up resources")
        self.__camera_manager.cleanup_cameras()

    def __format_landmarks_for_position (self, located_objects, camera_heading):
        as_list = []
        for lid in located_objects:
            this_obj = copy.deepcopy(located_objects[lid])
            this_obj['camera_heading'] = camera_heading
            this_obj['priority'] = self.__field_map.get_landmark_priority(lid)
            as_list.append({lid:this_obj})
        
        return as_list
