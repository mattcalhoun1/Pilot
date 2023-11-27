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
from pilot.navigation_thread_manager import NavigationThreadManager
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
        self.__curr_altitude = starting_altitude
        self.__vehicle = vehicle

        self.__lidar_enabled_positioning = self.__config['Lidar']['EnabledPositioning']
        self.__smoothing_cycles_per_image = self.__config['Landmarks']['Detection']['SmoothingCyclesPerImage']
        self.__lidar_map = None
        self.__lidar_time = 0
        self.__lidar_max_age = self.__config['Lidar']['MaxAge'] # max seconds old lidar data can be. As long as the vehicle doesn't move the lidar should be good indefinitely, as long as objects don't move around it

        self.__locator = TFLiteObjectLocator(model_configs = pilot_resources.get_model_configs())
        self.__estimator_mode = PilotNavigation.__get_estimator_mode(self.__config['Positioning']['EstimatorMode'])

        self.__delay_between_location_attempts = self.__config['Positioning']['PositionRetryDelaySeconds'] # how long to wait between location attemps

        # max number of tries to receive a preferred confidence location
        self.__max_position_attempts = self.__config['Positioning']['MaxTargetConfidenceAttempts']

         # minimum location confidence before it can be considered valid
        self.__min_position_confidence = PilotNavigation.__get_confidence(self.__config['Positioning']['MinPositionConfidence'])

         # minimum location confidence before it can be considered valid
        self.__preferred_position_confidence = PilotNavigation.__get_confidence(self.__config['Positioning']['PreferredPositionConfidence'])

        self.__min_object_confidence = 0.25 if 'MinObjectConfidence' not in self.__config['Landmarks']['Detection'] else self.__config['Landmarks']['Detection']['MinObjectConfidence']
        self.__max_positioning_landmarks = self.__config['Landmarks']['Maximum']
        
        self.__save_images = self.__config['Landmarks']['Detection']['SavePositioningImages']
        self.__save_empty_images = self.__config['Landmarks']['Detection']['SaveEmptyPositioningImages']
        self.__multithreaded_positioning = self.__config['Landmarks']['Detection']['MultithreadedPositioning']

        self.__save_search_images = self.__config['Landmarks']['Detection']['SaveSearchImages']
        self.__multithreaded_search = self.__config['Landmarks']['Detection']['MultithreadedSearch']

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
                use_multithreading=self.__config['Positioning']['Multithreaded'],
                estimator_mode = PilotNavigation.__get_estimator_mode(self.__estimator_mode),
                max_lidar_drift_deg = self.__config['Lidar']['MaxDriftDegrees'],
                max_lidar_visual_variance_pct = self.__config['Lidar']['MaxVisualDistVariancePct'],
                adjust_for_altitude=self.__config['Positioning']['AdjustForAltitude']
            )        

    def __get_estimator_mode (estimator_mode_str):
        if estimator_mode_str.lower() == "fast":
            return EstimatorMode.FAST
        elif estimator_mode_str.lower() == "very_precise":
            return EstimatorMode.VERY_PRECISE
        else:
            return EstimatorMode.PRECISE
        
    def __get_confidence (confidence_str):
        if confidence_str.lower() == 'medium':
            return Confidence.CONFIDENCE_MEDIUM
        if confidence_str.lower() == 'low':
            return Confidence.CONFIDENCE_LOW
        if confidence_str.lower() == 'high':
            return Confidence.CONFIDENCE_HIGH
        
        return Confidence.CONFIDENCE_FACT
            
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
    
    def __prepare_for_multiprocessing (self):
        # dump any potentially troubling items from the object graph so this object can be pickled
        self.__locator.unload_models()

    # this locates landmarks on a single camera in a threadsafe way, so multiple
    # cameras can be 'queried' simultaneously
    def locate_landmarks_on_camera (self, camera_id, latest_image_files = None):
        located_landmarks = {}
        try:
            # go through location cycle multiple times so smoothing can work
            c_located_objects = None
            for locate_cycle in range(self.__smoothing_cycles_per_image):
                if latest_image_files is None:
                    c_located_objects = self.__locator.find_objects_on_camera(camera=self.__get_camera(camera_id), min_confidence = self.__min_object_confidence)
                else:
                    c_located_objects = self.__locator.find_objects_in_image_file(image_file = latest_image_files[locate_cycle], min_confidence = self.__min_object_confidence)
                logging.getLogger(__name__).info(f"Camera {camera_id} found {len(c_located_objects)} objects: {c_located_objects}")
                located_landmarks[camera_id] = {}
                for f in self.__finders[camera_id]:
                    f_located = f.locate_landmarks(object_locations=c_located_objects)
                    #logging.getLogger(__name__).info(f"Camera {c} Landmarks: {f_located}")
                    for lid in f_located:
                        located_landmarks[camera_id][lid] = f_located[lid]
                    #located_landmarks[c] = located_landmarks[c] + f.locate_landmarks(object_locations=c_located_objects)#, id_filter=['light'])
        
            angles = None
            distances = None
            if self.__save_images:
                angles = self.__position_est[camera_id].extract_object_view_angles(
                    located_objects = self.__format_landmarks_for_position (located_objects = located_landmarks[camera_id], camera_heading = self.__camera_headings[camera_id]), 
                    add_relative_angles = True)
                distances = self.__position_est[camera_id].extract_distances (view_angles=angles, view_altitude=self.get_altitude())

                # save any images where landmarks were spotted, or all images if configured
                if len(located_landmarks[camera_id]) > 0 or self.__save_empty_images:
                    # save the image with bound boxes for inspection. the distance calculation can be slow!
                    latest_img = self.__locator.get_latest_image()
                    image_file = f"{self.__config['CacheLocations']['Images']}/coordinates_cam_{camera_id}_{round(self.__camera_headings[camera_id])}.png"
                    LandmarkLabeler().export_labeled_image(
                        image = latest_img,
                        landmarks=located_landmarks[camera_id],
                        distances=distances,
                        angles=angles,
                        file_name=image_file)
                    self.__newest_images[f"{camera_id}_{round(self.__camera_headings[camera_id])}"] = image_file
        except Exception as e:
            logging.getLogger(__name__).warning(f"Locate landmarks failed. Possibly camera glitch: {e}")
        
        return located_landmarks

    def locate_landmarks (self):
        consolidated_landmarks = {} # keyed by camera id
        camera_searches = [] # for multithreading
        for c in self.__enabled_cameras:
            # Get each camera search going in separate threads
            if self.__multithreaded_positioning:
                # capture as many images as necessary for smoothing
                images = []
                for i in range(self.__smoothing_cycles_per_image):
                    file_name = f'/tmp/{c}_processing_{i}.npy'
                    img = self.__get_camera(c).capture_image (preprocess = True, file_name = file_name)
                    if img is not None:
                        images.append(file_name)
                if len(images) >= self.__smoothing_cycles_per_image:
                    camera_searches.append((
                        self,
                        c,
                        images
                    ))
            else:
                camera_results = self.locate_landmarks_on_camera(c)
                if c in camera_results:
                    consolidated_landmarks[c] = camera_results[c]

        # consolidate results into a dictionary keyed by camera id
        # if multithreading, we need to wait for results to come in
        if self.__multithreaded_positioning:
            self.__prepare_for_multiprocessing()
            pool = NavigationThreadManager.get_thread_pool()
            async_results = pool.starmap_async(external_locate_landmarks, camera_searches)
            try:
                for thread_result in async_results.get():
                    for cid in thread_result:
                        consolidated_landmarks[cid] = thread_result[cid]
            except TimeoutError as te:
                # is the thread pool corrupt in this case? Does it have a zombie
                logging.getLogger(__name__).info("Timed out, some or all camera results may not be included")

        return consolidated_landmarks

    def locate_objects_on_camera (self, objects, camera_id, latest_image_files = None):
        combined_located_objects = {}

        all_objects = {}

        try:
            for locate_cycle in range(self.__smoothing_cycles_per_image):
                c_located_objects = None
                if latest_image_files is None:
                    c_located_objects = self.__locator.find_objects_on_camera(camera=self.__get_camera(camera_id), min_confidence = self.__min_object_confidence)
                else:
                    c_located_objects = self.__locator.find_objects_in_image_file(image_file = latest_image_files[locate_cycle], min_confidence = self.__min_object_confidence)

                #logging.getLogger(__name__).info(f"Camera {camera_id} found {len(c_located_objects)} objects: {c_located_objects}")
                combined_located_objects[camera_id] = {}
                for f in self.__obj_search_finders[camera_id]:
                    f_located = f.locate_landmarks(object_locations=c_located_objects, id_filter=objects)
                    #logging.getLogger(__name__).info(f"Camera {camera_id} Landmarks: {f_located}")
                    for lid in f_located:
                        combined_located_objects[camera_id][lid] = f_located[lid]
                    #located_landmarks[c] = located_landmarks[c] + f.locate_landmarks(object_locations=c_located_objects)#, id_filter=['light'])
        
            # if the object was found after applying filtering and smoothing
            if len(combined_located_objects[camera_id]) > 0:
                angles = None
                distances = None
                angles = self.__position_est[camera_id].extract_object_view_angles(
                    located_objects = self.__format_landmarks_for_position (located_objects = combined_located_objects[camera_id], camera_heading = self.__camera_headings[camera_id]), 
                    add_relative_angles = True)
                distances = self.__position_est[camera_id].extract_distances (view_angles=angles, view_altitude=self.get_altitude(), filter_unmapped_objects = False)
                all_objects[camera_id] = {
                    'angles':angles,
                    'distances':distances
                }

                if self.__save_search_images:
                    # save the image with bound boxes for inspection
                    latest_img = self.__locator.get_latest_image()
                    image_file = f"{self.__config['CacheLocations']['Images']}/search_cam_{camera_id}.png"
                    ObjectSearchLabeler().export_labeled_image(
                        image = latest_img,
                        landmarks=combined_located_objects[camera_id],
                        distances=distances,
                        angles=angles,
                        file_name=image_file)
        except Exception as e:
            logging.getLogger(__name__).warning(f"Locate objects failed. Possibly camera glitch: {e}")
        
        return all_objects

    def locate_objects (self, objects):
        all_objects = {}

        camera_searches = [] # for multithreading
        for c in self.__enabled_cameras:
            # Get each camera search going in separate threads
            if self.__multithreaded_search:
                # capture as many images as necessary for smoothing
                images = []
                for i in range(self.__smoothing_cycles_per_image):
                    file_name = f'/tmp/{c}_processing_{i}.npy'
                    img = self.__get_camera(c).capture_image (preprocess = True, file_name = file_name)
                    if img is not None:
                        images.append(file_name)
                if len(images) >= self.__smoothing_cycles_per_image:
                    camera_searches.append((
                        self,
                        objects,
                        images,
                        c
                    ))
            else:
                camera_results = self.locate_objects_on_camera(objects, c)
                if c in camera_results:
                    all_objects[c] = camera_results[c]


        # consolidate results into a dictionary keyed by camera id
        # if multithreading, we need to wait for results to come in
        if self.__multithreaded_search:
            self.__prepare_for_multiprocessing()
            pool = NavigationThreadManager.get_thread_pool()
            async_results = pool.starmap_async(external_locate_objects, camera_searches)
            try:
                for thread_result in async_results.get():
                    for cid in thread_result:
                        all_objects[cid] = thread_result[cid]
            except TimeoutError as te:
                # is the thread pool corrupt in this case? Does it have a zombie
                logging.getLogger(__name__).info("Timed out, some or all camera results may not be included in objectg search")

        return all_objects

    def locate_objects_multi_angles (self, objects):
        # begin at default heading
        self.reposition_cameras()
        found_objects = None

        num_repositions_allowed = self.__get_num_alt_camera_headings()
        num_repositions_used = 0
        found = False

        while found == False and num_repositions_used <= num_repositions_allowed:
            found_objects = self.locate_objects(objects)
            found = len(found_objects) > 0
            if found == False:
                self.reposition_cameras(num_repositions_used)
                num_repositions_used += 1

        return found_objects
    
    def invalidate_position (self):
        self.__lidar_map = None

    def __get_lidar_map (self):
        if self.__lidar_enabled_positioning and (self.__lidar_map is None or time.time() - self.__lidar_time > self.__lidar_max_age):
            self.__lidar_map = self.__vehicle.get_live_lidar_map(timeout=15.0)
            self.__lidar_time = time.time()
        return self.__lidar_map

    def get_coords_and_heading (self, allow_camera_reposition = True, cam_start_default_position = True, preferred_confidence = None):
        if preferred_confidence is None:
            preferred_confidence = self.__preferred_position_confidence

        if cam_start_default_position:
            # ensure cameras are at correct heading
            self.reposition_cameras()

        num_repositions_allowed = self.__get_num_alt_camera_headings() if allow_camera_reposition else 0
        attempts = 0

        x = None
        y = None
        heading = None
        confidence = Confidence.CONFIDENCE_LOW
        basis = None
        
        # keep looping until we get coordinates with target confidence or we hit max number of retries (in which case, we go with minimum confidence)
        while confidence < preferred_confidence and attempts < self.__max_position_attempts:
            if attempts > 0:
                time.sleep(self.__delay_between_location_attempts)

            attempts += 1

            num_repositions_used = 0
            unique_landmark_ids = []
            combined_landmarks = []
            landmark_preferences_met = False # do we have a great set of sightings to position
            landmark_requirements_met = False # do we have a good enough set of sightings to position

            while confidence < preferred_confidence and num_repositions_used <= num_repositions_allowed and landmark_preferences_met == False:
                landmarks = self.locate_landmarks()
                
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
                
                # get look at which tiers of landmarks we have
                tiered_landmarks = self.__get_landmarks_by_tier(combined_landmarks=combined_landmarks)
                landmark_preferences_met = self.__are_landmark_preferences_met(tiered_landmarks=tiered_landmarks, unique_landmarks=combined_landmarks)
                landmark_requirements_met = landmark_requirements_met or self.__are_landmark_requirements_met(tiered_landmarks=tiered_landmarks, unique_landmarks=combined_landmarks)

                # if we prefer to have more and there are repositions left, do that now
                if landmark_preferences_met == False and num_repositions_used < num_repositions_allowed:
                    logging.getLogger(__name__).info(f"Prefer more landmarks, only ({len(unique_landmark_ids)}) found. Repositioning to find more.")
                    logging.getLogger(__name__).info(f"Combined Landmarks: {combined_landmarks}")
                    # rotate the cameras to the next position
                    self.reposition_cameras(num_repositions_used)
                    num_repositions_used += 1
                elif landmark_requirements_met:
                    logging.getLogger(__name__).info(f"Combined Landmarks: {combined_landmarks}")
                    x, y, heading, confidence, basis = self.get_coords_and_heading_for_landmarks (combined_landmarks=combined_landmarks, allow_lidar = True)
                    if x is None and len(combined_landmarks) > self.__config['Landmarks']['Minimum']:
                        # One of the landmarks may be bad. Try trimming the lowest hanging one
                        logging.getLogger(__name__).info(f"Positioning failed, looks like possibly an invalid landmark value. Trimming the lowest one and trying again.")
                        x, y, heading, confidence, basis = self.get_coords_and_heading_for_landmarks (combined_landmarks=combined_landmarks, allow_lidar = True, max_landmarks=self.__max_positioning_landmarks - 1)

                    logging.getLogger(__name__).info(f"=== Coords: ({x} , {y})  Heading: {heading}, Confidence: {confidence} ===")
                    if x is not None and y is not None and heading is not None and confidence is not None and confidence >= self.__min_position_confidence:
                        # display on vehicle, if configured
                        self.__vehicle.display_position(x=x, y=y, heading=heading, wait_for_result = True) # if we dont wait for result, the repositino can fail
                    elif num_repositions_used < num_repositions_allowed:
                        self.reposition_cameras(num_repositions_used)
                        num_repositions_used += 1
                    else:
                        num_repositions_used += 1
                        logging.getLogger(__name__).warning(f"Coordinates failed. Landmarks found, but position is bad, probably a bad landmark")

                else:
                    if num_repositions_used < num_repositions_allowed:
                        self.reposition_cameras(num_repositions_used)
                    num_repositions_used += 1

            # if we had to move the cameras, set them back to their default location
            if num_repositions_used > 0:
                self.reposition_cameras()
        
        if confidence >= self.__min_position_confidence:
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

        # log this failure, if configured
        self.__pilot_logger.log_position_failure(map_id=self.__map_id, basis=basis)

        # positioning failed
        return None,None,None,None

    def __are_landmark_requirements_met (self, tiered_landmarks, unique_landmarks):
        if len(unique_landmarks) < self.__config['Landmarks']['Minimum']:
            return False

        for tier_id in self.__config['Landmarks']['Tiers']:
            this_tier_min = self.__config['Landmarks']['Tiers'][tier_id]['Minimum']
            if tier_id not in tiered_landmarks or len(tiered_landmarks[tier_id]) < this_tier_min:
                return False

        return True

    def __are_landmark_preferences_met (self, tiered_landmarks, unique_landmarks):
        if len(unique_landmarks) < self.__config['Landmarks']['Preferred']:
            return False

        for tier_id in self.__config['Landmarks']['Tiers']:
            this_tier_preferred = self.__config['Landmarks']['Tiers'][tier_id]['Preferred']
            logging.getLogger(__name__).info(f"Landmark Tier {tier_id} prefers: {this_tier_preferred}")
            if tier_id not in tiered_landmarks or len(tiered_landmarks[tier_id]) < this_tier_preferred:
                return False

        return True

    # returns the landmarks grouped by tiers, with only one of each landmark (highest confidence)
    def __get_landmarks_by_tier (self, combined_landmarks):
        tiered_landmarks = {}
        unique_landmarks = {}
        

        # first filter down to one unique instance per landmark
        for lm in combined_landmarks:
            for lid in lm:
                if lid not in unique_landmarks or lm[lid]['confidence'] > unique_landmarks[lid]['confidence']:
                    unique_landmarks[lid] = lm[lid]

        # now group the landmarks by tier
        for lid in unique_landmarks:
            this_tier = "unknown"
            if 'tier' in unique_landmarks[lid]:
                this_tier = unique_landmarks[lid]['tier']
            else:
                this_tier = self.__field_map.get_landmark_tier(lid)

            if this_tier in tiered_landmarks:
                tiered_landmarks[this_tier].append(unique_landmarks[lid])
            else:
                tiered_landmarks[this_tier] = [unique_landmarks[lid],]

        return tiered_landmarks

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
        
        found_priorities.sort() # 1 is top priority, higher numbers mean lower priority
        for priority in found_priorities[:max_landmarks]:
            top_landmarks.append(prioritized_map[priority])
        
        return top_landmarks


    def get_coords_and_heading_for_landmarks (self, combined_landmarks, allow_lidar = True, lidar_map = None, max_landmarks = None):
        landmarks_to_keep = max_landmarks if max_landmarks is not None else self.__max_positioning_landmarks
        if allow_lidar and lidar_map is None:
            lidar_map = self.__get_lidar_map()
        else:
            # if we were given a lidar map and told not to use it, null the reference
            lidar_map = None

        #logging.getLogger(__name__).info(f"*** Unfiltered Landmarks: {combined_landmarks}")
        filtered_landmarks = self.__select_top_landmarks (combined_landmarks, max_landmarks = landmarks_to_keep)
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
            this_obj['tier'] = self.__field_map.get_landmark_tier(lid)
            as_list.append({lid:this_obj})
        
        return as_list

    def __getstate__(self):
        state = self.__dict__.copy()
        #print(state.keys())
        # Don't pickle anything not required to complete object detection in another process
        del state["_PilotNavigation__camera_manager"]
        del state["_PilotNavigation__resources"]
        del state["_PilotNavigation__newest_images"]
        return state
    
def external_locate_landmarks (pilot_nav_inst, camera_id, image_files):
    return pilot_nav_inst.locate_landmarks_on_camera(camera_id, image_files)

def external_locate_objects (pilot_nav_inst, objects, images, camera_id):
    return pilot_nav_inst.locate_objects_on_camera(objects, camera_id, images)

if __name__ == "__main__":
    print("in main")