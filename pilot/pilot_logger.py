from navsvc.nav_service import NavService
from datetime import datetime
import logging
import json

class PilotLogger:
    def __init__(self, config_file):
        self.__load_config(config_file)
        self.__vehicle_id = self.__config['Vehicle']
        
        # for now, session is just current date
        self.__session_id = datetime.now().strftime('%Y-%m-%d')
        self.__nav_svc = None
    
    def log_lidar (self, lidar_map):
        try:
            resp = self.__get_nav_service().log_lidar(
                vehicle_id = self.__vehicle_id,
                session_id = self.__session_id,
                lidar_map = lidar_map
            )
        except Exception as e:
            logging.getLogger(__name__).error(f"Failed to log lidar: {e}")
            return False
        return True
        
    
    def log_coordinates (self, map_id, x, y, heading, basis):
        try:
            resp = self.__get_nav_service().log_position_and_heading(
                vehicle_id = self.__vehicle_id,
                session_id = self.__session_id,
                map_id = map_id,
                x = x,
                y = y,
                heading = heading,
                basis = basis
            )
            return resp
        except Exception as e:
            logging.getLogger(__name__).error(f"Log coordinates failed: {e}")
            return None

    def log_coordinates_and_images (self, map_id, x, y, heading, images, basis):
        # log the coordinates and retrieve the new entry num
        resp = self.log_coordinates(map_id, x, y, heading, basis)
        entry_num = resp['entry_num']
        
        # any images we save are attached to the given entry entry_num
        for i in images:
            resp = self.__get_nav_service().log_camera_view(
                vehicle_id = self.__vehicle_id, 
                session_id = self.__session_id, 
                entry_num = entry_num, 
                camera_id = i['camera_id'], 
                camera_heading = i['camera_heading'], 
                image_file=i['image_file'],
                image_format=i['image_format']
            )

    def log_search_hit (self, map_id, object_type, est_visual_distance, est_lidar_dist, 
                        vehicle_relative_heading, est_x, est_y, vehicle_x, vehicle_y, vehicle_heading, 
                        confidence, camera_id, camera_angle, image_file, image_format):
        resp = self.__get_nav_service().log_search_hit (
            vehicle_id = self.__vehicle_id,
            session_id = self.__session_id,
            map_id = map_id, 
            object_type = object_type,
            est_visual_distance = est_visual_distance,
            est_lidar_dist = est_lidar_dist, 
            vehicle_relative_heading = vehicle_relative_heading,
            est_x = est_x,
            est_y = est_y,
            vehicle_x = vehicle_x,
            vehicle_y = vehicle_y,
            vehicle_heading = vehicle_heading, 
            confidence = confidence,
            camera_id = camera_id,
            camera_angle = camera_angle,
            image_file = image_file,
            image_format = image_format
        )

    def __get_nav_service (self):
        if self.__nav_svc is None:
            for trial_url in self.__config['NavServiceUrls']:
                svc = NavService(trial_url)
                try:
                    vehicles = svc.get_vehicles()
                    if len(vehicles) > 0:
                        self.__nav_svc = svc
                        logging.getLogger(__name__).info(f"Nav found at {trial_url}")
                        break
                except:
                    logging.getLogger(__name__).info(f"Nav Service not found at {trial_url}")
        
        if self.__nav_svc is None:
            logging.getLogger(__name__).error(f"Nav service not found in list: {self.__config['NavServiceUrls']}")
            raise Exception("Nav Service Connection Error")
        
        return self.__nav_svc
        
        
    def __load_config (self, config_file):
        self.__config_file = config_file
        with open(self.__config_file, 'r') as cfg:
            self.__config = json.loads(cfg.read())
