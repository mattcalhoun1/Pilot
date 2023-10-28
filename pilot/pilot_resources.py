import time
import logging
import statistics
from camera.camera_info import CameraInfo
from camera.camera_manager import CameraManager
from field.field_map import FieldMap
from navsvc.nav_service import NavService
import json
import copy
import os

class PilotResources:
    def __init__(self, config_file):
        self.__load_config(config_file)
        self.__assignments = []
        
        self.__map_cache_location = self.__config['CacheLocations']['Maps']
        self.__model_cache_location = self.__config['CacheLocations']['Models']
        
        self.__maps = {} # map id to field map
        self.__models = {} # map of model ids to model configs
        
        self.__nav_svc = None
    
    def get_config (self):
        return self.__config
        
    def get_map (self, map_id):
        return self.__maps[map_id]
    
    def download_resources (self, use_cached_maps = True, use_cached_models = True):
        self.get_assignments(refresh=True, use_cached_maps = use_cached_maps, use_cached_models = use_cached_models)
        

    def get_assignments (self, refresh=False, use_cached_maps = True, use_cached_models = True):
        if refresh or self.__assignments is None or len(self.__assignments) == 0:
            self.__download_assignments()

        # download maps
        self.__download_maps(use_cached_maps)
        
        # download models
        self.__download_models(use_cached_models)
        
        return self.__assignments

    def download_map (self, map_id, use_cached_maps = True, use_cached_models = True):
        if map_id not in self.__maps:
            self.__fetch_map(map_id, use_cached_maps)

            self.__download_models (cache_allowed = use_cached_models, map_id = map_id)


    def __fetch_map (self, map_id, cache_allowed):
        map_file = f"{self.__map_cache_location}/{map_id}.json"
        map_json = None
        if (cache_allowed and os.path.isfile(map_file)):
            with open(map_file, 'r') as min:
                map_content = min.read()
                map_json = json.loads(map_content)
                logging.getLogger(__name__).debug(f"Using cached map: {map_id}")
                
        else:
            # download from the service
            map_json = self.__get_nav_service().get_map(map_id)
            logging.getLogger(__name__).info(f"Downloaded map: {map_id}")

            # save to the cache
            with open(map_file, 'w') as mout:
                mout.write(json.dumps(map_json, indent=2))
                logging.getLogger(__name__).info(f"Saved map: {map_id} to {map_file}")

        if map_json is not None:
            self.__maps[map_id] = FieldMap(
                landmarks=map_json['landmarks'],
                shape=map_json['shape'],
                boundaries=map_json['boundaries'],
                search=map_json['search'],
                obstacles=map_json['obstacles'],
                near_boundaries=map_json['near_boundaries'],
                name=map_id)
        else:
            logging.getLogger(__name__).error(f"Unable to download or locate map: {map_id}")

    def __fetch_model (self, model, cache_allowed):
        model_config = self.__get_model_config_location(model)
        if (cache_allowed and os.path.isfile(model_config)):
            logging.getLogger(__name__).debug(f"Using cached model: {model}")
        else:
            # need to download the model and mappings
            downloaded_model = self.__get_nav_service().get_recognition_model(model)
            self.__save_model(downloaded_model, model)
            logging.getLogger(__name__).info(f"Downloaded model: {model}")
        
        # add this model to our list of models
        with open(model_config, 'r') as min:
            self.__models[model] = json.loads(min.read())

    def complete_assignment (self, assignment):
        self.__get_nav_service().complete_assignment(assignment['vehicle_id'], assignment['entry_num'])

    def __download_assignments (self):
        self.__assignments = self.__get_nav_service().get_assignments(self.__config['Vehicle'])

    def get_model_configs (self):
        return self.__models
            
    def __load_config (self, config_file):
        self.__config_file = config_file
        with open(self.__config_file, 'r') as cfg:
            self.__config = json.loads(cfg.read())
        
    # download all maps required for the given assignments
    def __download_maps (self, cache_allowed = True):
        # only download maps we don't already have
        for a in self.__assignments:
            m = a['assignment']['map_id']
            self.__fetch_map(m, cache_allowed)                
        
    # download all models required for the given assignments
    def __download_models (self, cache_allowed = True, map_id = None):
        # only download models we don't already have
        ready_models = []
        maps_to_check = self.__maps
        if map_id is not None:
            maps_to_check = [map_id,]

        for m in maps_to_check:
            # download the 
            required_models = self.__maps[m].get_required_models()
            for model in required_models:
                if model not in ready_models:
                    self.__fetch_model(model, cache_allowed)
                    ready_models.append(m)
                        

    def __save_model (self, downloaded_model, model_id):
        model_file = self.__get_model_file_location(model_id)
        mapping_file = self.__get_mapping_file_location(model_id)
        
        # save the mappings
        mappings = downloaded_model['additional_params']['object_mappings']
        obj_num = 0
        with open(mapping_file, 'w') as mout:
            while f"{obj_num}" in mappings:
                obj_name = mappings[f"{obj_num}"]
                mout.write(f"{obj_num} {obj_name}\n")
                obj_num += 1
        
        # save the model content
        encoded_model = downloaded_model['encoded_model']
        with open(model_file, 'wb') as mout:
            mout.write(self.__get_nav_service().decode(encoded_model))
        
        # save the config
        cfg = {
            "ModelType":downloaded_model['model_format'],
            "LabelFile":self.__get_mapping_file_location(model_id),
            "ModelFile":self.__get_model_file_location(model_id),
            "LandmarkType":downloaded_model['additional_params']['object_type']
        }

        with open(self.__get_model_config_location(model_id), 'w') as mout:
            mout.write(json.dumps(cfg, indent=2))

                    
    def __get_model_config_location (self, model_id):
        return f"{self.__model_cache_location}/{model_id}.json"

    def __get_model_file_location (self, model_id):
        return f"{self.__model_cache_location}/{model_id}.tflite"

    def __get_mapping_file_location (self, model_id):
        return f"{self.__model_cache_location}/{model_id}.txt"
            
    
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
