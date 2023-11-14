import requests
import base64
from datetime import datetime
import logging
from lidar.lidar_map import LidarMap
import json
from navsvc.nav_json_encoder import NavJsonEncoder

class NavService ():
    def __init__(self, base_url):
        self.__base_url = base_url
    
    def get_assignments (self, vehicle_id):
        resp = requests.get(self.__get_url("assignments", [vehicle_id,]))
        resp.raise_for_status()
        
        return resp.json()
    
    def complete_assignment (self, vehicle_id, entry_num):
        resp = requests.post(
            self.__get_url("assignment", [vehicle_id,entry_num]),
            json= {
                'vehicle_id':vehicle_id,
                'entry_num': entry_num,
                'complete':True
            }
        )
        resp.raise_for_status()
        
    
    def get_maps (self):
        resp = requests.get(self.__get_url("nav_maps"))
        resp.raise_for_status()
        
        return resp.json()

    def get_map (self, map_id):
        resp = requests.get(
            self.__get_url(
                "nav_map",
                [ map_id,]))
        resp.raise_for_status()
        
        return resp.json()

    def get_vehicles (self):
        resp = requests.get(self.__get_url("vehicles"))
        resp.raise_for_status()
        
        return resp.json()

    def get_recognition_model (self, model_id, model_type = 'object_detection', model_format = 'tflite'):
        resp = requests.get(
            self.__get_url(
                "recognition_model",
                [ model_id, model_type, model_format,]))
        resp.raise_for_status()
        
        return resp.json()
    
    def log_position_and_heading (self, vehicle_id, session_id, map_id, x, y, heading, basis):
        curr_time = datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
        position_data={
            "session_id": session_id,
            "occurred" : curr_time,
            "vehicle_id": vehicle_id,
            "created" : curr_time,
            "position_x" : x,
            "position_y" : y,
            "navmap_id" : map_id,
            "heading": heading,
            "basis": basis
        }
        post_data = json.dumps(position_data, cls=NavJsonEncoder)

        resp = requests.post(
            self.__get_url('position_log', [vehicle_id, session_id]),
            data=post_data,
            headers={'content-type': 'application/json'}
        )
        resp.raise_for_status()
        return resp.json()

    def log_lidar (self, vehicle_id, session_id, lidar_map : LidarMap):
        curr_time = datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
        
        lidar_json = {}
        lidar_data = lidar_map.get_lidar_data()
        for angle in lidar_map.get_available_angles():
            lidar_json[str(angle)] = lidar_data[angle]
        
        resp = requests.post(
            self.__get_url('lidar', [vehicle_id, session_id]),
            json={
                "session_id": session_id,
                "vehicle_id": vehicle_id,
                "occurred" : curr_time,
                "lidar_data": lidar_json
            }
        )
        resp.raise_for_status()
        return resp.json()


    def log_camera_view (self, vehicle_id, session_id, entry_num, camera_id, camera_heading, image_file, image_format):
        encoded_image = None
        with open(image_file, 'rb') as imagein:
            raw_img = imagein.read()
            encoded_image = base64.b64encode(raw_img).decode('utf-8')
        
        if encoded_image is not None:
            try:
                resp = requests.post(
                    self.__get_url('position_view', [vehicle_id, entry_num, camera_id ]),
                    json={
                        "session_id": session_id,
                        "vehicle_id": vehicle_id,
                        "entry_num": entry_num,
                        "camera_id": camera_id,
                        "camera_angle": camera_heading,
                        "image_format":image_format,
                        "encoded_image":encoded_image
                    }
                )
                resp.raise_for_status()
            except Exception as e:
                logging.getLogger(__name__).error(f"Failed to save camera view to service: {e}")
        else:
            logging.getLogger(__name__).warning("Camera view failed to save, encoded image was null")

    def log_search_hit (self, vehicle_id, session_id, map_id, object_type, est_visual_distance, est_lidar_dist, 
                        vehicle_relative_heading, est_x, est_y, vehicle_x, vehicle_y, vehicle_heading, 
                        confidence, camera_id, camera_angle, image_file, image_format):
        encoded_image = None
        with open(image_file, 'rb') as imagein:
            raw_img = imagein.read()
            encoded_image = base64.b64encode(raw_img).decode('utf-8')
        
        if encoded_image is not None:
            json_req = {
                    'object_type':object_type,# = serializers.CharField(required=True, allow_blank=False, max_length=32)
                    'map_id':map_id,# = serializers.CharField(max_length=32)
                    'est_visual_dist':est_visual_distance,# = serializers.FloatField(required=True)
                    'est_lidar_dist':est_lidar_dist,# = serializers.FloatField(required=False)
                    'vehicle_relative_heading':vehicle_relative_heading,# = serializers.FloatField(required=False)
                    'est_x':est_x,# = serializers.FloatField(required=True)
                    'est_y':est_y,# = serializers.FloatField(required=True)
                    'vehicle_x':vehicle_x,# = serializers.FloatField(required=True)
                    'vehicle_y':vehicle_y,# = serializers.FloatField(required=True)
                    'vehicle_heading':vehicle_heading,# = serializers.FloatField(required=True)
                    'confidence':confidence,# = serializers.FloatField(required=True)
                    'vehicle_id':vehicle_id,# = serializers.CharField(required=True, allow_blank=False, max_length=32)
                    'session_id':session_id,# = serializers.CharField(required=True, allow_blank=False, max_length=64)
                    'camera_id':camera_id,# = serializers.CharField(required=True, allow_blank=False, max_length=32)
                    'camera_angle':camera_angle,# = serializers.FloatField()
                    'image_format':image_format,# = serializers.CharField(required=True, allow_blank=False, max_length=4)
                }
            #logging.getLogger(__name__).info(f"Sending : {json.dumps(json_req, indent=2)}")
            json_req['encoded_image'] = encoded_image# = serializers.CharField()        

            resp = requests.post(
                self.__get_url('new_search_hit', [vehicle_id, session_id ]),
                json=json_req
            )
            resp.raise_for_status()
        else:
            logging.getLogger(__name__).warning("Camera view failed to save, encoded image was null")

    def decode (self, encoded):
        return base64.b64decode(encoded)
        
    def __get_url (self, service, params = []):
        svc_url = f"{self.__base_url}/{service}/"
        
        for p in params:
            svc_url = svc_url + f"{p}/"
        
        return svc_url
