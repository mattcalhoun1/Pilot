from camera.camera import Camera
from camera.picamera2_camera import Picamera2Camera
from camera.cv2_camera import CV2Camera
from camera.camera_info import CameraInfo
from recognition.object_locator import ObjectLocator
import logging

class CameraManager:
    __cameras = {}

    def __init__(self, release_on_cleanup = False):
        # class-level cache so cameras dont get recreated just because new instance of camera manager
        self.__cameras = CameraManager.__cameras
        self.__release_on_cleanup = release_on_cleanup
    
    def get_camera(self, camera_id : str, camera_config : str, auto_optimize_object_locator : ObjectLocator = None) -> Camera:
        if camera_id in self.__cameras:
            return self.__cameras[camera_id]
        
        cfg = CameraInfo.CameraConfig[camera_config]
        self.__log_camera_config(camera_id, camera_config, cfg, auto_optimize_object_locator)
        
        if cfg['CLASS'] == 'Picamera2Camera':
            self.__cameras[camera_id] = Picamera2Camera(
                high_res=cfg['HIGHRES'],
                flipped=cfg['FLIPPED'],
                default_focus_distance=cfg['DEFAULT_FOCUS'],
                auto_optimize=cfg['AUTO_OPTIMIZE'],
                auto_optimize_object_locator=auto_optimize_object_locator,
                sensor_id=cfg['SENSOR_ID']
            )
            return self.__cameras[camera_id]

        elif cfg['CLASS'] == 'CV2Camera':
            self.__cameras[camera_id] = CV2Camera(
                high_res=cfg['HIGHRES'],
                flipped=cfg['FLIPPED'],
                default_focus_distance=cfg['DEFAULT_FOCUS'],
                auto_optimize=cfg['AUTO_OPTIMIZE'],
                auto_optimize_object_locator=auto_optimize_object_locator,
                sensor_id=cfg['SENSOR_ID']
            )
            return self.__cameras[camera_id]
        
        raise Exception(f"Unknown camera type: {camera_config}")

    def __log_camera_config (self, camera_id, camera_config, cfg, locator : ObjectLocator):
        logging.getLogger(__name__).info(f"== Camera configuration ==")
        printable_settings = {
            'Class': cfg['CLASS'],
            'Camera ID': camera_id, 
            'Config': camera_config,
            'Sensor ID': cfg['SENSOR_ID'],
            'Flipped':cfg['FLIPPED'],
            'Locator Type':type(locator) if locator is not None else 'None'
        }
        for k in printable_settings:
            logging.getLogger(__name__).info(f"{k:>15}: {printable_settings[k]}")

    def cleanup_cameras (self):
        # on cm4 this is not necessary
        if self.__release_on_cleanup:
            for cid in self.__cameras:
                self.__cameras[cid].stop_camera()
                self.__cameras[cid].release()
            
            self.__cameras.clear()
