import cv2
try:
    from picamera2 import MappedArray, Picamera2, Preview
    from libcamera import controls, Transform
except:
    # running on jetson, or missing library
    pass
import time
import logging
from camera.camera import Camera
import numpy as np

# camera implementation based on picamera2 library, only for PI
# it is able to apply picam v3 specific settings
# default_focus_distance is the starting distance in meters
# auto_optimize is whether the camera will re-optimize settings
# every time the focus distance changes by a certain threshold
# auto-optimizing is slow when first done for any given distance, but faster afterward
class Picamera2Camera (Camera):
    def __init__(self, high_res = False, flipped = True, default_focus_distance = 5, auto_optimize = True, auto_optimize_object_locator = None, picam = None, sensor_id = 0, image_resolution = None):

        attached_cameras = Picamera2.global_camera_info()
        logging.getLogger(__name__).info(f"Attached Cameras: {attached_cameras}")

        self.__normal_size = (round(image_resolution.get_width()), round(image_resolution.get_height()))
        self.__lowres_size = (round(image_resolution.get_width()), round(image_resolution.get_height())) if high_res == True else (640,640)

        if picam is not None:
            self.__picam2 = picam
        else:
            self.__picam2 = Picamera2(camera_num=sensor_id) # if there are multiple cameras attached, change this to identify the #
            h_flip = 1 if flipped ==True else 0
            v_flip = 1 if flipped == True else 0
            config = self.__picam2.create_preview_configuration(main={"size": self.__normal_size},
                                                        lores={"size": self.__lowres_size},#, "format": "YUV420"},
                                                        transform=Transform(hflip=h_flip,vflip=v_flip))
            self.__picam2.configure(config)

        self.__stride = self.__picam2.stream_configuration("lores")["stride"]

        self.__auto_optimize = auto_optimize
        self.__af_obj_locator = auto_optimize_object_locator

        if self.__auto_optimize and self.__af_obj_locator is None:
            logging.getLogger(__name__).error('Auto-optimize is configured, but no default object locater was given. Auto-optimize will be disabled!')

        self.__focus = {
            'curr_focus_meters':default_focus_distance,
            'curr_focus':self.__calc_focus_for_meters(default_focus_distance),
            'focus_step':0.5, # step size in meters
            'focus_next_step':0.5, # size (meters) and direction of next step
            'focus_max':100.0, # Farthest we should search (meters)
            'focus_min':0.5 # Closest we should search (meters)
        }

        self.__optimal_settings = {}

        # turn off auto-focus
        self.start_camera()
        self.__picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": self.__focus['curr_focus']})

        self.set_focus_meters(default_focus_distance)

    # sets focus on the givn meters.
    # the optional next_direction_closer sets the flag for future
    # focus search to go closer or more distant. If you wnt it to search closer on, set to False
    def set_focus_meters (self, meters, next_direction_closer = True):
        # from picam2 docs:
        # The units for this control are dioptres (1 / distance in meters), so that zero can be used to denote
        #" infinity".
        if self.__auto_optimize:
            self.optimize_for_distance (meters)
        else:
            self.__set_focus_meters(meters=meters)
        if next_direction_closer:
            self.__focus['focus_next_step'] = -1 * self.__focus['focus_step']
        else:
            self.__focus['focus_next_step'] = 1 * self.__focus['focus_step']

        logging.getLogger(__name__).debug(f"Focus adjusted to {meters} meters")


    # if doing a focus search, this will set to the next setp of the search
    def set_next_focus (self):
        curr_focus_meters = self.get_current_focus()
        next_focus_meters = curr_focus_meters + (curr_focus_meters * self.__focus['focus_next_step'])

        # if we hit top or bottom end of focus range, change direction
        if next_focus_meters <= self.__focus['focus_min']:
            self.__focus['focus_next_step'] = self.__focus['focus_step']
            next_focus_meters = self.__focus['focus_min']
        elif next_focus_meters >= self.__focus['focus_max']:
            next_focus_meters = self.__focus['focus_max']
            self.__focus['focus_next_step'] = -1 * self.__focus['focus_step']

        if self.__auto_optimize:
            self.optimize_for_distance (next_focus_meters)
        else:
            self.__set_focus_meters(next_focus_meters)

    def __calc_focus_for_meters (self, meters):
        return round(1/meters,2)

    def __set_focus_meters (self, meters):
        if self.__focus['curr_focus_meters'] != meters:
            logging.getLogger(__name__).debug(f"Focus adjusted to {meters} meters, next step: {'closer' if self.__focus['focus_next_step'] < 0 else 'farther'}")
            self.__focus['curr_focus_meters'] = meters
            self.__focus['curr_focus'] = self.__calc_focus_for_meters(self.__focus['curr_focus_meters'])        
            self.__picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": self.__focus['curr_focus']})
        else:
            logging.getLogger(__name__).debug(f"Focus unchanged at {meters} meters, next step: {'closer' if self.__focus['focus_next_step'] < 0 else 'farther'}")


    def get_current_focus (self):
        return self.__focus['curr_focus_meters']

    def start_camera (self):
        self.__picam2.start()

    def capture_image (self, preprocess = False, file_name = None):
        buffer = self.__picam2.capture_buffer("lores")
        grey = buffer[:self.__stride * self.__lowres_size[1]].reshape((self.__lowres_size[1], self.__stride))

        if preprocess:
            grey = self.preprocess_image(grey)

        if file_name is not None:
            #cv2.imwrite(file_name, grey)
            #np.ndarray.tofile(grey, file_name)
            np.save(file_name, grey)

        return grey

    def preprocess_image (self, image):
        return image
    
    def stop_camera (self):
        self.__picam2.stop()
        
    def release (self):
        pass

    # sets focus, finds (if necessary) optimal settings, and applies optimal settings
    # needs to be pointed at some objects during this optimization, or default settings will be used
    def optimize_for_distance (self, meters, force_search = False, match_threshold_pct = 0.2, expected_num_objects = 1, object_ids = None, min_confidence = 0.4):
        #logging.getLogger(__name__).info(f'Optimizing for {meters} meters.')
        self.__set_focus_meters(meters=meters)

        if self.__af_obj_locator is not None:
            # find optimal settings for the given distance
            cached_settings = None

            # see if we have a cached setting that would work
            if force_search == False:
                # try to find a suitable setting
                # 20% allowance
                closest_match = None
                for s in self.__optimal_settings:
                    if closest_match is None or (abs(s - meters) < abs(closest_match - meters)):
                        closest_match = s
                
                if closest_match is not None and (abs(meters - closest_match)/meters <= match_threshold_pct):
                    logging.getLogger(__name__).debug(f'Settings for {closest_match} are close enough.')
                    cached_settings = self.__optimal_settings[closest_match]
                else:
                    logging.getLogger(__name__).debug(f'No suitable settings found, initiating search.')


            if cached_settings is None:
                # do the search
                optimal_settings = self.__find_optimal_settings(apply_settings=True, search_stop_threshold=expected_num_objects,object_ids=object_ids, min_confidence=min_confidence)
                for setting_name in optimal_settings:
                    logging.getLogger(__name__).debug(f"Optimal Setting @ {meters} meters - {setting_name} : {optimal_settings[setting_name]}")

                # cache the results
                self.__optimal_settings[meters] = optimal_settings
                cached_settings = optimal_settings
            else:
                self.__apply_camera_settings(cached_settings)

            logging.getLogger(__name__).debug(f'Optimized for {meters} meters. {cached_settings}')
        else:
            logging.getLogger(__name__).warning('Auto-optimize bypassed because no object locator was configured')
        
    # locates optimal settings by trying some and feeding
    # images to the given object locator. when objects are not found
    # the settings are considered sub-optimal.
    # if no objects are ever found, the defaults are used.
    def __find_optimal_settings (self, apply_settings = True, search_stop_threshold = 3, object_ids = None, min_confidence = 0.4):
        # adjust the settings until we find the desired number of objects.
        # if that number is never found, the settings that found the highest number
        # of objects is used. If all are 0, the camera default settings are used
        base_settings = {}
        selected_settings = {}

        # if a group is already visible, leave the settings alone
        all_opts = self.__setting_options()
        img = self.capture_image()
        objects = self.__af_obj_locator.find_objects_in_image(img, object_ids, min_confidence=min_confidence)
        base_settings_work = len(objects) >= search_stop_threshold

        for setting_name in all_opts:
            logging.getLogger(__name__).debug(f"Getting base setting for {setting_name}")
            base_settings[setting_name] = self.__get_camera_setting(setting_name)
            logging.getLogger(__name__).debug(f"{setting_name} : {base_settings[setting_name]}")

            # only do search if base settings didn't meet search stop threshold
            if base_settings_work == False:
                max_found = 0
                
                # adjust the setting until a group becomes visible.
                for opt in all_opts[setting_name]:
                    # apply the setting 
                    self.__apply_camera_setting(setting_name, opt)

                    # wait a few frames, some setting changes take a few frames to apply
                    time.sleep(.1)

                    # see if this setting is better than the last
                    img = self.capture_image()
                    objects = self.__af_obj_locator.find_objects_in_image(img, object_ids, min_confidence=min_confidence)
                    if len(objects) > max_found:
                        max_found = len(objects)
                        selected_settings[setting_name] = opt
                    
                    # if we reached target, stop searching
                    if max_found >= search_stop_threshold:
                        break
                # set back to base setting
                self.__apply_camera_setting(setting_name, base_settings[setting_name])

        # for any settings we didn't find optimal values, set back to default
        for setting_name in base_settings:
            if setting_name not in selected_settings:
                selected_settings[setting_name] = base_settings[setting_name]

        if apply_settings:
            self.__apply_camera_settings(selected_settings)
            logging.getLogger(__name__).debug("Optimal settings applied.")
        else:
            # set back to base setting
            self.__apply_camera_settings(base_settings)
            logging.getLogger(__name__).debug("Reverted to base settings after search.")

        return selected_settings

    def __apply_camera_settings (self, settings):
        self.__picam2.set_controls(settings)

    def __apply_camera_setting (self, setting_name, setting_value):
        logging.getLogger(__name__).debug(f"setting {setting_name} = {setting_value}")
        self.__picam2.set_controls({setting_name: setting_value})

    def __get_camera_setting (self, setting_name):
        minval, maxval, currval = self.__picam2.camera_controls[setting_name]
        return currval

    def __setting_options (self):
        # setting that deals with lighting
        return {
            #'AwbMode': [
            #    controls.AwbModeEnum.Tungsten,
            #    controls.AwbModeEnum.Fluorescent,
            #    controls.AwbModeEnum.Indoor,
            #    controls.AwbModeEnum.Daylight,
            #    controls.AwbModeEnum.Cloudy,
            #],
            'Brightness': [
                0,
                -0.05,
                -0.15,
                -0.25,
                -0.5,
                -0.75,
                -0.9
            ],
            'NoiseReductionMode': [
                controls.draft.NoiseReductionModeEnum.Off,
                controls.draft.NoiseReductionModeEnum.Fast,
                controls.draft.NoiseReductionModeEnum.HighQuality,
            ],
            'Sharpness': [
                0.0,
                2.0,
                4.0,
                7.0,
                10.0,
                13.0
            ]
        }
    
