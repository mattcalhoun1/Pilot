#from recognition.lcd.lcd_screen import LcdScreen
from landmarks.emitter_landmark_finder import EmitterLandmarkFinder
from landmarks.basic_landmark_finder import BasicLandmarkFinder
from position.position_estimator_with_clustering import PositionEstimatorWithClustering
import time
import logging
import statistics
from camera.camera_info import CameraInfo
from camera.camera_manager import CameraManager
from recognition.tflite_object_locator import TFLiteObjectLocator
from field.field_map import FieldMap
import json
import copy

device_config = {}
with open('/home/matt/projects/NavConfig/pi_cm4/nav_settings.json', 'r') as cfg:
    device_config = json.loads(cfg.read())

locator = TFLiteObjectLocator(model_configs = device_config['Models'])
camera_mgr = CameraManager()
last_location_update = time.time()

enabled_cameras = {}
camera_headings = {}
def get_camera (c):
    return camera_mgr.get_camera(c, camera_config=enabled_cameras[c], auto_optimize_object_locator=locator)


for c in device_config['Cameras']:
    if device_config['Cameras'][c]['Enabled'] == True:
        enabled_cameras[c] = device_config['Cameras'][c]['Config']
        camera_headings[c] = device_config['Cameras'][c]['DefaultHeading']
        
        # start the camera
        get_camera(c)

field_map = FieldMap(
    landmarks=device_config['Maps']['Basement']['Landmarks'],
    shape=device_config['Maps']['Basement']['Shape'],
    boundaries=device_config['Maps']['Basement']['Boundaries']
    )

finders = {}
for c in enabled_cameras:
    # for each model in the map, add a finder
    finders[c] = []
    for m in device_config['Models']:
        if device_config['Models'][m]['LandmarkType'] == 'emitter':
            finders[c].append(EmitterLandmarkFinder(
                camera_config=CameraInfo.CameraConfig[enabled_cameras[c]],
                field_map=field_map
            ))
        
        elif device_config['Models'][m]['LandmarkType'] == 'basic':
            finders[c].append(BasicLandmarkFinder(
                camera_config=CameraInfo.CameraConfig[enabled_cameras[c]],
                field_map=field_map,
                model_name = m
            ))


def locate_landmarks (confidence_threshold):
    located_landmarks = {}
    for c in enabled_cameras:
        c_located_objects = locator.find_objects_on_camera(camera=get_camera(c), min_confidence = confidence_threshold)#object_filter=['light'], min_confidence = confidence_threshold)
        logging.getLogger(__name__).debug(f"Camera {c} found {len(c_located_objects)} objects: {c_located_objects}")
        located_landmarks[c] = {}
        for f in finders[c]:
            f_located = f.locate_landmarks(object_locations=c_located_objects)
            for lid in f_located:
                located_landmarks[c][lid] = f_located[lid]
            #located_landmarks[c] = located_landmarks[c] + f.locate_landmarks(object_locations=c_located_objects)#, id_filter=['light'])
    
    return located_landmarks

position_est = {}
for c in enabled_cameras:
    position_est[c] = PositionEstimatorWithClustering(
        field_map = field_map,
        horizontal_fov = CameraInfo.get_fov_horizontal(config_id=enabled_cameras[c]),
        vertical_fov=CameraInfo.get_fov_vertical(config_id=enabled_cameras[c]),
        view_width=CameraInfo.get_resolution_width(config_id=enabled_cameras[c]),
        view_height=CameraInfo.get_resolution_height(config_id=enabled_cameras[c]),
        base_front=90.0
    )


def format_landmarks_for_position (located_objects, camera_heading):
    as_list = []
    for lid in located_objects:
        this_obj = copy.deepcopy(located_objects[lid])
        this_obj['camera_heading'] = camera_heading
        as_list.append({lid:this_obj})
    
    return as_list

if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)

    # order to try focus searches
    focus_distances = [10.0, 20.0, 30.0, 5.0, 50.0, 15.0, 40.0]
    curr_focus = 0
    confidence_threshold = 0.2

    # distances to cycle through when searching
    #camera_mgr.get_camera('0').set_focus_meters(focus_distances[curr_focus])
    #camera_mgr.get_camera('0').optimize_for_distance(
    #    focus_distances[curr_focus], 
    #    match_threshold_pct = 0.2, 
    #    expected_num_objects=3, 
    #    object_ids = ['light'],
    #    min_confidence=confidence_threshold)

    go = True

    try:
        while (go):
            landmarks = locate_landmarks(confidence_threshold=confidence_threshold)

            #combined_landmarks = []
            for c in landmarks:
                camera_heading = camera_headings[c]# - 90
                #logging.getLogger(__name__).info(format_landmarks_for_position(landmarks))
                for lid in landmarks[c]:
                    l = landmarks[c][lid]
                    logging.getLogger(__name__).info(f"Cam {c} (@{camera_heading}) Located: {lid} @ {l['x1'],l['y1']} - {l['x2'],l['y2']}")

                formatted_landmarks = format_landmarks_for_position(located_objects=landmarks[c],camera_heading=camera_heading)
                angles = position_est[c].extract_object_view_angles(located_objects=formatted_landmarks)
                distances =position_est[c].extract_distances(view_angles=angles, view_altitude=8.5)

                    
                logging.getLogger(__name__).info(f"Distances: {distances}")
            
                time.sleep(1)
    except KeyboardInterrupt:
        logging.getLogger(__name__).info("Keyboard interrupt")
    except Exception as e:
        logging.getLogger(__name__).warning(f"Exception thrown: {e}")

    camera_mgr.cleanup_cameras()

