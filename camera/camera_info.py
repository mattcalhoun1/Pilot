from camera.image_resolution import ImageResolution
from camera.camera_specific_detection import CameraSpecificDetection

class CameraInfo:
    @staticmethod
    def get_fov_horizontal (config_id):
        return CameraInfo.CameraConfig[config_id]['FOV_H']

    @staticmethod
    def get_fov_vertical (config_id):
        return CameraInfo.CameraConfig[config_id]['FOV_V']
    
    @staticmethod
    def get_resolution_width (config_id):
        return CameraInfo.CameraConfig[config_id]['IMAGE_RESOLUTION'].get_width()
    
    @staticmethod
    def get_resolution_height (config_id):
        return CameraInfo.CameraConfig[config_id]['IMAGE_RESOLUTION'].get_height()

    @staticmethod
    def get_resolution (config_id):
        return CameraInfo.CameraConfig[config_id]['IMAGE_RESOLUTION']

    # ImageResolution(1536.0,864.0) if high_res == True else ImageResolution(640.0,640.0)
    CameraConfig = {
        'PI2_STD_HQ_CROPPED_0': {
            'SENSOR_ID':0,
            'FOV_H': 38.42,
            'FOV_V': 24.354,
            'CLASS': 'CV2Camera',
            'HIGHRES':False,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1920.0,1080.0),
            'DETECTION': CameraSpecificDetection.Settings[CameraSpecificDetection.STANDARD],
            'BARREL_DISTORTION_AT_EDGE':0.0
        },        
        'PI2_STD_HQ_CROPPED_1': {
            'SENSOR_ID':1, 
            'FOV_H': 38.42,
            'FOV_V': 24.354,
            'CLASS': 'CV2Camera',
            'HIGHRES':False,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0),
            'DETECTION': CameraSpecificDetection.Settings[CameraSpecificDetection.STANDARD],
            'BARREL_DISTORTION_AT_EDGE':0.0
        },

        'xPI2_STD_HQ_CROPPED_0': {
            'SENSOR_ID':0,
            'FOV_H': 71.0,
            'FOV_V': 49.4,
            'CLASS': 'CV2Camera',
            'HIGHRES':True,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0),
            'DETECTION': CameraSpecificDetection.Settings[CameraSpecificDetection.WIDE],
            'BARREL_DISTORTION_AT_EDGE':0.045
        },        
        'xPI2_STD_HQ_CROPPED_1': {
            'SENSOR_ID':1, 
            'FOV_H': 71.0,
            'FOV_V': 49.4,
            'CLASS': 'CV2Camera',
            'HIGHRES':True,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1280.0,720.0),
            'DETECTION': CameraSpecificDetection.Settings[CameraSpecificDetection.WIDE],
            'BARREL_DISTORTION_AT_EDGE':0.045
        },
   
        'PI2_STD_HIRES': {
            'SENSOR_ID':0,
            'FOV_H': 62.2,
            'FOV_V': 48.8,
            'CLASS': 'CV2Camera',
            'HIGHRES':True,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0),
            'DETECTION': CameraSpecificDetection.Settings[CameraSpecificDetection.STANDARD],
            'BARREL_DISTORTION_AT_EDGE':0.0
        },        
        'PI3_STD_HQ_CROPPED_0': {
            'FOV_H': 44.0,
            'FOV_V': 27.333,
            'CLASS': 'Picamera2Camera',
            'HIGHRES':True,
            'FLIPPED':False,
            'DEFAULT_FOCUS':10,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1536.0,864.0),
            'SENSOR_ID':1,
            'DETECTION': CameraSpecificDetection.Settings[CameraSpecificDetection.STANDARD],
            'BARREL_DISTORTION_AT_EDGE':0.0
        },
        'PI3_STD_HQ_CROPPED_1': {
            'FOV_H': 44.0,
            'FOV_V': 27.333,
            'CLASS': 'Picamera2Camera',
            'HIGHRES':True,
            'FLIPPED':False,
            'DEFAULT_FOCUS':10,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1536.0,864.0),
            'SENSOR_ID':0,
            'DETECTION': CameraSpecificDetection.Settings[CameraSpecificDetection.STANDARD],
            'BARREL_DISTORTION_AT_EDGE':0.0
        },
        'PI3_FULL_WIDTH_0': {
            'FOV_H': 66.0,
            'FOV_V': 30.15, # we crop the top and bottom down to 1000px
            'CLASS': 'Picamera2Camera',
            'HIGHRES':True,
            'FLIPPED':False,
            'DEFAULT_FOCUS':10,
            'AUTO_OPTIMIZE':False,
            'IMAGE_RESOLUTION':ImageResolution(2304.0,1000.0),
            'SENSOR_ID':1,
            'DETECTION': CameraSpecificDetection.Settings[CameraSpecificDetection.WIDE],
            'BARREL_DISTORTION_AT_EDGE':0.045
        },
        'PI3_FULL_WIDTH_1': {
            'FOV_H': 66.0,
            'FOV_V': 30.15, # we crop the top and bottom down to 1000px
            'CLASS': 'Picamera2Camera',
            'HIGHRES':True,
            'FLIPPED':False,
            'DEFAULT_FOCUS':10,
            'AUTO_OPTIMIZE':False,
            'IMAGE_RESOLUTION':ImageResolution(2304.0,1000.0),
            'SENSOR_ID':0,
            'DETECTION': CameraSpecificDetection.Settings[CameraSpecificDetection.WIDE],
            'BARREL_DISTORTION_AT_EDGE':0.045
        }
    }

#PI3 Details:
# imx708_noir [4608x2592] (/base/soc/i2c0mux/i2c@1/imx708@1a)
# Modes: 'SRGGB10_CSI2P' : 1536x864 [120.13 fps - (768, 432)/3072x1728 crop]
#                          2304x1296 [56.03 fps - (0, 0)/4608x2592 crop]
#                          4608x2592 [14.35 fps - (0, 0)/4608x2592 crop]
# Focal Length: 4.7mm
# Diagonal FOV: 75
# Horizontal FOV: 66
# Vertical FOV: 41
# Focal Stop: F1.8

