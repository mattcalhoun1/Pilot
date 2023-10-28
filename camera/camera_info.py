from camera.image_resolution import ImageResolution
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

    # ImageResolution(1536.0,864.0) if high_res == True else ImageResolution(640.0,640.0)
    CameraConfig = {
        'PI2_STD_HQ_CROPPED_0': {
            'SENSOR_ID':0,
            'FOV_H': 71.0,
            'FOV_V': 49.4,
            'CLASS': 'CV2Camera',
            'HIGHRES':True,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0)
        },        
        'PI2_STD_HQ_CROPPED_1': {
            'SENSOR_ID':1,
            'FOV_H': 71.0,
            'FOV_V': 49.4,
            'CLASS': 'CV2Camera',
            'HIGHRES':True,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0)
        },
        'XX_PI2_STD_HQ_CROPPED_0': {
            'SENSOR_ID':0,
            'FOV_H': 62.2,
            'FOV_V': 48.8,
            'CLASS': 'CV2Camera',
            'HIGHRES':True,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0)
        },        
        'XX_PI2_STD_HQ_CROPPED_1': {
            'SENSOR_ID':1,
            'FOV_H': 62.2,
            'FOV_V': 48.8,
            'CLASS': 'CV2Camera',
            'HIGHRES':True,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0)
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
            'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0)
        },        
        'PI2_STD_LOWRES': {
            'SENSOR_ID':0,
            'FOV_H': 62.2,
            'FOV_V': 48.8,
            'CLASS': 'CV2Camera',
            'HIGHRES':True,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1640.0,922.0)
        },        

        'PI3_STD': {
            'FOV_H': 66.0,
            'FOV_V': 41.0,
            'CLASS': 'Picamera2Camera',
            'HIGHRES':True,
            'FLIPPED':False,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1536.0,864.0)
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
            'SENSOR_ID':1
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
            'SENSOR_ID':0
        },
        'PI3_STD_CROPPED_0': {
            'FOV_H': 33.0,
            'FOV_V': 20.5,
            'CLASS': 'Picamera2Camera',
            'HIGHRES':True,
            'FLIPPED':False,
            'DEFAULT_FOCUS':10,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1536.0,864.0),
            'SENSOR_ID':1
        },
        'PI3_STD_CROPPED_1': {
            'FOV_H': 33.0,
            'FOV_V': 20.5,
            'CLASS': 'Picamera2Camera',
            'HIGHRES':True,
            'FLIPPED':False,
            'DEFAULT_FOCUS':10,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1536.0,864.0),
            'SENSOR_ID':0
        },
        'PI3_WIDE': {
            'FOV_H': 102.0,
            'FOV_V': 67.0,
            'CLASS': 'Picamera2Camera',
            'HIGHRES':True,
            'FLIPPED':False,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1536.0,864.0)
        },
        'IMX_219_160': {
            'FOV_H': 162.4,
            'FOV_V': 124.0,
            'CLASS': 'Picamera2Camera',
            'HIGHRES':True,
            'FLIPPED':True,
            'DEFAULT_FOCUS':5,
            'AUTO_OPTIMIZE':True,
            'IMAGE_RESOLUTION':ImageResolution(1536.0,864.0)
        }

    }
