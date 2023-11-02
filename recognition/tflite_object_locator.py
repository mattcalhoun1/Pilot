import cv2
import numpy as np
try:
    import tflite_runtime.interpreter as tflite
except:
    import tensorflow.lite as tflite
import time
import logging
from recognition.object_locator import ObjectLocator

class TFLiteObjectLocator (ObjectLocator):
    def __init__(self, model_configs = {}, keep_latest_image = True):
        self.__models = {}
        self.__labels = {}
        self.__model_configs = model_configs
        self.__latest_image = None
        self.__keep_latest_image = keep_latest_image
        
        for model_name in model_configs:
            self.__labels[model_name] = self.read_label_file(model_configs[model_name]['LabelFile'])
            if model_configs[model_name]['ModelType'] in ['tflite', 'lite']:
                interpreter = tflite.Interpreter(model_path=model_configs[model_name]['ModelFile'], num_threads=4)
                interpreter.allocate_tensors()
                self.__models[model_name] = interpreter
                logging.getLogger(__name__).info(f"Model {model_name}: TFLite interpreter created.")
            else:
                raise Exception (f"Unknown model type: {model_configs[model_name]['ModelType']}")
            
    def find_objects_on_camera(self, camera, object_filter = None, min_confidence = 0.4):
        captured = camera.capture_image()
        if captured is not None:
            preprocessed = camera.preprocess_image(captured)
            return self.find_objects_in_image(image = preprocessed, object_filter=object_filter, min_confidence=min_confidence)
        logging.getLogger(__name__).warning("Null image received from camera")
        return []

    def get_latest_image (self):
        return self.__latest_image

    def find_objects_in_image(self, image, object_filter = None, min_confidence = 0.4):
        if self.__keep_latest_image:
            self.__latest_image = image # for retrieving later

        last_width = None
        last_height = None
        last_input_data = None
        last_floating_model = None
        detected_objects = []
        
        for m in self.__models:
            interpreter = self.__models[m]
            input_details = interpreter.get_input_details()
            output_details = interpreter.get_output_details()
            height = input_details[0]['shape'][1]
            width = input_details[0]['shape'][2]
            floating_model = False
            if input_details[0]['dtype'] == np.float32:
                floating_model = True
            
            # don't reconvert the image if we dont need to
            input_data = None
            if height == last_height and width == last_width and last_input_data is not None and last_floating_model == floating_model:
                input_data = last_input_data
            else:
                logging.getLogger(__name__).debug("Converting image, since this is first model pass")
                last_width = width
                last_height = height
                last_floating_model = floating_model
                
                # picamera2 will have 2d image shape, cv2 will have 3d (channels)
                initial_h = None
                initial_w = None
                picture = None
                if len(image.shape) == 3: # cv2 camera
                    # model does better with gray scale
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    initial_h, initial_w = image.shape
                    picture = cv2.resize(image, (width, height), cv2.INTER_CUBIC) # INTER_CUBIC, INTER_AREA, INTER_LINEAR, INTER_NEAREST
                    picture = cv2.cvtColor(picture, cv2.COLOR_GRAY2RGB)
                    cv2.imwrite('/tmp/tflite_locator.png', picture)
                    
                else:
                    rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
                    initial_h, initial_w, channels = rgb.shape
                    picture = cv2.resize(rgb, (width, height))

                input_data = np.expand_dims(picture, axis=0)
                if floating_model:
                    input_data = (np.float32(input_data) - 127.5) / 127.5
                last_input_data = input_data

            # invoke the detection model
            interpreter.set_tensor(input_details[0]['index'], input_data)
            interpreter.invoke()

            # retrieve the detected bound boxes
            detected_boxes = interpreter.get_tensor(output_details[0]['index'])
            detected_classes = interpreter.get_tensor(output_details[1]['index'])
            detected_scores = interpreter.get_tensor(output_details[2]['index'])
            num_boxes = interpreter.get_tensor(output_details[3]['index'])

            # fix for this new model
            if True:
                #box_count = num_boxes[0]
                #c_temp = detected_boxes
                #detected_boxes = detected_scores
                #detected_scores = c_temp
                #detected_classes = num_boxes
                #num_boxes = box_count

                # order in the custom model:
                # 0 = scores, 1 = boxes, 2 = num objects detected?, 3 = classes
                
                detected_scores = interpreter.get_tensor(output_details[0]['index'])
                detected_boxes = interpreter.get_tensor(output_details[1]['index'])
                num_boxes = interpreter.get_tensor(output_details[2]['index'])
                detected_classes = interpreter.get_tensor(output_details[3]['index'])

                logging.getLogger(__name__).debug("Detected Boxes:")
                logging.getLogger(__name__).debug(f"{detected_boxes}")
                logging.getLogger(__name__).debug("num boxes:")
                logging.getLogger(__name__).debug(f"{num_boxes}")
                logging.getLogger(__name__).debug("detected classes:")
                logging.getLogger(__name__).debug(f"{detected_classes}")
                logging.getLogger(__name__).debug("detected scores:")
                logging.getLogger(__name__).debug(f"{detected_scores}")
            # end fix


            for i in range(int(num_boxes[0])):
                top, left, bottom, right = detected_boxes[0][i]
                classId = int(detected_classes[0][i])
                if object_filter is None or self.__labels[m][classId] in object_filter:
                    score = detected_scores[0][i]
                    if score > min_confidence:
                        #logging.getLogger(__name__).info(f"{self.__labels[m][classId]} - Top: {top}, Left: {left}, Bottom: {bottom}, Right: {right}, Conf: {score}")
                        
                        xmin = left * initial_w
                        ymin = top * initial_h
                        xmax = right * initial_w
                        ymax = bottom * initial_h
                        box = [xmin, ymin, xmax, ymax]
                        x_center = xmin + ((xmax-xmin)/2)
                        y_center = ymin + ((ymax-ymin)/2)
                        
                        #if xmin >= 0 and ymin >= 0 and ymax <= last_height and xmax <= last_width:
                            #rectangles.append(box)
                            #logging.getLogger(__name__).info(f"Found {self.__labels[m][classId]} centered at ({x_center},{y_center}), confidence: {score}, [({xmin},{ymin}):({xmax},{ymax})]")
                        detected_objects.append({
                            'object':self.__labels[m][classId],
                            'x_center':x_center,
                            'y_center':y_center,
                            'x_min':max(1,xmin),
                            'x_max':min(initial_w, xmax),
                            'y_min':max(1,ymin),
                            'y_max':min(initial_h, ymax),
                            'confidence':score
                        })
                        #else:
                        #    logging.getLogger(__name__).warning("Out of bounds object on image, ignoring!")

        return detected_objects

