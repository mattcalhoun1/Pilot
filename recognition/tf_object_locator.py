import cv2
import numpy as np
#import tflite_runtime.interpreter as tflite
import tensorflow as tf
import time
import logging
from recognition.object_locator import ObjectLocator

class TFObjectLocator (ObjectLocator):
    def __init__(self, model_configs = {}):
        self.__models = {}
        self.__labels = {}
        self.__model_configs = model_configs
        
        for model_name in model_configs:
            self.__labels[model_name] = self.read_label_file(model_configs['LabelFile'])
            if model_configs['ModelFile'] == 'tflite':
                self.__models[model_name] = tf.saved_model.load(model_configs['ModelFile'])
            else:
                raise Exception (f"Unknown model type: {model_configs['ModelFile']}")

    def find_objects_on_camera(self, camera, object_filter = None, min_confidence = 0.4):
        return self.find_objects_in_image(image = camera.capture_image(), object_filter=object_filter, min_confidence=min_confidence)

    def find_objects_in_image(self, image, object_filter = None, min_confidence = 0.4):
        #interpreter = tflite.Interpreter(model_path=self.__model, num_threads=4)
        #interpreter.allocate_tensors()

        #input_details = interpreter.get_input_details()
        #output_details = interpreter.get_output_details()
        #height = input_details[0]['shape'][1]
        #width = input_details[0]['shape'][2]
        #floating_model = False
        #if input_details[0]['dtype'] == np.float32:
        #    floating_model = True

        rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        initial_h, initial_w, channels = rgb.shape

        picture = cv2.resize(rgb, (width, height))

        input_data = np.expand_dims(picture, axis=0)
        if floating_model:
            input_data = (np.float32(input_data) - 127.5) / 127.5

        #interpreter.set_tensor(input_details[0]['index'], input_data)

        #interpreter.invoke()

    input_tensor = tf.convert_to_tensor(image_np)
    input_tensor = input_tensor[tf.newaxis, ...]
    # input_tensor = np.expand_dims(image_np, 0)
    detections = detect_fn(input_tensor)

        detected_boxes = interpreter.get_tensor(output_details[0]['index'])
        detected_classes = interpreter.get_tensor(output_details[1]['index'])
        detected_scores = interpreter.get_tensor(output_details[2]['index'])
        num_boxes = interpreter.get_tensor(output_details[3]['index'])

        detected_objects = []

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
            if object_filter is None or self.__labels[classId] in object_filter:
                score = detected_scores[0][i]
                if score > min_confidence:
                    xmin = left * initial_w
                    ymin = bottom * initial_h
                    xmax = right * initial_w
                    ymax = top * initial_h
                    box = [xmin, ymin, xmax, ymax]
                    x_center = xmin + ((xmax-xmin)/2)
                    y_center = ymin + ((ymax-ymin)/2)
                    #rectangles.append(box)
                    #logging.getLogger(__name__).debug(f"Found {self.__labels[classId]} centered at ({x_center},{y_center}), confidence: {score})")
                    detected_objects.append({
                        'object':self.__labels[classId],
                        'x_center':x_center,
                        'y_center':y_center,
                        'x_min':xmin,
                        'x_max':xmax,
                        'y_min':ymin,
                        'y_max':ymax,
                        'confidence':score
                    })

        return detected_objects

