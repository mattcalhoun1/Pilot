import cv2
import numpy as np
try:
    import tflite_runtime.interpreter as tflite
except:
    import tensorflow.lite as tflite
import time
import logging
from recognition.object_locator import ObjectLocator
import math

class TFLiteObjectLocator (ObjectLocator):
    def __init__(self, model_configs = {}, keep_latest_image = True, preload_models = True):
        self.__models = {}
        self.__labels = {}
        self.__model_configs = model_configs
        self.__latest_image = None
        self.__keep_latest_image = keep_latest_image
        
        if preload_models:
            for model_name in model_configs:
                self.__load_model(model_name)

    # unloads models so this locator can be pickled for multiprocessing            
    def unload_models (self):
        self.__models = {}
        self.__labels = {}

    def __load_model (self, model_name):
        if model_name not in self.__labels:
            # ensures the given model / labels are loaded
            self.__labels[model_name] = self.read_label_file(self.__model_configs[model_name]['LabelFile'])
            if self.__model_configs[model_name]['ModelType'] in ['tflite', 'lite']:
                interpreter = tflite.Interpreter(model_path=self.__model_configs[model_name]['ModelFile'], num_threads=4)
                interpreter.allocate_tensors()
                self.__models[model_name] = interpreter
                #logging.getLogger(__name__).info(f"Model {model_name}: TFLite interpreter loaded.")


    def find_objects_on_camera(self, camera, object_filter = None, min_confidence = 0.4):
        captured = camera.capture_image()
        if captured is not None:
            preprocessed = camera.preprocess_image(captured)
            return self.find_objects_in_image(image = preprocessed, object_filter=object_filter, min_confidence=min_confidence)
        logging.getLogger(__name__).warning("Null image received from camera")
        return []

    def get_latest_image (self):
        return self.__latest_image

    def find_objects_in_image_file (self, image_file, object_filter = None, min_confidence = 0.4):
        #image = cv2.imread(image_file)
        image = np.load(image_file)
        return self.find_objects_in_image(image = image, object_filter=object_filter, min_confidence=min_confidence)


    def __get_vertical_slices(self, image):
        slices = []
        initial_h = None
        initial_w = None

        if len(image.shape) == 3:
            initial_h, initial_w, _ = image.shape
        else:
            initial_h, initial_w = image.shape

        #logging.getLogger(__name__).info(f"Full Height: {initial_h}, Width: {initial_w}, Dimensions: {len(image.shape)}")

        v_slices = math.floor(initial_w / initial_h)
        if v_slices > 1:
            #logging.getLogger(__name__).info(f"Slicing image into {v_slices}")
            slices = np.split(image, v_slices, axis=1)
            for s in slices:
                if len(s.shape) == 3:
                    s_h, s_w, _ = s.shape
                else:
                    s_h, s_w = s.shape
                #logging.getLogger(__name__).info(f"Slice Height: {s_h}, Width: {s_w}, Dimensions: {len(s.shape)}")

        else:
            slices.append(image)
        return slices


    def find_objects_in_image(self, image, object_filter = None, min_confidence = 0.4):
        if self.__keep_latest_image:
            self.__latest_image = image # for retrieving later

        initial_h = None
        initial_w = None
        last_width = None
        last_height = None
        last_input_data = None
        last_floating_model = None
        detected_objects = []
        
        for m in self.__model_configs:
            #logging.getLogger(__name__).info(f"Checking model {m} for objects")
            self.__load_model(m)
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
                #logging.getLogger(__name__).info("Converting image, since this is first model pass")
                last_width = width
                last_height = height
                last_floating_model = floating_model
                prepared_slices = [] # image slices resized as necessary for model input

                # picamera2 will have 2d image shape, cv2 will have 3d (channels)
                picture = None
                if len(image.shape) == 3: # cv2 camera
                    # model does better with gray scale
                    initial_h, initial_w = image.shape

                    hires_slices = self.__get_vertical_slices(image=image)
                    for i,img_slice in enumerate(hires_slices):
                        picture = cv2.cvtColor(img_slice, cv2.COLOR_BGR2GRAY)
                        picture = cv2.resize(picture, (width, height), cv2.INTER_CUBIC) # INTER_CUBIC, INTER_AREA, INTER_LINEAR, INTER_NEAREST
                        picture = cv2.cvtColor(picture, cv2.COLOR_GRAY2RGB)
                        cv2.imwrite(f'/tmp/tflite_locator_{i}.png', picture)
                        prepared_slices.append(picture)
                    
                else:
                    initial_h, initial_w = image.shape

                    hires_slices = self.__get_vertical_slices(image=image)
                    for i,img_slice in enumerate(hires_slices):
                        rgb = cv2.cvtColor(img_slice, cv2.COLOR_GRAY2RGB)
                        picture = cv2.resize(rgb, (width, height))
                        cv2.imwrite(f'/tmp/tflite_locator_{i}.png', picture)
                        prepared_slices.append(picture)

                input_data = []
                for input_slice in prepared_slices:
                    expanded = np.expand_dims(input_slice, axis=0)
                    if floating_model:
                        expanded = (np.float32(expanded) - 127.5) / 127.5

                    input_data.append(expanded)


                last_input_data = input_data

            for i, img_part in enumerate(input_data):
                # how many pixels to adjust bounding box by, given we may only be looking at a slice of the image
                slice_width = initial_w / len(input_data)
                horz_pixel_offset = i * slice_width

                # invoke the detection model
                interpreter.set_tensor(input_details[0]['index'], img_part)
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
                            
                            xmin = horz_pixel_offset + (left * slice_width)
                            ymin = top * initial_h
                            xmax = horz_pixel_offset + (right * slice_width)
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

