import cv2
import numpy as np
#import tflite_runtime.interpreter as tflite
import tensorflow as tf
import time
import logging
from recognition.object_locator import ObjectLocator

# This is way to memory intensive for jetson nano, dont bother
class TFObjectLocator (ObjectLocator):
    def __init__(self, model_configs = {}, keep_latest_image = True):
        self.__models = {}
        self.__labels = {}
        self.__model_configs = model_configs
        self.__latest_image = None
        self.__keep_latest_image = keep_latest_image
        
        for model_name in model_configs:
            self.__labels[model_name] = self.read_label_file(model_configs[model_name]['LabelFile'])
            if model_configs[model_name]['ModelType'] == 'tf':
                self.__models[model_name] = tf.saved_model.load(model_configs[model_name]['ModelFile'])
                logging.getLogger(__name__).info(f"Model {model_name}: TF model loaded.")
            else:
                raise Exception (f"Unknown model type: {model_configs[model_name]['ModelType']}")

    def find_objects_on_camera(self, camera, object_filter = None, min_confidence = 0.4):
        return self.find_objects_in_image(image = camera.capture_image(), object_filter=object_filter, min_confidence=min_confidence)

    def find_objects_in_image(self, image, object_filter = None, min_confidence = 0.4):
        outputs = []
        for m in self.__models:
            output_dict = self.__run_inference_for_single_image(model = self.__models[m], image=image)
            logging.getLogger(__name__).info(f"Output: {output_dict}")
            outputs.append(output_dict)
        
        return outputs
        
        
    def __run_inference_for_single_image(self, model, image):
      #image = np.asarray(image)
      # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
      input_tensor = tf.convert_to_tensor(image)
      # The model expects a batch of images, so add an axis with `tf.newaxis`.
      input_tensor = input_tensor[tf.newaxis,...]

      # Run inference
      model_fn = model.signatures['serving_default']
      output_dict = model_fn(input_tensor)

      # All outputs are batches tensors.
      # Convert to numpy arrays, and take index [0] to remove the batch dimension.
      # We're only interested in the first num_detections.
      num_detections = int(output_dict.pop('num_detections'))
      output_dict = {key:value[0, :num_detections].numpy() 
                     for key,value in output_dict.items()}
      output_dict['num_detections'] = num_detections

      # detection_classes should be ints.
      output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)
       
      # Handle models with masks:
      if 'detection_masks' in output_dict:
        # Reframe the the bbox mask to the image size.
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                  output_dict['detection_masks'], output_dict['detection_boxes'],
                   image.shape[0], image.shape[1])      
        detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                           tf.uint8)
        output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()
        
      return output_dict

