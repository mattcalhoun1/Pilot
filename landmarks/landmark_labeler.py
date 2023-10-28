import cv2
import logging

class LandmarkLabeler:
    def __init__(self):
        pass


    # expects landmark to be a dict, such as:
    # {
    #  'w_tree': {
    #    'id': 'w_tree', 
    #    'time': 1692359312.4, 
    #    'x1': 452.88457949956256, 
    #    'x2': 696.5187883377075, 
    #    'y1': 446.5919737815857, 
    #    'y2': 770.3363962173462, 
    #    'confidence': 0.99972785
    #  },
    # }
    # NOTE: This may alter the image, so any analysis after this will include the bounding boxes
    def export_labeled_image (self, image, landmarks, distances, angles, file_name):
        boxed_image = self.__add_boxes(image, landmarks, distances, angles)
        
        # write out to file_name
        cv2.imwrite(file_name, boxed_image)
    
    def __add_boxes (self, image, landmarks, distances, angles):
        
        for lid in landmarks:
            # add a line
            start_point = (int(landmarks[lid]['x1']), int(landmarks[lid]['y1']))
            end_point = (int(landmarks[lid]['x2']), int(landmarks[lid]['y2']))
            color = ((int)(255 * landmarks[lid]['confidence']), 0, 0)
            thickness = 2            
            image = cv2.rectangle(image, start_point, end_point, color, thickness)

            # add text label
            font = cv2.FONT_HERSHEY_SIMPLEX
            location = (int(landmarks[lid]['x1'] + 25), int(landmarks[lid]['y1'] + 15))
            fontScale = 1
            label_text = f"{lid} (~{round(distances[lid]['ground'],1)})"
            image = cv2.putText(image, label_text, location, font, 
                   fontScale, color, thickness, cv2.LINE_AA)
            
            
            crosshair_height = 20
            crosshair_width = 20
            crosshair_color = (0,0,255)
            crosshair_thickness = 1
            x_center = int(((landmarks[lid]['x2'] - landmarks[lid]['x1']) / 2) + landmarks[lid]['x1'])
            y_center = int(((landmarks[lid]['y2'] - landmarks[lid]['y1']) / 2) + landmarks[lid]['y1'])
            # vertical crosshair
            image = cv2.line(
                image, 
                (x_center,y_center - crosshair_height), 
                (x_center,y_center + crosshair_height), 
                crosshair_color, 
                crosshair_thickness) 

            # horizontal crosshair
            image = cv2.line(
                image, 
                (x_center - crosshair_width,y_center), 
                (x_center + crosshair_width,y_center), 
                crosshair_color, 
                crosshair_thickness) 
            
        return image
