"""Created by felipe-nunes on 22/09/2022
"""

from rcpilot.abstract_modules.vision_base import VisionBase
import numpy as np
from PIL import Image

MIN_SCORE = 0
WIDTH = 300
HEIGHT = 300

class Vision:
    def __init__(self) :
        pass

    def execute(self):
        img_array = self._get_image()
        output_dict = self._get_output_dict()
        output = self._filter_boxes(output_dict)

        vectors = self.create_vectors(img_array, output)


    def _get_image(self):
        img = Image.open('rcpilot/modules/test_images/img1.jpg')
        #img = self._resize_img(img)

        img_array = np.asarray(img)

        return img_array

    def _get_output_dict(self):
        output_dict = {
            'num_detections': 3, 
            # bounding boxes - (y_min, x_min, y_max, x_max)
            'detection_boxes': [[106, 213, 332, 363], [184, 10, 417, 273], [10, 100, 20, 123]], 
            # Realiability parameters
            'detection_scores': [0.89, 0.88, 0.4],
            # 2 - small_base; 1 - larger_base; 3 - smaller_base
            'detection_classes': [2, 1, 3]
        }
        return output_dict

    def _resize_img(self, img):
        # cv2.resize(img, dsize=(54, 140), interpolation=cv2.INTER_CUBIC)
        img_resized = img.resize((WIDTH, HEIGHT))
        
        return img_resized


    def _filter_boxes(self, output_dict):
        output = {
            'classes': [],
            'boxes': [],
            'scores': []
        }
        for i in range(output_dict['num_detections']):
            if output_dict['detection_scores'][i] >= MIN_SCORE:
                output['scores'].append(output_dict['detection_scores'][i])
                output['boxes'].append(output_dict['detection_boxes'][i])
                output['classes'].append(output_dict['detection_classes'][i])
        
        return output
    
    def create_vectors(self, img_array, output):
        # TODO: verify img_array shape format
        width, height, channels = img_array.shape
        
        vectors = []

        # image center
        origin = (width/2, height/2)

        axis_x = (origin[0]+10, origin[1]) # 10 = arbitrary number to create axis aligned vector

        for i in range(len(output['classes'])):
            y_min, x_min, y_max, x_max = output['boxes'][i]

            box_center = ((x_max + x_min)/2, (y_max + y_min)/2)
            # class, score, size, angle
            vectors.append((output['classes'][i], 
                            output['scores'][i],
                            self._vec_size(origin, box_center),
                            self._get_angle(box_center, axis_x, origin)))
        
        print(vectors)
        return vectors

    #return vector norm
    def _vec_size(self, origin, dest):
        x = dest[0]-origin[0]
        y = dest[1]-origin[1]

        return np.sqrt(x*x + y*y)

    def _get_angle(self, point, axis_x, origin):
        point = list(point)
        point[0] -= origin[0]
        point[1] -= origin[1]

        axis_x = list(axis_x)
        axis_x[0] -= origin[0]
        axis_x[1] -= origin[1]

        x1, y1 = point
        x2, y2 = axis_x


        dot_product = x1*x2 + y1*y2
        size1 =  np.linalg.norm(point)
        size2 =  np.linalg.norm(axis_x)

        res = float(dot_product) / (size1*size2)
        angle = np.arccos(res)

        # 3º and 4º quadrant
        if(y1 > 0):
            angle = 2*np.pi - angle 

        return angle

    def direction_vectors():
        pass    
    
    

    