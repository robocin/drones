"""Created by felipe-nunes on 22/09/2022
"""
import time

import cv2
from rcpilot.abstract_modules.vision_base import VisionBase
from rcpilot.packages.vision_output import VisionOutput
import numpy as np
from PIL import Image

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput

from rcpilot.environment import Vision

WIDTH = Vision.IMAGE_WIDTH
HEIGHT = Vision.IMAGE_HEIGHT
THRESHOLD = Vision.SCORE_THRESHOLD

class Vision(VisionBase):
    def __init__(self) :
        self._output = None

        self.net = detectNet(argv=['--model=rcpilot/modules/models/ssd-mobilenet.onnx', '--labels=rcpilot/modules/models/labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'], threshold=0.7)

        #image is 640x480
        self.camera = videoSource("/dev/video0", [f'--input-width={WIDTH}', f'--input-height={HEIGHT}'])

        self.display = videoOutput("display://0")


    def _share_package(self):
        print(self._output)


    def execute(self):
        #img, vectors = self._share_package()
        try:
            img = self.camera.Capture()
            detections = self.net.Detect(img)
            filtered_detections = self._filter_detections(detections)
            img, vectors = self.create_vectors(img, filtered_detections)
            self._output = []
            for vector in vectors:  
                self._output.append(VisionOutput(
                    vector[0],
                    vector[1],
                    vector[2],
                    vector[3]))

            # self.display.Render(img)
            # self.display.SetStatus("Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))
        except:
            self._output = None
            

            

    def _filter_detections(self, detections):
        filtered_detections = []
        for detection in detections:
            if detection.Confidence > THRESHOLD:
                filtered_detections.append(detection)
        
        return filtered_detections
    
    def create_vectors(self, img, detections):
        
        vectors = []

        # image center
        origin = np.array([WIDTH/2, HEIGHT/2])

        axis_x = np.array([10, 0]) # 10 = arbitrary number to create axis aligned vector

        for detection in detections:
            # class, score, norm, angle
            relative_center = np.array(detection.Center) - origin
            # ClassID - 1 = large, 2 = small
            vectors.append((detection.ClassID, 
                            detection.Confidence,
                            np.linalg.norm(relative_center),
                            self._get_angle(relative_center, axis_x)))
        
        print(vectors)
        return img, vectors

    def _get_angle(self, box_center, axis_x):

        size1 =  np.linalg.norm(box_center)
        size2 =  np.linalg.norm(axis_x)

        res = float(np.dot(box_center, axis_x)) / (size1*size2)
        angle = np.arccos(res)

        # 3º and 4º quadrant
        if(box_center[1] > 0):
            angle = 2*np.pi - angle 

        return angle

    def direction_vectors():
        pass    
    
    

    