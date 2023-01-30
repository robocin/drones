"""Created by felipe-nunes on 22/09/2022
"""


class VisionOutput:
    def __init__(self, class_id, score, norm, angle):
        self.class_id = class_id
        self.score = score
        self.norm = norm 
        self.angle = angle
