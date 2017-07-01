import numpy as np

class Object:
    COLOR_NONE = "None"
    SHAPE_NONE = "None"
    rect = False
    inRange = False
    seq = 0
    exist = False
    def __init__(self):
        self.x = -1
        self.y = -1
        self.w = 0
        self.h = 0
        self.color = self.COLOR_NONE  # Colour should have a name
        self.shape = self.SHAPE_NONE  # Shape should have a name
