import cv2
import sys
import numpy as np
import robot
import math
import particle
import drive_functionality
import selflocalize_method
from time import sleep
from enum import Enum
import drivingStrategy

otto = robot.Robot()
class Direction(Enum):
    Left = 1
    Right = 2



# Landmarks.
# The robot knows the position of 4 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [1, 2, 3, 4]

landmarks_inOrder = [1,2,3,4,1]
landmarks_index = 0

while landmarks_index < 5:
    if drivingStrategy.detectLandmark(landmarks_inOrder[landmarks_index]) == 1: #spotted
        drive_functionality.iDrive()
    #landmarkX, landmarkY = landmarks[landmarks_inOrder[landmarks_index]]
    if drivingStrategy.driveAlongVec(vec_x, vec_y, vec_t) == 1: # Target reached
        landmarks_index += 1
