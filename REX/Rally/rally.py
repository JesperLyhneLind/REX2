import numpy as np
import robot
import math
import selflocalize_method
from time import sleep
from enum import Enum
import drivingStrategy
import camera


cam = camera.Camera(0, 'arlo', useCaptureThread = True)
otto = robot.Robot()
class Direction(Enum):
    Left = 1
    Right = 2



# Landmarks.
landmarkIDs = [1, 2, 3, 4]
landmarks_inOrder = [1,2,3,4,1]
landmarks_index = 0



# Turns the robot towards goal.
while landmarks_index < 5:
    #Check orientation
    vec_t, vec_x, vec_y = drivingStrategy.orientation(landmarks_inOrder[landmarks_index], cam)
    # If it has seen the goal and drives blindly or the distance has been reached successfully
    if drivingStrategy.driveAlongVec(vec_x, vec_y, vec_t, landmarks_inOrder[landmarks_index], cam) == 1 : 
        print("\n NOW INCREMENTED!!!!! \n")
        landmarks_index += 1
        
