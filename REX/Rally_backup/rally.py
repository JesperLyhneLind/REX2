import cv2
import sys
import numpy as np
import robot
import math
import particle
import drive_functionality
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
    if drivingStrategy.detectLandmark(landmarks_inOrder[landmarks_index]) == 1: # Spotted landmark
        print("Spotted landmark - driving to")
        while drive_functionality.iDrive(1) == 0: # Kachowwwwwwwwww skrrrtt swag lol yolo brrr brrr 888 kesi
            print("Target not reached - i drive more")
        print("Target reached - next landmark")
        landmarks_index += 1
        break
    else: 
        print("Finding obstacles")
        if drivingStrategy.detectObstacle == 1: # Spotted obstacle
            print("Driving to(wards) obstacle")
            if drive_functionality.iDrive(0.5) == 1: # Stopped due to obstacle
               print("Obstacle detected - avoiding")
               drivingStrategy.avoid() 
                

        
    
