import cv2
import sys
import numpy as np
import robot
import math
import particle
import selflocalize_method
from time import sleep
from enum import Enum
import camera
import drive_functionality

class Direction(Enum):
    Left = 1
    Right = 2


# Landmarks.
# The robot knows the position of 4 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [1, 2, 3, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),  # Coordinates for landmark 3
    4: (400.0, 300.0)  # Coordinates for landmark 4
}
cam = camera.Camera(0, 'arlo', useCaptureThread = True)

#landmarks_inOrder = [1,2,3,4,1]

def detectLandmark(id):
    for i in range(12):
        colour = cam.get_next_frame()
        # Detect objects
        d_objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        if id in d_objectIDs:
            return 1 # Spotted
        drive_functionality.turn(Direction.Right, 30)
    return 0 # Turned full 360 degrees

def detectObstacle():
    for i in range(12):
        colour = cam.get_next_frame()
        # Detect objects
        d_objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        if not isinstance(d_objectIDs, type(None)):
            return 1 # Spotted
        drive_functionality.turn(Direction.Right, 30)
    return 0 # Turned full 360 degrees
                
        

# Avoids an object and drives the robot 0.3m if there's nothing detected in front of it.
def avoid():
    Left_sensor, Right_sensor, Front_sensor = drive_functionality.check()

    if Left_sensor >= Right_sensor:
        print("Turning left - avoid, 45")
        drive_functionality.turn(Direction.Left, 45)
    else:
        print("Turning right - avoid, 45")
        drive_functionality.turn(Direction.Right, 45)

    drive_functionality.iDrive(0.5)

# Turns the robot and drives towards the goal while avoiding objects.
def driveAlongVec(vecX, vecY, theta):
    distance = math.sqrt(vecX**2 + vecY**2) # pythagorean theorem.
    print("driving " + str(distance-40) + " cm to goal along " + str(vecX) + " " + str(vecY))
    # Let the robot face the goal.
    if np.sign(theta) == 1:
        drive_functionality.turn(Direction.Right, theta) # right.
        print("turning right with degrees ", theta)
    elif np.sign(theta) == -1:
        drive_functionality.turn(Direction.Left, abs(theta)) # left.
        print("turning right with degrees ", abs(theta))

    # Drives the robot towards the goal, while there's longer than 0,4m to the goal.
    if drive_functionality.iDrive((distance-40)/100) == 1:
        print("avoiding")
        avoid()
        return 0 # Ends with avoid
    else:
        print("target reached")
        return 1 # Target reached

        