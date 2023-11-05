import cv2
import sys
import numpy as np
import robot
import math
import particle
import selflocalize_method
from time import sleep
from enum import Enum
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
landmarks_inOrder = [1,2,3,4,1]

# Funtion for finding the orientation from the robot towards its next goal in degrees.
def orientation(id_index):
    # The estimate of the robots current pose
    robot_pose = selflocalize_method.self_localize(landmarks, landmarkIDs)

    # Calculate the vector, that the robot should drive to in order to visit the goal.
    vec_posX = (landmarks[id_index])[0] - robot_pose.getX() # x-coordinate
    vec_posY = (landmarks[id_index])[1] - robot_pose.getY() # y-coordinate

    # Calculate the new theta.
    vec_theta = math.atan2(vec_posX, vec_posY)

    # Returns the orientation-vector.
    return math.degrees(vec_theta), vec_posX, vec_posY # degrees instead of radians.

# Avoids an object and drives the robot 0.3m if there's nothing detected in front of it.
def avoid():
    Left_sensor, Right_sensor, Front_sensor = drive_functionality.check()

    if Left_sensor >= Right_sensor:
        print("Turning left")
        drive_functionality.turn(Direction.Left, 45)
    else:
        print("Turning right")
        drive_functionality.turn(Direction.Right, 45)

    drive_functionality.iDrive(0.5)

# Turns the robot and drives towards the goal while avoiding objects.
def driveToGoal(goalX, goalY, theta):
    distance = math.sqrt(goalX**2 + goalY**2) # pythagorean theorem.
    print("driving " + str(distance) + "m to goal in " + str(goalX) + str(goalY))

    # Let the robot face the goal.
    if np.sign(theta) == 1:
        drive_functionality.turn(Direction.Right, theta) # left.
    elif np.sign(theta) == -1:
        drive_functionality.turn(Direction.Left, abs(theta)) # right.

    # Drives the robot towards the goal, while there's longer than 0,4m to the goal.
    if drive_functionality.iDrive((distance-40)/100) == 1:
        print("avoiding")
        avoid()
        return 0 # Ends with avoid
    else:
        print("target reached")
        return 1 # Target reached

        