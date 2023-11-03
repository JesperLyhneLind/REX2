import cv2
import sys
import numpy as np
import robot
import math
import particle
import detection
import selflocalize_method
import time
from time import sleep
from enum import Enum

otto = robot.Robot()
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

# Initialize particles.
num_particles = 1000
particles = selflocalize_method.initialize_particles(num_particles)
# The estimate of the robots current pose

est_pose = particle.estimate_pose(particles)

# The estimate of the robots current pose
robot_pose = particle.estimate_pose(particles) # (x, y, theta)

# Funtion for finding the orientation vector from the robot towards its next goal.
def orientation_vector(x, y, theta):
    if landmarkIDs == 1:

        # Calculate the relative vector.
        dx = landmarks[1][0] - robot_pose.getX
        dy = landmarks[1][1] - robot_pose.getY

        # Calculate the magnitude of the vector.
        magnitude = math.sqrt(dx**2 + dy**2)

        # Normalize the vector by dividing each component by the magnitude.
        dx_norm = dx / magnitude
        dy_norm = dy / magnitude

        # Returns the orientation-vector.
        return (dx_norm, dy_norm)

ret_particles = selflocalize_method.self_localize(landmarks, landmarkIDs, num_particles, particles)

time1 = time.time()
while True:
    est_pose = particle.estimate_pose(ret_particles)
    print("est_pose:", est_pose.getX(), est_pose.getY())
    detection.turn(Direction.Right, 30)
    time2 = time.time()
    if time1 - time2 > 40:
        break

# otto.go_diff(70,71,1,1)
# sleep(2.6*meters)
# print(otto.stop())
# sleep(0.18)