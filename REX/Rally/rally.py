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

otto = robot.Robot()
class Direction(Enum):
    Left = 1
    Right = 2



# Landmarks.
# The robot knows the position of 4 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [6, 7, 3, 4]
landmarks = {
    6: (0.0, 0.0),  # Coordinates for landmark 1
    7: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),  # Coordinates for landmark 3
    4: (400.0, 300.0)  # Coordinates for landmark 4
}

# Initialize particles.
num_particles = 1000
particles = selflocalize_method.initialize_particles(num_particles)
# The estimate of the robots current pose

# est_pose = particle.estimate_pose(particles)

# The estimate of the robots current pose
robot_pose = particle.estimate_pose(particles) # (x, y, theta)





ret_particles = selflocalize_method.self_localize(landmarks, landmarkIDs, num_particles, particles)

est_pose = particle.estimate_pose(ret_particles)
print("est_pose:", est_pose.getX(), est_pose.getY())


