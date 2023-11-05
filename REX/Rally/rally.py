import numpy as np
import robot
import math
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
# landmarks = {
#     1: (0.0, 0.0),  # Coordinates for landmark 1
#     2: (0.0, 300.0),  # Coordinates for landmark 2
#     3: (400.0, 0.0),  # Coordinates for landmark 3
#     4: (400.0, 300.0)  # Coordinates for landmark 4
# }
landmarks_inOrder = [1,2,3,4,1]
landmarks_index = 0

# Initialize particles.
num_particles = 1000
particles = selflocalize_method.initialize_particles(num_particles)
# The estimate of the robots current pose

# est_pose = particle.estimate_pose(particles)

# Turns the robot towards goal.
while landmarks_index < 5:
    vec_t, vec_x, vec_y = drivingStrategy.orientation(landmarks_inOrder[landmarks_index])
    #landmarkX, landmarkY = landmarks[landmarks_inOrder[landmarks_index]]
    if drivingStrategy.driveAlongVec(vec_x, vec_y, vec_t) == 1: # Target reached
        vec_theta, vec_posX, vec_posY = drivingStrategy.orientation(landmarks_inOrder[landmarks_index])
        print("Distance ", (math.sqrt(vec_posX**2) + math.sqrt(vec_posY**2)))
        if (math.sqrt(vec_posX**2) + math.sqrt(vec_posY**2)) <= 40:
            print("now going to next goal")
            landmarks_index += 1
        print("not close enough to target")
