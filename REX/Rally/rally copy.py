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

def turn(dir: Direction, angle: int):
    if dir == Direction.Left:
        print(otto.go_diff(40, 40, 0, 1))
        sleep(angle/65) 
        print(otto.stop())
        sleep(0.18)
    else:
        print(otto.go_diff(40, 40, 1, 0))
        sleep(angle/65)
        print(otto.stop())
        sleep(0.18)

# Landmarks.
# The robot knows the position of 4 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [6, 7, 3, 4]
landmarks = {
    6: (0.0, 0.0),  # Coordinates for landmark 1
    7: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),  # Coordinates for landmark 3
    4: (400.0, 300.0)  # Coordinates for landmark 4
}
landmarks_inOrder = [1,2,3,4,1]
landmarks_index = 0

# Initialize particles.
num_particles = 1000
particles = selflocalize_method.initialize_particles(num_particles)
# The estimate of the robots current pose

# est_pose = particle.estimate_pose(particles)

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

est_pose = particle.estimate_pose(ret_particles)
print("est_pose:", est_pose.getX(), est_pose.getY())

# Skal pakkes ind med noget drive og dit dat
left_sensor, right_sensor, front_sensor = drive_functionality.check() 
if front_sensor < 400: 
    landmarks_index += 1



# otto.go_diff(70,71,1,1)
# sleep(2.6*meters)
# print(otto.stop())
# sleep(0.18)