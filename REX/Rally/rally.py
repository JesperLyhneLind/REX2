import cv2
import self_localization_ex
#from self_localization_ex import camera
from self_localization_ex import selflocalize
import numpy as np
import robot1
import math

otto = robot1.Robot()

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
particles = selflocalize.initialize_particles(1000)

# The estimate of the robots current pose
est_pose = selflocalize.particle.estimate_pose(particles)

# The estimate of the robots current pose
robot_pose = selflocalize.particle.estimate_pose(particles) # (x, y, theta)

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