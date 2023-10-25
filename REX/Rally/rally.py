import cv2
import self_localization_ex
#from self_localization_ex import camera
import numpy as np
import robot1

otto = robot1.Robot()

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [1, 2, 3, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (100.0, 0.0)  # Coordinates for landmark 2
}