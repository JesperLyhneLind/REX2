import time
from time import sleep
import robot
from enum import Enum

arlo = robot.Robot()

class Direction(Enum):
    Left = 1
    Right = 2

# Checks if there's any object in the path of the robot. 
Front_sensor = arlo.read_front_ping_sensor()
print("Front: " + str(Front_sensor))

