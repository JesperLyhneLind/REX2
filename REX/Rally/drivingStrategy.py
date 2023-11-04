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

# Funtion for finding the orientation from the robot towards its next goal in degrees.
def orientation(x, y, theta, id):
    # Calculate the wanted position, that the robot should drive to in order to visit the goal.
    wanted_posX = landmarkIDs[id-1][0] - robot_pose.getX # x-coordinate
    wanted_posY = landmarkIDs[id-1][1] - robot_pose.getY # y-coordinate

    # Calculate the new theta.
    wanted_theta = math.atan2(wanted_posX, wanted_posY)

    # Returns the orientation-vector.
    return math.degrees(wanted_theta), wanted_posX, wanted_posY # degrees instead of radians.

# Checks if the detected ID is in the landmarkIDs.
#def correctID(id):
#    id in landmarkIDs


# Drives the robot and checks which direction to go for avoiding an object.
def avoid(): 
    #arlo.go_diff(70, 71, 1, 1)
    Left_sensor, Right_sensor, Front_sensor = drive_functionality.check()

    if Left_sensor >= Right_sensor:
        print("left")
        turn(Direction.Left, 45)
        Left_sensor, Right_sensor, Front_sensor = drive_functionality.check() # updates the front sensor.
        if Front_sensor < 400:
            avoid()
        else:
            drive_functionality.iDrive(0.3)
            turn(Direction.Right, 90)
            Left_sensor, Right_sensor, Front_sensor = drive_functionality.check() # updates the front sensor.
            drive_functionality.iDrive(1)
            turn(Direction.Left, 45)
            drive_functionality.iDrive(1)
        

    elif Right_sensor > Left_sensor:
        print("Right")
        turn(Direction.Right, 45)
        drive_functionality.iDrive(1)
        turn(Direction.Left, 90)
        drive_functionality.iDrive(1)
        turn(Direction.Right, 45)
        drive_functionality.iDrive(1)
    else:
        pass

# Turns the robot and drives towards the goal while avoiding objects.
def driveToGoal(goalX, goalY, theta):
    distance = math.sqrt(goalX**2 + goalY**2) # pythagorean theorem.

    # Let the robot face the goal.
    if np.sign(theta) == 1:
        drive_functionality.turn(Direction.Right, theta) # left.
    elif np.sign(theta) == -1:
        drive_functionality.turn(Direction.Left, abs(theta)) # right.
    else:
        pass # straight forward.

    left_sensor, right_sensor, front_sensor = drive_functionality.check()
    if front_sensor > 400:
        drive_functionality.iDrive(0.3) # drives 0,3 m.
    else:
        
        


    # Drive towards the goal.
    drive_functionality.iDrive(distance-30)

    
    pass

# Skal pakkes ind med noget drive og dit dat
left_sensor, right_sensor, front_sensor = drive_functionality.check() 
if front_sensor < 400: 
    landmarks_index += 1