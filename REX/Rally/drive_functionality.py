import time
from time import sleep
import robot
from enum import Enum
import selflocalize_method

arlo = robot.Robot()

class Direction(Enum):
    Left = 1
    Right = 2

# Checks if there's any object in the path of the robot.  
def check():
    while(True):
        Front_sensor = arlo.read_front_ping_sensor()
        Right_sensor = arlo.read_right_ping_sensor()
        Left_sensor = arlo.read_left_ping_sensor()

        if Left_sensor < 300 or Right_sensor < 300 or Front_sensor < 400:
            print("Left: " + str(Left_sensor))
            print("Front: " + str(Front_sensor))
            print("Right: " + str(Right_sensor))
            return Left_sensor, Right_sensor, Front_sensor
        
# Turns the robot angle degrees.
def turn(dir: Direction, angle: int):
    if dir == Direction.Left:
        print(arlo.go_diff(40, 40, 0, 1))
        sleep(angle/65) 
        print(arlo.stop())
        sleep(0.18)
    else:
        print(arlo.go_diff(40, 40, 1, 0))
        sleep(angle/65)
        print(arlo.stop())
        sleep(0.18)

# Drives one meter.
def iDrive(meters):
    print(arlo.go_diff(70, 71, 1, 1))
    sleep(2.6*meters)
    print(arlo.stop())
    sleep(0.18)

# Drives the robot and checks which direction to go for avoiding an object.
def drive(): 
    arlo.go_diff(70, 71, 1, 1)
    Left_sensor, Right_sensor, Front_sensor = check()

    if Left_sensor >= Right_sensor:
        print("left")
        turn(Direction.Left, 45)
        iDrive(1)
        turn(Direction.Right, 90)
        iDrive(1)
        turn(Direction.Left, 45)
        iDrive(1)
        

    elif Right_sensor > Left_sensor:
        print("Right")
        turn(Direction.Right, 45)
        iDrive(1)
        turn(Direction.Left, 90)
        iDrive(1)
        turn(Direction.Right, 45)
        iDrive(1)
    else:
        pass
