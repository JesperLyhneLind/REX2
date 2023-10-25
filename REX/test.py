import robot
import time
from time import sleep
from enum import Enum

arlo = robot.Robot()
class Direction(Enum):
    Left = 1
    Right = 2

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

turn(Direction.Right, 90)