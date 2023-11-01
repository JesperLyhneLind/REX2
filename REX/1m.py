# Simpelt eksempel fra absalon
from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")
def iDrive(meters):
    print(arlo.go_diff(70, 74, 1, 1))
    sleep(2.6*meters)
    print(arlo.stop())
    sleep(0.18)

iDrive(3)