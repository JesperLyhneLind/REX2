# Simpelt eksempel fra absalon
from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")


# send a go_diff command to drive forward

for i in range(1):
    
    speed = 53
    # rotate 90 degrees
    print(arlo.go_diff(speed, 0, 0, 0))
    sleep(4)
    print(arlo.go_diff(speed, 0, 0, 0))
    sleep(4)



print("Finished")



