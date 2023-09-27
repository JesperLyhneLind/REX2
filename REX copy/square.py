# Simpelt eksempel fra absalon
from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")


# send a go_diff command to drive forward

for i in range(12):
    
    #1m forward
    print(arlo.go_diff(33, 33, 1, 1))
    sleep(0.25)
    print(arlo.go_diff(64, 64.3, 1, 1))
    sleep(0.95)
    print(arlo.stop())
    sleep(0.041)

    leftSpeed = 41
    rightSpeed = 41

    # rotate 90 degrees
    print(arlo.go_diff(leftSpeed, rightSpeed, 0,1))
    sleep(0.45)# Wait a bit while robot moves backwards
    print(arlo.stop())
    sleep(0.041)



print("Finished")
