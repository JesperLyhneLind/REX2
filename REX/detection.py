import numpy as np
import cv2
from cv2 import aruco
import time
from time import sleep
import robot
from enum import Enum
# import statemachine
arlo = robot.Robot()

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

# Open a camera device for capturing
#imageSize = (1280, 720)
imageSize = (624, 352)
FPS = 15
cam = picamera2.Picamera2()
frame_duration_limit = int(1/FPS * 1000000) # Microseconds
# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                            controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                            queue=False)
cam.configure(picam2_config) # Not really necessary
cam.start(show_preview=False)

print(cam.camera_configuration()) # Print the camera configuration in use

time.sleep(1)  # wait for camera to setups


WIN_RF = "Ottos camera"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)


# Defining the ArUCo types.
aruco_type = aruco.DICT_6X6_250
#id = 2
aruco_dict = aruco.Dictionary_get(aruco_type)

class Direction(Enum):
    Left = 1
    Right = 2

# Turns the robot angle degrees.
def turn(dir: Direction, angle: int):
    if dir == Direction.Left:
        print(arlo.go_diff(49, 49, 0, 1))
        sleep(angle/90)
        print(arlo.stop())
        sleep(0.041)
    else:
        print(arlo.go_diff(49, 49, 1, 0))
        sleep(angle/90)
        print(arlo.stop())
        sleep(0.041)

# Drives one meter.
def driveM(meters):
    leftSpeed = 70
    rightSpeed = 70
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    # Wait a bit while robot moves forward
    sleep(1.7*meters)
    # send a stop command
    print(arlo.stop())   
    sleep(0.041)  

def go_to_box(angle_sign, angle, dist, ids):
        print("going to box")
        print("id: ", ids)
        print("dist: ", dist)
        print("actual dist:", (dist - 50) / 10)
        if angle_sign == -1:
            turn(Direction.Left, angle) 
            driveM((dist - 50) / 100) #drive to box with 50 cm to spare
            print(arlo.stop()) 
        elif angle_sign == 1:
            turn(Direction.Right, angle)
            driveM((dist - 50) / 100)
            print(arlo.stop()) 
        else:
            driveM((dist - 50) / 100)
            print(arlo.stop()) 

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    # retval, frameReference = cam.read() # Read frame
    image = cam.capture_array("main")

    # if not image: # Error
        
    #     print("Error!")
    #     exit(-1)

    # Show frames
    cv2.imshow(WIN_RF, image)

    params = aruco.DetectorParameters_create()
    corners, ids, rejected_corners = aruco.detectMarkers(image, aruco_dict, parameters=params)
    camMatrix = np.matrix([[459.3346823, 0, 612], # 612 px = 161.925 mm
                           [0, 459.3346823, 360],   # 360 px = 95.25 mm
                           [0, 0, 1]])

    rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(corners, 145, camMatrix, None, None)
    z_vector = np.array([0, 0, 1])

    if tvecs is not None:
        # print(arlo.stop()) 
        norms = []
        for i in range(len(tvecs)):
            norms.append(np.linalg.norm(tvecs[i]))
        maxvecidx = int(norms.index(min(norms)))
        vec = tvecs[norms.index(min(norms))][0] #choose the closest vector
        dist = np.linalg.norm(vec) #distance to the box
        dot = np.dot((vec / dist), z_vector)
        angle = np.degrees(np.arccos(dot))
        angle_sign = np.sign(vec) # 1 is right, -1 is left
        print("tvecs:",tvecs)
        print("norms:",norms)
        print("maxvecidx:",maxvecidx)
        print("vec:",vec)
        print("angle:", angle)
        print("ids", ids)
        go_to_box(angle_sign[0], angle, dist, ids[maxvecidx])

    else:
        turn(Direction.Right, 40)
        sleep(3)
 
#print("ArUCo type '{}' with ID '{}".format(aruco_type, id))
#tag_size = 600
#tag = np.zeros((tag_size, tag_size, 1), dtype="uint8")
#aruco.drawMarker(aruco_dict, id, tag_size, tag, 1)

#tag_name = "arucoMarkers/" + str(aruco_type) + "_" + str(id) + ".png"
#cv2.imwrite(tag_name, tag)