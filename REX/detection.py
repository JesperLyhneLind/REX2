import numpy as np
import cv2
from cv2 import aruco
import time
from time import sleep
import robot
from enum import Enum
import grids as g
import matplotlib.pyplot as plt
# import statemachine
arlo = robot.Robot()

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

# Open a camera device for capturing
#imageSize = (624, 352)
imageSize = (1280, 720)
FPS = 30
cam = picamera2.Picamera2()
frame_duration_limit = int(1/FPS * 1000000)  # Microseconds
# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                               controls={"FrameDurationLimits": (
                                                   frame_duration_limit, frame_duration_limit)},
                                               queue=False)
cam.configure(picam2_config)  # Not really necessary
cam.start(show_preview=False)

print(cam.camera_configuration())  # Print the camera configuration in use

WIN_RF = "Ottos camera"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

time.sleep(1)  # wait for camera to setups

# Defining the ArUCo types.
aruco_type = aruco.DICT_6X6_250
# id = 2
aruco_dict = aruco.Dictionary_get(aruco_type)


class Direction(Enum):
    Left = 1
    Right = 2

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

# This method fucking drives ONE meter.
def iDrive(meters):
    print(arlo.go_diff(70, 70, 1, 1))
    sleep(2.2*meters)
    print(arlo.stop())
    sleep(0.18)

# HUSK:
# SØRGE FOR EN STRATEGI FOR AT FINDE LANDMARKS

def go_to_box(angle_sign, angle, dist, ids):
    # The distance is not accurate and we therefore add a number to it.
    #dist += 355
    print("going to box")
    print("id: ", ids)
    print("dist: ", dist)  # in centimeters.
    print("actual dist:", dist / 1000)  # in meters.
    if angle_sign == -1:
        print("angle: ", angle)
        print("angle_sign: ", angle_sign)
        print("turning left with " + str(angle) + " degrees")
        turn(Direction.Left, angle)
        iDrive((dist - 200) / 1000)  # Stops the robot 2 cm before the box.
    elif angle_sign == 1:
        print("angle: ", angle)
        print("angle_sign: ", angle_sign)
        print("turning right with " + str(angle) + " degrees")
        turn(Direction.Right, angle)
        iDrive((dist - 200) / 1000)
    else:
        print("angle: ", angle)
        print("angle_sign: ", angle_sign)
        print("not turning at all")
        iDrive((dist - 200) / 1000)


params = aruco.DetectorParameters_create()
camMatrix = np.matrix([[1803.766667, 0, 640],  # 612 px = 161.925 mm
                       [0, 1803.766667, 360],   # 360 px = 95.25 mm
                       [0, 0, 1]])

while cv2.waitKey(4) == -1:  # Wait for a key pressed event
    # retval, frameReference = cam.read() # Read frame
    image = cam.capture_array("main")
    cv2.imshow(WIN_RF, image)
    cv2.waitKey(1)
    counter = 0

    corners, ids, rejected_corners = aruco.detectMarkers(
        image, aruco_dict, parameters=params)

    rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(
        corners, 145, camMatrix, None, None)
    z_vector = np.array([0, 0, 1])

    if tvecs is not None:
        # print(arlo.stop())
        arucos = []  # List for containing all ArUCo codes.
        coordinates = []
        #  coordinates = np.zeros(1, len(ids), 2)
        for i in range(len(ids)):
            print("")
            # Converting the array of lists of lists (corners) to a list.
            # print("tvecs: ", tvecs[i,0,0], tvecs[i,0,2])
            # print("ids: ", list(ids[i]))
            # print("")
            # Making a list of tuples containing ids & points.
            coordinates.append((tvecs[i,0,0]/100, tvecs[i,0,2]/100)) #coordinates in dm
            arucos.append((tvecs[i,0,0], tvecs[i,0,2], ids[i,0]))
            print("list of ArUCos: \n", arucos)       
        norms = []
        coords_np = np.array(coordinates)
        print("coords_np: ", coords_np)
        for i in range(len(tvecs)):
            norms.append(np.linalg.norm(tvecs[i]))

        maxvecidx = int(norms.index(min(norms)))
        vec = tvecs[norms.index(min(norms))][0]  # choose the closest vector
        dist = np.linalg.norm(vec)  # distance to the box
        dot = np.dot((vec / dist), z_vector)
        angle = np.degrees(np.arccos(dot))
        angle_sign = np.sign(vec)  # 1 is right, -1 is left
    
        map = g.GridOccupancyMap()
        map.populate(len(ids), coords_np)
        plt.clf()
        map.draw_map()
        plt.show()

        go_to_box(angle_sign[0], angle, dist, ids[maxvecidx])
    else:
        turn(Direction.Right, 45)
