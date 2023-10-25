import numpy as np
import cv2
from cv2 import aruco
import time
from time import sleep
import robot
from enum import Enum
import grids as g
import matplotlib.pyplot as plt
arlo = robot.Robot()
from matplotlib.animation import FFMpegWriter
import robot_models, rrt

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

# print(cam.camera_configuration())  # Print the camera configuration in use

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
    sleep(2.6*meters)
    print(arlo.stop())
    sleep(0.18)

# HUSK:
# SØRGE FOR EN STRATEGI FOR AT FINDE LANDMARKS

def go_to_box(angle_sign, angle, dist, ids):
    # The distance is not accurate and we therefore add a number to it.
    #dist += 355
    print("going to box")
    # print("id: ", ids)
    # print("dist: ", dist)  # in centimeters.
    # print("actual dist:", dist / 1000)  # in meters.
    if angle_sign == -1:
        # print("angle: ", angle)
        # print("angle_sign: ", angle_sign)
        print("turning left with " + str(angle) + " degrees")
        turn(Direction.Left, angle)
        iDrive((dist - 200) / 1000)  # Stops the robot 2 cm before the box.
    elif angle_sign == 1:
        # print("angle: ", angle)
        # print("angle_sign: ", angle_sign)
        print("turning right with " + str(angle) + " degrees")
        turn(Direction.Right, angle)
        iDrive((dist - 200) / 1000)
    else:
        # print("angle: ", angle)
        # print("angle_sign: ", angle_sign)
        print("not turning at all")
        iDrive((dist - 200) / 1000)

#go to box but instead it goes to a point
def go_to_point(old, start, end):
    new_vec = np.array([end[0] - start[0], end[1] - start[1]])
    old_vec =  np.array([start[0] - old[0], start[1] - old[1]])


    new_dist = np.linalg.norm(new_vec)  
    old_dist = np.linalg.norm(old_vec)  
    dot = np.dot(new_vec , old_vec)
    angle = np.degrees(np.arccos(dot/(new_dist*old_dist)))
    print("dist", new_dist)
    angle_sign = np.sign(new_vec[0])  # 1 is right, -1 is left


    if angle_sign == -1:
        # print("angle: ", angle)
        # print("angle_sign: ", angle_sign)
        print("turning left with " + str(angle) + " degrees")
        turn(Direction.Left, angle)
        iDrive((new_dist) / 10)  # Stops the robot 2 cm before the box.
        print("driving " + str(new_dist/10) + " m") 
    elif angle_sign == 1:
        # print("angle: ", angle)
        # print("angle_sign: ", angle_sign)
        print("turning right with " + str(angle) + " degrees")
        turn(Direction.Right, angle)
        print("driving " + str(new_dist/10) + " m")        
        iDrive((new_dist) / 10)
    else:
        # print("angle: ", angle)
        # print("angle_sign: ", angle_sign)
        print("not turning at all")
        iDrive((new_dist) / 10)
        print("driving " + str(new_dist/10) + " m")

    
    

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
            # Making a list of tuples containing ids & points.
            coordinates.append((tvecs[i,0,0]/100, tvecs[i,0,2]/100)) #coordinates in dm
            arucos.append((tvecs[i,0,0], tvecs[i,0,2], ids[i,0])) #coordinates in mm and ids
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
    
        path_res = 1
        map = g.GridOccupancyMap(low=(-10, 0), high=(10, 40), res=path_res)
        map.populate(len(ids), coords_np)

        robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])   
        rrt = rrt.RRT(
            start=[0, 0],
            goal=[0, 39],
            robot_model=robot,
            map=map,
            expand_dis=5,
            path_resolution=path_res,
        )
        
        show_animation = True
        metadata = dict(title="RRT Test")
        writer = FFMpegWriter(fps=2, metadata=metadata)
        fig = plt.figure()
        
        with writer.saving(fig, "rrt_test.mp4", 100):
            path = rrt.planning(animation=show_animation, writer=writer)

            if path is None:
                print("Cannot find path")
            else:
                print("found path!!")
                print("FINAL PATH:", path)
                # Draw final path
                if show_animation:
                    rrt.draw_graph()
                    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                    plt.grid(True)
                    plt.pause(0.01)  # Need for Mac
                    plt.show()
                    writer.grab_frame()

                path.reverse() 
                print("path flipped:", path)
                for i in range(len(path)-1):
                    if i == 0:
                        go_to_point([0,0], path[i], path[i+1])
                    else:
                        go_to_point(path[i-1], path[i], path[i+1])
                            
    else:
        turn(Direction.Right, 45)