import cv2
from cv2 import aruco
import grids as g
import numpy as np

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

cam = picamera2.Picamera2()
image = cam.capture_array("main")

# Defining the ArUCo types.
aruco_type = aruco.DICT_6X6_250
# id = 2
aruco_dict = aruco.Dictionary_get(aruco_type)

params = aruco.DetectorParameters_create()
camMatrix = np.matrix([[1803.766667, 0, 640],  # 612 px = 161.925 mm
                       [0, 1803.766667, 360],   # 360 px = 95.25 mm
                       [0, 0, 1]])

corners, ids, rejected_corners = aruco.detectMarkers(
    image, aruco_dict, parameters=params)

rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(
    corners, 145, camMatrix, None, None)

coordinates = []
coords_np = np.array(coordinates)

path_res = 1
map = g.GridOccupancyMap(low=(-20, 0), high=(20, 20), res=path_res)
map.populate(len(ids), coords_np)


def MCL(x, u, z, map):
    