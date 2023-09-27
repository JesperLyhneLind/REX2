from time import sleep
import cv2
#from cv2 import aruco
import robot
import cv2 # Import the OpenCV library
import time
from pprint import *
#from statemachine import Statemachine, State

#cv2.aruco.Dictionary_get

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)


print("OpenCV version = " + cv2.__version__)

# Open a camera device for capturing
imageSize = (1280, 720)
FPS = 30
cam = picamera2.Picamera2()
frame_duration_limit = int(1/FPS * 1000000) # Microseconds
# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                            controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                            queue=False)
cam.configure(picam2_config) # Not really necessary
cam.start(show_preview=False)

pprint(cam.camera_configuration()) # Print the camera configuration in use

time.sleep(1)  # wait for camera to setup


# Open a window
WIN_RF = "Example 1"
# cv2.namedWindow(WIN_RF)
# cv2.moveWindow(WIN_RF, 100, 100)

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.arucox.DetectorParameters_create()
(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
	parameters=arucoParams)

cnt = 100
while cv2.waitKey(4) == -1:
    print('test')
    text = input('')
    image = cam.capture_array("main")
    corners, ids, rejected_corners = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    print(ids)
    print(corners)
    # Show frames
    # cv2.imshow(WIN_RF, image)
    if text == ' ': #takes picture when pressing space
        print("pressed space")
        filename = 'Pictures/' + str(cnt) + '.jpg'
        cnt = cnt+1
        cv2.imwrite(filename, image)
    elif text == 'q':
        break
# Finished successfully

# Detect markers in an image

