import cv2 # Import the OpenCV library
import time
from pprint import *

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

def gstreamer_pipeline(capture_width=1024, capture_height=720, framerate=30):
    """Utility function for setting parameters for the gstreamer camera pipeline"""
    return (
        "libcamerasrc !"
        "videobox autocrop=true !"
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "videoconvert ! "
        "appsink"
        % (
            capture_width,
            capture_height,
            framerate,
        )
    )

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
WIN_RF = "Focal Length"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

cnt = 40.0
while cv2.waitKey(4) == -1: # Wait for a key pressed event
    retval, frameReference = cam.read() # Read frame

    if not retval: # Error
        print("):< ):< ):< Error >:( >:( >:(")
        exit(-1)

    # Show frames
    cv2.imshow(WIN_RF, frameReference)
    if cv2.waitKey(4) == 32: # Takes picture when pressing space
        print("pressed space")
        path = '../REX/REX/pictures'
        filename = 'pictures/' + str(cnt) + '.jpg'
        cv2.imwrite(filename, frameReference) # Saving the image
        print("The image saved succesfully: " + cv2.imwrite(filename, frameReference))
        print(" ")
        print("After saving image:")  
        cnt += 0.1
        #cnt += 20


# Finished successfully
