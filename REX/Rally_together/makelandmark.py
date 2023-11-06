import cv2  # Import the OpenCV library
import numpy as np


# TODO make this configurable from the command line using parser
markerID = 10 # Try 1 - 4

# Define some relevant constants
dpi = 72  # dots (pixels) per inch [inch^(-1)]
inch2mm = 25.4  # [mm / inch]
dpmm = dpi / inch2mm  # Dots per mm [mm^(-1)]

# Define size of output image to fit A4 paper
a4width = 210.0  # [mm]
a4height = 297.0  # [mm]

# Size of output image
width = np.uint(np.round(a4width * dpmm))  # [pixels]
height = np.uint(np.round(a4height * dpmm))  # [pixels]

# Size of ArUco marker
markerPhysicalSize = 150  # [mm]
markerSize = np.uint(np.round(markerPhysicalSize * dpmm))  # [pixels]

# Create landmark image (all white gray scale image)
#landmarkImage = np.ones((width, height), dtype=np.uint8) * np.uint8(255)
landmarkImage = np.ones((height, width), dtype=np.uint8) * np.uint8(255)

# Initialize the ArUco dictionary
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Draw marker
startWidth = int(np.round((width-markerSize)/2))
startHeight = int(np.round((height-markerSize)/2))
landmarkImage[startHeight:int(startHeight+markerSize), startWidth:int(startWidth+markerSize)] = cv2.aruco.drawMarker(arucoDict, markerID, markerSize, 1)
cv2.putText(landmarkImage, str(markerID), (startWidth, startHeight - 60), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,0,0), 2)


# Save image
cv2.imwrite("../../../data/landmark" + str(markerID) + ".png", landmarkImage)

# Show image
cv2.namedWindow("Landmark")
cv2.imshow("Landmark", landmarkImage)
cv2.waitKey()
