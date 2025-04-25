import numpy as np 
import cv2 as cv
from time import sleep, time
import pickle
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
import board
import smbus2


''' 
'''

# Load camera calibration information
with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist = pickle.load(f)

# Aruco marker dictionary and detector parameters
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50) # using a 6x6 marker
parameters = cv.aruco.DetectorParameters()





cap = cv.VideoCapture(0) 

currentColor = None
lastColor= None
lcdPrompt = []

last_update_time = time()




def get_color(cnrs):
    for i in range(len(ids)):
        # Get the coordinates of the marker corners
        marker_corners = cnrs[i][0]
        x_min, y_min = np.min(marker_corners, axis=0).astype(int)
        x_max, y_max = np.max(marker_corners, axis=0).astype(int)

        # Define the region to the right and left of the marker (with an offset)
        offset = 100  # Pixels to shift the region to the right or left
        roi_x_min_left = max(0, x_min - offset)
        roi_x_max_left = x_min
        roi_x_min_right = x_max
        roi_x_max_right = min(frame.shape[1], x_max + offset)
        roi_y_min = y_min
        roi_y_max = y_max

        # Extract the region of interest (ROI) for the right side of the marker (red arrow)
        roi_right = frame[roi_y_min:roi_y_max, roi_x_min_right:roi_x_max_right]

        # Extract the region of interest (ROI) for the left side of the marker (green arrow)
        roi_left = frame[roi_y_min:roi_y_max, roi_x_min_left:roi_x_max_left]

        # Convert both ROIs to HSV color space
        hsv_right = cv.cvtColor(roi_right, cv.COLOR_BGR2HSV)
        hsv_left = cv.cvtColor(roi_left, cv.COLOR_BGR2HSV)

        # Define color ranges for red and green in HSV
        red_range_1 = [(0, 120, 70), (10, 255, 255)]  # Lower red range
        red_range_2 = [(170, 120, 70), (180, 255, 255)]  # Upper red range
        green_range = [(40, 40, 40), (80, 255, 255)]  # Green range

        # Threshold the HSV images to detect red and green colors
        red_mask_right_1 = cv.inRange(hsv_right, np.array(red_range_1[0]), np.array(red_range_1[1]))
        red_mask_right_2 = cv.inRange(hsv_right, np.array(red_range_2[0]), np.array(red_range_2[1]))
        red_mask_right = cv.bitwise_or(red_mask_right_1, red_mask_right_2)  # Combine both red ranges

        green_mask_left = cv.inRange(hsv_left, np.array(green_range[0]), np.array(green_range[1]))

        # Check if any pixels are detected for red or green color in the right or left regions
        if np.count_nonzero(red_mask_right) > 0:
            color = "Red"
        elif np.count_nonzero(green_mask_left) > 0:
            color = "Green"
    return color



# Repeat until user hits 'q'
while True: 
    ret, frame = cap.read()
    if not ret:
        continue
    
    # Undistort frame
    frame = cv.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
    
    # Convert to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Detect Aruco markers
    corners, ids, rejected = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is not None:
        currColor = get_color(corners)
        print(currColor)
        
    else:
        currColor = None

    # Show the frame
    cv.imshow("Aruco Detection", frame)
    
    # Exit on key press
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
lcd.clear()
lcd.color = [0,0,0]

print("Done")
