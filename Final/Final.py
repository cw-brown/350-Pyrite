import numpy as np 
import cv2 as cv
from time import sleep, time
import pickle
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
import board
import smbus2
import struct


''' 
'''
# Set the feet from marker to start detecting the color of the arrow
COLOR_DETECTION_THRESHOLD = 2

# Skip any markers outside of 5 feet
MARKER_THRESHOLD = 5
# Arduino I2C address. Verify with i2cdetect 1 in the pi terminal
ARD_ADDR = 8  

# I2C declaration for arduino
i2cARD = smbus2.SMBus(1)  # Use I2C bus 1 for communication with Arduino

# Camera Calibration and Distortion Matricies
with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist = pickle.load(f)

# Aruco marker dictionary and detector parameters (6x6 marker 1)
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
parameters = cv.aruco.DetectorParameters()

# init the camera
cap = cv.VideoCapture(0)

# Angle and distance place holders
currAngle = 99.9
lastAngle = 99.9

# Marker detection flag (0 = No markers detected, 1 = markers detected)
markerFound = 0

# Arrow detection flag ( 0 = No arrow, 1 = Right, 2 = Left)
arrow = 0

# Implement time based sending
last_update_time = time()

# find the angle off the center axis of rotation. return an angle to 1 decimal point of accuracy      
def get_angle(rvecs, tvecs):
##    print("Angle")
    for i in range(len(ids)):
        rvec = rvecs[i][0]
        tvec = tvecs[i][0]
        
        # Compute rotation matrix
        R, _ = cv.Rodrigues(rvec)
        
        # Angle between camera axis and marker
        angle = -1*np.degrees(np.arctan2(tvec[0], tvec[2]))
        angle = float(round(angle, 1)) # round the angle to 2 decimal points.
    return angle

# find the distance to the marker retrun a distance of 1 decimal point of accuracy
def get_distance(rvecs, tvecs):
##    print("Distance")
    for i in range(len(ids)):
        rvec = rvecs[i][0]
        tvec = tvecs[i][0]
        distance = float(round(tvec[2]*3.28,1))  # Convert meters to feet
    return distance

# find the color on the marker
def get_color(cnrs, frame, ids):
##    print("color")
    for i in range(len(ids)):
        color = None
        # Get the coordinates of the marker corners
        marker_corners = cnrs[i][0]
        x_min, y_min = np.min(marker_corners, axis=0).astype(int)
        x_max, y_max = np.max(marker_corners, axis=0).astype(int)

        # Define the region to the right and left of the marker (with an offset)
        offset = 70  # Pixels to shift the region to the right or left
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
        hsv_right = cv.cvtColor(roi_right, cv.COLOR_BGR2HSV) if roi_right.size > 0 else None
        hsv_left = cv.cvtColor(roi_left, cv.COLOR_BGR2HSV) if roi_left.size > 0 else None
##        print(hsv_left)
        # Define color ranges for red and green in HSV
        red_range_1 = [(0, 120, 70), (5, 255, 255)]  # Lower red range
        red_range_2 = [(170, 120, 70), (175, 255, 255)]  # Upper red range
        green_range = [(50, 40, 40), (70, 255, 255)]  # Green range

        # Threshold the HSV images to detect red and green colors
        red_mask_right = None
        green_mask_left = None
        if hsv_right is not None:
            red_mask_right_1 = cv.inRange(hsv_right, np.array(red_range_1[0]), np.array(red_range_1[1]))
            red_mask_right_2 = cv.inRange(hsv_right, np.array(red_range_2[0]), np.array(red_range_2[1]))
            red_mask_right = cv.bitwise_or(red_mask_right_1, red_mask_right_2)  # Combine both red ranges
        
        if hsv_left is not None:
            green_mask_left = cv.inRange(hsv_left, np.array(green_range[0]), np.array(green_range[1]))
        # Check if any pixels are detected for red or green color in the right or left regions
        if red_mask_right is not None and np.count_nonzero(red_mask_right) > 0:
            color = 1 #Red
##            print("-90.0 / Red")
        elif green_mask_left is not None and np.count_nonzero(green_mask_left) > 0:
            color = 2 #Green
##            print("90.0 / Green")
        else:
            color = 0 # Whtie
##    cropped_frame = frame[roi_y_min:roi_y_max, roi_x_min_left:roi_x_max_right]
##    cv.imshow("Aruco Detection", cropped_frame)
##    print(color)
    return color

##def get_color_code(cnrs, frame, ids):
##    # Initialize locked_color on first call
##    if not hasattr(get_color_code, "locked_color"):
##        get_color_code.locked_color = 0
##
##    # If no marker, reset lock and return 0
####    if ids is None or len(ids) == 0:
####        get_color_code.locked_color = 0
####        return 0
##
##    # Process only the first detected marker
##    marker_corners = cnrs[0][0]
##    x_min, y_min = np.min(marker_corners, axis=0).astype(int)
##    x_max, y_max = np.max(marker_corners, axis=0).astype(int)
##
##    offset = 70
##    roi_left  = frame[y_min:y_max, max(0, x_min-offset):x_min]
##    roi_right = frame[y_min:y_max, x_max:min(frame.shape[1], x_max+offset)]
##
##    # Gaussian blur to reduce noise
##    roi_left_blur  = cv.GaussianBlur(roi_left,  (7, 7), 0) if roi_left.size  > 0 else None
##    roi_right_blur = cv.GaussianBlur(roi_right, (7, 7), 0) if roi_right.size > 0 else None
##
##    hsv_left  = cv.cvtColor(roi_left_blur,  cv.COLOR_BGR2HSV) if roi_left_blur  is not None else None
##    hsv_right = cv.cvtColor(roi_right_blur, cv.COLOR_BGR2HSV) if roi_right_blur is not None else None
##
##    # HSV ranges
##    red_range_1   = [(0, 120, 70),   (5,   255, 255)]
##    red_range_2   = [(170, 120, 70), (179, 255, 255)]
##    green_range   = [(50, 40, 40),   (70,  255, 255)]
##    area_thresh   = 300
##
##    raw_color = 0  # default: no arrow
##
##    # Detect red on the right
##    if hsv_right is not None:
##        r1 = cv.inRange(hsv_right, np.array(red_range_1[0]), np.array(red_range_1[1]))
##        r2 = cv.inRange(hsv_right, np.array(red_range_2[0]), np.array(red_range_2[1]))
##        red_mask = cv.bitwise_or(r1, r2)
##        red_cnts, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
##        red_area = sum(cv.contourArea(c) for c in red_cnts)
##        if red_area > area_thresh:
##            raw_color = 1
##        # optional debug:
##        if np.count_nonzero(red_mask) > 0:
##            mean_r = cv.mean(hsv_right, mask=red_mask)
####            print("    Red mean HSV:", tuple(map(int, mean_r[:3])))
##
##    # Detect green on the left
##    if hsv_left is not None:
##        gmask = cv.inRange(hsv_left, np.array(green_range[0]), np.array(green_range[1]))
##        green_cnts, _ = cv.findContours(gmask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
##        green_area = sum(cv.contourArea(c) for c in green_cnts)
##        if green_area > area_thresh:
##            raw_color = 2
##        # optional debug:
##        if np.count_nonzero(gmask) > 0:
##            mean_g = cv.mean(hsv_left, mask=gmask)
####            print("    Green mean HSV:", tuple(map(int, mean_g[:3])))
##
##    # Update lock only on a real arrow detection
##    if raw_color in (1, 2):
##        get_color_code.locked_color = raw_color
##    print(get_color_code.locked_color)
##    return get_color_code.locked_color

# Run until the user types 'q' to quit
while True:
    
    # Capture the frame
    ret, frame = cap.read()
    if not ret:
        print("Camera error")
        continue
    
    # Undistort frame
    frame = cv.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
    hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
##    cv.imshow("HSV Frame", hsvFrame)
##    cv.imshow("Frame", frame)
    # Convert to grayscale (This will used to find the marker)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Detect Aruco markers
    corners, ids, rejected = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, dist)

    if ids is not None: # We see markers
        
        markerFound = 1

        # Find distance and angle to marker location
        currDistance = get_distance(rvecs, tvecs) 
        currAngle = get_angle(rvecs, tvecs)
        if currDistance >= MARKER_THRESHOLD:
            continue
        
        # Check the distance, start detecting angle within COLOR_DETECTION_THRESHOLD
        if currDistance <= COLOR_DETECTION_THRESHOLD:
            # Set the turn to the detected arrow
            arrow = get_color(corners,frame,ids)
##            currDistance = 1 # tell the robot to not move
        else:
            arrow = 0 # set the arrow to 0 if there is no marker

    else: # No markers are found
        markerFound = 0 
        arrow = 0
        # Do we need these if there are no markers found?
##        currAngle = 99.9
##        currDistance = 99.9

    
##    cv.imshow("Aruco Detection", frame)
    
    try: # try to send to ard
##        print(message[0])
##        print(message[1])
        if arrow < 0 or arrow > 2:
            continue
        print(arrow)
        data = struct.pack('bbff', markerFound, arrow, currDistance, currAngle)
##        print(len(data))
##        print(list(data))
##        print(list(data)) 
##        data = [struct.pack('<B', markerFound), struct.pack('<B', arrow)] + list(struct.pack('<f', currDistance)) + list(struct.pack('<f', currAngle))
        i2cARD.write_i2c_block_data(ARD_ADDR, 0,list(data))
        
        niceData = [markerFound, arrow, currDistance, currAngle]
        # Only send a valid 10 byte vector
        if len(data) != 10:
##            print("Skip")
            continue

##        print(list(data))
##        i2cARD.write_i2c_block_data(ARD_ADDR, 0, data)
##        i2cARD.write_i2c_block_data(ARD_ADDR, 0,data)
##        print("send")
    except: # continue if we get an i2c error
        continue
    
    
    # Quit when the user hits 'q'
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release all resources (camera, LCD, windows)
cap.release()
cv.destroyAllWindows()

print("Done")
