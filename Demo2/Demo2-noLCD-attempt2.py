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

# Angle, arrow and distance place holders
currAngle = None
lastAngle = None
currDistance = None
lastDistance= None
Arrow = None

# Marker detection flag
markerFound = 0 

# Implement time based sending
last_update_time = time()

# find the angle off the center axis of rotation. return an angle to 1 decimal point of accuracy      
def get_angle(rvecs, tvecs):        
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
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(cnrs, 0.05, cameraMatrix, dist)
    for i in range(len(ids)):
        rvec = rvecs[i][0]
        tvec = tvecs[i][0]
        distance = float(round(tvec[2]*3.28,1))  # Convert meters to feet
    return distance

# find the color on the marker
def get_color(cnrs, frame, ids):
    for i in range(len(ids)):
        color = None
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
            color = 0
##    cropped_frame = frame[roi_y_min:roi_y_max, roi_x_min_left:roi_x_max_right]
##    cv.imshow("Aruco Detection", cropped_frame)
##    print(color)

    return color

# Run until the user types 'q' to quit
while True:
    
    # Capture the frame
    ret, frame = cap.read()
    if not ret:
        print("Shit")
        continue
    
    # Undistort frame
    frame = cv.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
    
    # Convert to grayscale (This will used to find the marker)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Detect Aruco markers
    corners, ids, rejected = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, dist)

    # Initialize the currDistance and currAngle as 9.9 and 99.9
    currDistance = 9.9
    currAngle = 99.9
    
    if ids is not None: # We see markers
        
        markerFound = 1

        # Find distance to marker location
        currDistance = get_distance(rvecs, tvecs) 

        # Check the distance ( > 1 or <= 1), only detect color within 1 foot, and detect angle outside 1 foot
        if currDistance <= 2:
            # Set the turn to the detected arrow
            arrow = get_color(corners,frame,ids) 
##            currDistance = 1 # tell the robot to not move
        else:
            currAngle = get_angle(rvecs, tvecs) # find the angle
    
        messageToArd = [markerFound, arrorw, currDistance, currAngle] # compile the current Distance and angle into a message to send to the arduino
         # Convert the floats into a byte array (4 bytes each)
##        data = struct.pack('bbff', message[0], message[1])
##        
##        # Send the packed float data (8 bytes total)
##        i2cARD.write_i2c_block_data(ARD_ADDR, 0, list(data))
##        message = str(message) # turn it into a string
##        command = [ord(character) for character in message]
##        i2cARD.write_i2c_block_data(ARD_ADDR, 0x00, command[1:-1])
   

        # Update LCD only when necessary (based on the angle, distance, or color change)
        if time() - last_update_time >= 0.5 and (currAngle != lastAngle or currDistance != lastDistance):
##        if time() - last_update_time >= 1:
            if currAngle != lastAngle:
                lastAngle = currAngle # update the angle
            if currDistance != lastDistance:
                lastDistance = currDistance # update the distance

    else: # No markers are found
        markerFound = 0 

        # Do we need these if there are no markers found?
        currAngle = 99.9
        currDistance = 99.9

        # Update the distance and angle if they change
        if currDistance != lastDistance or currAngle != lastAngle:
            lastAngle = currAngle
            lastDistance = currDistance
            
        messageToArd = [markerFound, arrow, currDistance, currAngle] # compile the current Distance and angle into a message to send to the arduino
        
         # Convert the floats into a byte array (4 bytes each)
##    data = struct.pack('bbff', message[0], message[1])
        
        # Send the packed float data (8 bytes total)
    try: # try to send to ard
##        print(message[0])
##        print(message[1])
        data = struct.pack('bbff', message[0] - 1, message[1])
        i2cARD.write_i2c_block_data(ARD_ADDR, 0, list(data))
        # Show the (gray) frame
        
    except: # continue if we get an i2c error
        continue
    
##    cv.imshow("Aruco Detection", frame)
    # Quit when the user hits 'q'
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release all resources (camera, LCD, windows)
cap.release()
cv.destroyAllWindows()

print("Done")
