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
This Python code enables a robot to autonomously detect a visual beacon using an Aruco marker 
ID, navigate to within 1.5 feet of it, and perform a directional turn based on the color of the
arrow indicator. 

The code supports two types of trials but edits to the DO_TURN variable in the Arduino Code are necessary: 
(1) approach and stop within 1.5 feet of the marker, 
and (2) approach and turn 90° in the correct direction. 

Performance is evaluated based on accuracy, timing, and reliability across runs.

The I2C communication protocol is used to send a string to the arduino to display the
desired location of the left wheel and right wheel, and to send a byte to the
arduino to commincate the desired location.

To run this code, ensure that the aruco marker used for testing is defined in the
imported aruco dictionary, and verify the following connections from the lcd pi shield
and the arduino:

    *NOTE: The LCD pi shield needs to properly attached where the left side of the shield
           is lined up with the SD card side of the PI. View the EENG 350 Assingment 1
           computer vision and communication tutorial for more details.

    Connections:
    pi        -        arduino
    --------------------------
    GND       -        GND
    SCL       -        A5
    SDA       -        A4

'''

COLOR_THRESH = 1
DETECTION_THRESH = 5
# define the I2C Address of Arduino
ARD_ADDR = 8  

# I2C declarations for ard and lcd
i2cLCD = board.I2C()  # uses board.SCL and board.SDA
i2cARD = smbus2.SMBus(1)  # Use I2C bus 1 for communication with Arduino
# init the camera
cap = cv.VideoCapture(0)

# Camera Calibration and Distortion Matricies
with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist = pickle.load(f)

# Aruco marker dictionary and detector parameters
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50) # using a 6x6 marker
parameters = cv.aruco.DetectorParameters()

# Angle, distance and color place holders
currAngle = None
lastAngle = None
currDistance = None
lastDistance= None
currColor = None
lastColor= None
lcdPrompt = []

# Implement time based sending to minimize any lag
last_update_time = time()

# find the angle off the center axis of rotation. return an angle to 1 decimal point of accuracy      
def get_angle(cnrs):
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(cnrs, 0.05, cameraMatrix, dist)
        
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
def get_distance(cnrs):
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(cnrs, 0.05, cameraMatrix, dist)
    for i in range(len(ids)):
        rvec = rvecs[i][0]
        tvec = tvecs[i][0]
        distance = float(round(tvec[2]*3.28,1))  # Convert meters to feet
    return distance

def get_color(cnrs, frame, ids):
##    print("color")
    for i in range(len(ids)):
        color = None
        # Get the coordinates of the marker corners
        marker_corners = cnrs[i][0]
        x_min, y_min = np.min(marker_corners, axis=0).astype(int)
        x_max, y_max = np.max(marker_corners, axis=0).astype(int)

        # Crop the image
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

        redLower = np.array([0, 150, 70], dtype=np.uint8)
        redUpper = np.array([10, 255, 255], dtype=np.uint8)
        redLower2 = np.array([170, 180, 50], dtype=np.uint8)
        redUpper2 = np.array([180, 255, 255], dtype=np.uint8)
        greenLower = np.array([70, 80, 40], dtype=np.uint8)
        greenUpper = np.array([90, 255, 255], dtype=np.uint8)

        
        red_range_1 = [redLower, redUpper]
        red_range_2 = [redLower2, redUpper2]
        green_range = [greenLower, greenUpper]

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
            color = -90 #Red
##            print("-90.0 / Red")
        elif green_mask_left is not None and np.count_nonzero(green_mask_left) > 0:
            color = 90 #Green
##            print("90.0 / Green")
        else:
            color = -50 # Whtie
##    cropped_frame = frame[roi_y_min:roi_y_max, roi_x_min_left:roi_x_max_right]
##    cv.imshow("Aruco Detection", cropped_frame)
##    print(color)
    return color

# Run until the user types 'q' to quit
while True:
    
    # Capture the frame
    ret, frame = cap.read()
    if not ret:
        print("Camera Error")
        continue
    
    # Undistort frame
    frame = cv.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
    
    # Convert to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
##    cv.imshow("Image", frame)
    # Detect Aruco markers
    corners, ids, rejected = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    currDistance = 99.9
    currAngle = 99.9
    if ids is not None: # if we find markers

        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, dist)
        
        # Calculate the distance for each marker (in feet)
        distances = [float(round(tvec[0][2] * 3.28084, 1)) for tvec in tvecs]
        
        # Find the index of the closest marker
        closest_idx = np.argmin(distances)
        
        # Access the closest marker directly
        closest_corner = corners[closest_idx]
        # closest_id = ids[closest_idx][0]
        # closest_distance = distances[closest_idx]
        currDistance = get_distance(closest_corner) # find the distance, use to determine if we need to send an angle
        if currDistance > DETECTION_THRESH:
            currDistance = 99.9
            try:
##                print([currDistance, currAngle])
                i2cARD.write_i2c_block_data(ARD_ADDR, 0, [currDistance, currAngle])  
            except:
                continue
        # Check the distance only detect color within Color thresh foot, and detect angle outside 1 foot
        if currDistance <= COLOR_THRESH:  
            currAngle = get_color(closest_corner,frame,ids) # Set the angle to the detected color
        else:
            currAngle = get_angle(closest_corner) # find the angle
##            currColor = 0 # unused input, set it to something so no errors
    
        message = [currDistance, currAngle] # compile the current Distance and angle into a message to send to the arduino
   
##        if time() - last_update_time >= 0.5 and (currAngle != lastAngle or currDistance != lastDistance):
##            if currAngle != lastAngle:
##                lastAngle = currAngle # update the angle
##            if currDistance != lastDistance:
##                lastDistance = currDistance # update the distance
##            if currColor != lastColor:
##                lastColor = currColor # update the color

    else:
        currAngle = 99.9
        currDistance = 99.9
##        if currDistance != lastDistance or currAngle != lastAngle:
##            lastAngle = currAngle
##            lastDistance = currDistance  
            
##        currColor = 1 # place holder value
        message = [currDistance, currAngle] # compile the current Distance and angle into a message to send to the arduino

    # Try to send data, if I2C error, continue to next iteration
    data = struct.pack('ff', message[0], message[1])
    ## print(message)
    try:
        i2cARD.write_i2c_block_data(ARD_ADDR, 1, list(data))   
    except:
        continue

    # Quit if user hits q
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release all resources (camera, LCD, windows)
cap.release()
cv.destroyAllWindows()

print("Exit")
