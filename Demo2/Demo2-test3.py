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
# define the I2C Address of Arduino
ARD_ADDR = 8  

# I2C declarations for ard and lcd
i2cLCD = board.I2C()  # uses board.SCL and board.SDA
i2cARD = smbus2.SMBus(1)  # Use I2C bus 1 for communication with Arduino

# Initialise the queue where a distance and vector is stored
lcdQueue = queue.Queue()

# Camera Calibration and Distortion Matricies
with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist = pickle.load(f)



# Aruco marker dictionary and detector parameters
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50) # using a 6x6 marker
parameters = cv.aruco.DetectorParameters()


# Initialise LCD rows and cols COMMENT IN FOR LCD
i2cLCD = board.I2C()  # uses board.SCL and board.SDA

LCD_COLUMS = 16
LCD_ROWS = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, LCD_COLUMS, LCD_ROWS)

# init the camera
cap = cv.VideoCapture(0)

# Angle, distance and color place holders
currentAngle = None
lastAngle = None
currentDistance = None
lastDistance= None
currentColor = None
lastColor= None
lcdPrompt = []

# Implement time based sending to minimize any lag
last_update_time = time()

# LCD thread to minimize delay in the system
def updateLCD():
##    print("Started LCD Thread")
    # LCD Init
    lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, LCD_COLUMS, LCD_ROWS)
    lcd.clear()
    lcd.color = [10, 0, 0]  # Set initial color (red)
##    print("LCD init done")

    # Update the LCD only when necessary
    while True:
        if not lcdQueue.empty():
            message = lcdQueue.get()
##        else:
##            message = [9.9,99.9]
##        i2cARD.write_i2c_block_data(ARD_ADDR, 0x00, message)

        #*************************************************
        # To send a string
            message = str(message)
            lcd.clear() 
            lcd.message = message
        # Send the wheelLocation data to the Arduino using smbus2


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

        # Define color ranges for red and green in HSV
        red_range_1 = [(0, 120, 70), (10, 255, 255)]  # Lower red range
        red_range_2 = [(170, 120, 70), (180, 255, 255)]  # Upper red range
        green_range = [(40, 40, 40), (80, 255, 255)]  # Green range

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
            color = -90.0 #Red     
        elif green_mask_left is not None and np.count_nonzero(green_mask_left) > 0:
            color = 90.0 # Green
    return color

# Start the LCD update thread
myThread = threading.Thread(target=updateLCD, daemon=True)
myThread.start()

# Run until the user types 'q' to quit
while True:
    
    # Capture the frame
    ret, frame = cap.read()
    if not ret:
        continue
    
    # Undistort frame
    frame = cv.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
    
    # Convert to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Detect Aruco markers
    corners, ids, rejected = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    currDistance = 9.9
    currAngle = 99.9
    if ids is not None: # if we find markers
        
        currDistance = get_distance(corners) # find the distance, use to determine if we need to send an angle

        # Check the distance ( > 1 or <= 1), only detect color within 1 foot, and detect angle outside 1 foot
        if currDistance <= 1:  
            currAngle = get_color(corners,frame,ids) # Set the angle to the detected color
            currDistance = 0.0 # tell the robot to not move
        else:
            currAngle = get_angle(corners) # find the angle
            currColor = 1 # unused input, set it to something so no errors
    
        message = [currDistance, currAngle] # compile the current Distance and angle into a message to send to the arduino
        message = str(message) # turn it into a string
        command = [ord(character) for character in message]
        i2cARD.write_i2c_block_data(ARD_ADDR, 0x00, command[1:-1])
   

        # Update LCD only when necessary (based on the angle, distance, or color change)
        if time() - last_update_time >= 0.5 and (currAngle != lastAngle or currDistance != lastDistance):
##        if time() - last_update_time >= 1:
            if currAngle != lastAngle:
                lastAngle = currAngle # update the angle
            if currDistance != lastDistance:
                lastDistance = currDistance # update the distance
            if currColor != lastColor:
                lastColor = currColor # update the color
            lcdPrompt = [currDistance, currAngle]
            lcdQueue.put(lcdPrompt)
            last_update_time = time()    

    else:
        currAngle = 9.9
        currDistance = 99.9
        if currDistance != lastDistance or currAngle != lastAngle:
            lastAngle = currAngle
            lastDistance = currDistance
            lcdPrompt = [currDistance, currAngle]
            lcdQueue.put(lcdPrompt)
            last_update_time = time()  
            
        currColor = 1 # place holder value
        message = [currDistance, currAngle] # compile the current Distance and angle into a message to send to the arduino
        message = str(message) # turn it into a string
        command = [ord(character) for character in message]
        i2cARD.write_i2c_block_data(ARD_ADDR, 0x00, command[1:-1])
    # Show the (gray) frame
    cv.imshow("Aruco Detection", gray)
    
    # Quit when the user hits 'q'
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release all resources (camera, LCD, windows)
cap.release()
cv.destroyAllWindows()
lcd.clear()
lcd.color = [0,0,0]

print("Done")
