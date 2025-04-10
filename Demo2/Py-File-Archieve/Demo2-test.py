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

# define the I2C Address of Arduino
ARD_ADDR = 8  

# Initialise I2C communications for both the LCD screen and the Arduino
i2cLCD = board.I2C()  # uses board.SCL and board.SDA
i2cARD = smbus2.SMBus(1)  # Use I2C bus 1 for communication with Arduino

# Initialise the queue where the updated marker quadrants are stored
lcdQueue = queue.Queue()

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
currentAngle = None
lastAngle = None
currentDistance = None
lastDistance= None
currentColor = None
lastColor= None
lcdPrompt = []

last_update_time = time()


def updateLCD():
    print("Started LCD Thread")

    #*********
    # LCD Init
    #*********
    
    # Initialise the LCD 
    lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, LCD_COLUMS, LCD_ROWS)
    lcd.clear()
    lcd.color = [10, 0, 0]  # Set initial color (red)
    print("LCD init done")

    #***********************************************
    # Update the LCD and send data only if necessary
    #***********************************************
    while True:
        if not lcdQueue.empty():
            message = lcdQueue.get()
        else:
            message = [9.9,99.9]
    ##            i2cARD.write_i2c_block_data(ARD_ADDR, 0x00, message)
        message = str(message)
        lcd.clear() 
    ##            lcd.message = str(message)
        lcd.message = message
        # Send the wheelLocation data to the Arduino using smbus2
        command = [ord(character) for character in message]
        i2cARD.write_i2c_block_data(ARD_ADDR, 0x00, command[1:-1])

        
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

def get_distance(cnrs):
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(cnrs, 0.05, cameraMatrix, dist)
    for i in range(len(ids)):
        rvec = rvecs[i][0]
        tvec = tvecs[i][0]
        distance = float(round(tvec[2]*3.28,1))  # Convert meters to feet
    return distance

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
            color = "Red"
        elif green_mask_left is not None and np.count_nonzero(green_mask_left) > 0:
            color = "Green"
    
    return color

# Start the LCD update thread
myThread = threading.Thread(target=updateLCD, daemon=True)
myThread.start()

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
        currAngle = get_angle(corners) # find the angle
        currDistance = get_distance(corners) # find the distance

        # Check the distance ( > 1 or <= 1)
        if currDistance <= 1:  # If the distance is <= 1 foot (3.28 meters)
            currColor = get_color(corners,frame,ids)
            currDistance = 0.0
            if currColor == "Green":
                currAngle = 90.0
            elif currColor == "Red":
                currAngle = -90.0
        else:
            currColor = None
            

        # Update LCD only when necessary (based on the angle, distance, or color change)
        if time() - last_update_time >= 0.5 and (currAngle != lastAngle or currDistance != lastDistance or currColor != lastColor):
            if currAngle != lastAngle:
                lastAngle = currAngle # update the angle
            if currDistance != lastDistance:
                lastDistance = currDistance # update the distance
            if currColor != lastColor:
                lastColor = currColor # update the color
            lcdPrompt = [currDistance, currAngle]
##            chunk_size = 32
##            for i in range(0,len(byte_data),chunk_size):
##                chunk = lcdPrompt[i:i+chunk_size]
##                i2cARD.write_i2c_block_data(ARD_ADDR, 0x00, list(chunk))
##                time.sleep(0.1)
##            i2cARD.write_i2c_block_data(ARD_ADDR, 0x00, lcdPrompt) # send the data to ard as an array of floats
            lcdQueue.put(lcdPrompt)
            last_update_time = time()    

    else:
        currAngle = None
        currDistance = None
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
