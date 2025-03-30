import numpy as np
import cv2 as cv
from time import sleep, time
import pickle
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
import board


''' 
'''

# Load camera calibration information
with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist = pickle.load(f)

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
last_update_time = time()


def updateLCD():
    print("Started LCD Thread")

    #*********
    # LCD Init
    #*********
    
    # Initialise the LCD 
    lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, LCD_COLUMS, LCD_ROWS)
    lcd.clear()
    lcd.color = [100, 0, 0]  # Set initial color (red)
    print("LCD init done")

    #***********************************************
    # Update the LCD and send data only if necessary
    #***********************************************
    while True:
        if not lcdQueue.empty():
            angle = lcdQueue.get()
            lcd.clear() 
##            print(f"Angle = {angle} degrees")
            lcd.message = str(angle) # COMMENT IN FOR LCD

    
def get_angle(cnrs):
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(cnrs, 0.05, cameraMatrix, dist)
        
    for i in range(len(ids)):
        rvec = rvecs[i][0]
        tvec = tvecs[i][0]
        
        # Compute rotation matrix
        R, _ = cv.Rodrigues(rvec)
        
        # Angle between camera axis and marker
        angle = -1*np.degrees(np.arctan2(tvec[0], tvec[2]))
        angle = round(angle, 2) # round the angle to 2 decimal points. 
    return angle

def get_distance(cnrs):
    distance = 0
    return distance

def get_color():
    color = 0
    return color

# Start the LCD update thread
myThread = threading.Thread(target=updateLCD, daemon=True)
myThread.start()

# repeat until user hits 'q'
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
        currAngle = get_angle(corners)
##            # Draw marker and axis
####            cv.aruco.drawDetectedMarkers(gray, corners) # NOT SURE IF WE NEED THIS
####            cv.drawFrameAxes(gray, cameraMatrix, dist, rvec, tvec, 0.03)
##            
##            # Add the angle to the queue every .1 sec
        if currAngle != lastAngle:
            if time() - last_update_time >= 0.5:
                lcdQueue.put(currAngle)  # Add the new angle to the queue
                lastAngle = currAngle # update the angle
                last_update_time = time()
        else:
            currAngle = None

    # Show the frame
    cv.imshow("Aruco Detection", gray)
    
    # Exit on key press
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
lcd.clear()
lcd.color = [0,0,0]

print("Done")
