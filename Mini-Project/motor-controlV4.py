# EENG 350 - Mini Project Pi Code.
# Group 7 - Pyrite
# Pi code created Bruce Bearden and Drew Barner
# Arduino code created by Parker Anderson and Caleb Brown

'''
This code detects the region where an aruco marker is detected, and
sends a command to the arduino to where the wheels should turn. Based on
which quadrant of the camera the marker is detected on the wheel will turn as
follows:

    Qudrant  -   Wheel Location [left wheel, right wheel]
    ----------------------------------------------------
      NE     -   [0,0]
      NW     -   [0,1]
      SE     -   [1,0]
      SW     -   [1,1]

To increase/decrease the lag adjustment of the frame rate is possible
The program runs optimally displaying every thrid frame, since the processing
power of the pi is not anything special.


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

# library imports
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import smbus2
import threading
import queue
import numpy as np
import cv2
from cv2 import aruco

# define the I2C Address of Arduino
ARD_ADDR = 8  

# Initialise I2C communications for both the LCD screen and the Arduino
i2cLCD = board.I2C()  # uses board.SCL and board.SDA
i2cARD = smbus2.SMBus(1)  # Use I2C bus 1 for communication with Arduino

# Initialise LCD rows and cols
LCD_COLUMS = 16
LCD_ROWS = 2

# Initialise the queue where the updated marker quadrants are stored
lcdQueue = queue.Queue()

# Initialize wheel location as none, since we will update the LCD when we find a marker
wheelLocation = None

def updateLCD():
    print("Started LCD Thread")

    #*********
    # LCD Init
    #*********
    
    # Initialise the LCD class
    lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, LCD_COLUMS, LCD_ROWS)
    lcd.clear()
    lcd.color = [0, 0, 100]  # Set initial color (Blue)
    print("LCD init done")

    #***********************************************
    # Update the LCD and send data only if necessary
    #***********************************************
    while True:
        if not lcdQueue.empty():
            quad = lcdQueue.get()
            lcd.clear()
            
            if wheelLocation is not None:
                location = wheelLocation # temp assignment in case of quick change in marker position
                lcd.message = str(quad)

                # Send the wheelLocation data to the Arduino using smbus2
                i2cARD.write_byte_data(ARD_ADDR, 0x00, location)

# Start the LCD update thread
myThread = threading.Thread(target=updateLCD, daemon=True)
myThread.start()

# Initialise the camera and the video
cam = cv2.VideoCapture(0)  # init the camera
cv2.namedWindow("Real-Time")  # shows real-time video
ret, frame = cam.read()
if not ret:
    print("Failed to grab frame.")
    exit()

# The default window size is x = 640 and y = 480
height, width, _ = frame.shape
vertThresh = width / 2
horiThresh = height / 2

quadrants = ["0 1", "0 0", "1 1", "1 0", "None"] # these will print to the lcd

wheelLocations = [2,1,3,4] # these will be sent to the arduino
wheelLocation = None # stores where the wheels should be
lastQuadrant = None # stores the last detected quadrant to detect change in marker location
quadrant = None # stores the current quadrant to detect change in marker location
frameCount = 0 # stores frame count to slow down camera for less lag
frames = 3 # set the fps of the program.

# ArUco marker setup (tells the program the aruco markers we can expect)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

while True:

    # Capture the frame, exit if camera capture doesn't work
    ret, frame = cam.read()
    if not ret:
        print("Failed to grab frame.")
        break
    # reduce lag by adjusting the frames
    if frameCount % frames == 0:
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert the frame to gray scale
        corners, ids, rejected = aruco.detectMarkers(grey, aruco_dict)  # Find the markers and associated corners

        # Draw border lines and detected markers
        overlay = aruco.drawDetectedMarkers(grey, corners, borderColor=4)

        cv2.line(overlay, (width // 2, 0), (width // 2, height), (0, 255, 0), 2)  # Horizontal line
        cv2.line(overlay, (0, height // 2), (width, height // 2), (0, 255, 0), 2)  # Vertical line

        if ids is not None:  # if any markers are detected
            ids = ids.flatten()
            for (outline, id) in zip(corners, ids):
                markerCorners = outline.reshape((4, 2))

                xCords = markerCorners[:, 0]
                yCords = markerCorners[:, 1]

                # Determine horizontal and vertical boundaries for the quadrants
                isLeft = np.all(xCords < vertThresh)  # All corners on the left of the vertical threshold
                isRight = np.all(xCords > vertThresh)  # All corners on the right of the vertical threshold
                isTop = np.all(yCords < horiThresh)  # All corners on the top of the horizontal threshold
                isBottom = np.all(yCords > horiThresh)  # All corners on the bottom of the horizontal threshold

                # Determine quadrant based on conditions
                if isLeft and isTop:
                    quadrant = quadrants[0]  # NW quadrant
                    wheelLocation = wheelLocations[0]
                elif isRight and isTop:
                    quadrant = quadrants[1]  # NE quadrant
                    wheelLocation = wheelLocations[1]
                elif isLeft and isBottom:
                    quadrant = quadrants[2]  # SW quadrant
                    wheelLocation = wheelLocations[2]
                elif isRight and isBottom:
                    quadrant = quadrants[3]  # SE quadrant
                    wheelLocation = wheelLocations[3]
                else:
                    quadrant = quadrants[4]  # None, if marker is somewhere in between
                    wheelLocation = None

        else: # No marker detected
            quadrant = quadrants[4]  
            wheelLocation = None

        # Update the LCD if the quadrant changes
        if quadrant != lastQuadrant:
            lcdQueue.put(quadrant)  # Add the new quadrant to the queue
            lastQuadrant = quadrant # update the last detected quadrant

        #Display the real-time feed with marker information
        cv2.imshow("Real-Time", overlay)

    frameCount += 1

    # Check for 'ESC' to exit
    k = cv2.waitKey(1)
    if k % 256 == 27:  # ESC pressed
        print("Escape hit, closing...")
        break

# Release the camera, close windows, and reset LCD
cam.release()
cv2.destroyAllWindows()
lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, LCD_COLUMS, LCD_ROWS)
lcd.clear()
lcd.color = [0, 0, 0] 
