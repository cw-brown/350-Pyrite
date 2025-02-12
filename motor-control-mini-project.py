#Bruce Bearden
# EENG 350 Motor Control Project

'''
This code uses the rasberry pi interface to determine which quadrant the marker
is on the screen. Based on that the pi will update the LCD screen with the goal
position and send a command to the arduino to move the wheels to a specified
position

'''


# library imports
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
import threading
import queue

##markerQuadrant = 1
##def updateLCD(markerQuadrant):
##    # LCD Init
##    # Initialise lcd size
##    lcd_columns = 16
##    lcd_rows = 2
##    # Initialise I2C bus.
##    i2c = board.I2C()  # uses board.SCL and board.SDA
##    # Initialise the LCD class
##    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
##    # Turn the screen red, and write no markers found since the program just started
##    lcd.clear()
##    lcd.color = [100,0,0]
##    lcd.message = "No Markers Found"
##    while True:
##        if not q.empty():
##            gotSomething = q.get()
##            print(f"Marker found in quadrant {markerQuadrant}")
##            #*******************************
##            # Write new data to the LCD here
            #*******************************
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # add markers to the dictionary
##myThread = threading.Thread(target=updateLCD, args=())
##myThread.start()
# LCD Init

# Initialise lcd size
lcd_columns = 16
lcd_rows = 2
# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# Turn the screen red, and write no markers found since the program just started
lcd.clear()
lcd.color = [100,0,0]
lcd.message = "No Markers Found"

# Camera and window Init
cam = cv2.VideoCapture(0) # init the camera
cv2.namedWindow("Real-Time") # shows real time video
##cv2.namedWindow("Snapshot") # displays a snapshot every second REMOVE THIS FUNCTIONALITY


# The default window size is x = 640 and y = 480
# region definitions for a 640 x 480 window.
ne = [[320,0],[640,0],[640,240],[320,240]]
se = [[320,240],[640,240],[640,480],[320,480]]
sw = [[240,0],[240,300],[480,300],[480,0]]
nw = [[0,0],[320,0],[320,240],[0,240]]
quadrants = ["NW", "NE", "SW", "SE"]




# Track the last capture time for snapshot
lastCapTime = time.time()

# Want to minimally update the LCD to minimize delay
updateLCD = False # tell us if we need to update the LCD, we do not need to at the start because if any markers are detected then the loop will immediately update
idsPast = np.array([]) # intialises a numpy array so we can check if the markers detected changes
while True: # repeat until the camera fails to capture or if the user presses the escape key
    
    # Capture the frame, exit if camera capture  doesn't work
    ret, frame = cam.read()
    if not ret:
        print("Failed to grab frame.")
        break
    height, width, _ = frame.shape
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # convert the frame to grey scale
    

    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict) # find the ids and their corners
    # corner location detection
    overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2RGB) # Convert back to RGB for imshow, as well as for the next step
    overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
    # draw the border lines
    cv2.line(overlay, (width // 2,0),(width // 2,height),(0,255,0),2) # hori line
    cv2.line(overlay, (0,height//2),(width, height//2),(0,255,0),2) # vert line
    if not ids is None: # markers found    
        ids = ids.flatten()
        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2))
            overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2) # draw the ids on the corner
##        print(markerCorners[2][0])
        vertThresh = width / 2
        horiThresh = height / 2
        if (markerCorners[0][0] > vertThresh and markerCorners[1][0] > vertThresh and markerCorners[2][0] > vertThresh and markerCorners[3][0] > vertThresh) and (markerCorners[0][1] < horiThresh and markerCorners[1][1] < horiThresh and markerCorners[2][1] < horiThresh and markerCorners[3][1] < horiThresh):
            print("NE")                                                                                                               
        elif (markerCorners[0][0] > vertThresh and markerCorners[1][0] > vertThresh and markerCorners[2][0] > vertThresh and markerCorners[3][0] > vertThresh) and (markerCorners[0][1] > horiThresh and markerCorners[1][1] > horiThresh and markerCorners[2][1] > horiThresh and markerCorners[3][1] > horiThresh):
            print("SE")                                                                                                                  
        elif (markerCorners[0][0] < vertThresh and markerCorners[1][0] < vertThresh and markerCorners[2][0] < vertThresh and markerCorners[3][0] < vertThresh) and (markerCorners[0][1] > horiThresh and markerCorners[1][1] > horiThresh and markerCorners[2][1] > horiThresh and markerCorners[3][1] > horiThresh):
            print("SW")                                                                                                                  
        elif (markerCorners[0][0] < vertThresh and markerCorners[1][0] < vertThresh and markerCorners[2][0] < vertThresh and markerCorners[3][0] < vertThresh) and (markerCorners[0][1] < horiThresh and markerCorners[1][1] < horiThresh and markerCorners[2][1] < horiThresh and markerCorners[3][1] < horiThresh):
            print("NW")
        else:
            print("no region")

    cv2.imshow("Real-Time", overlay) # display the real time feed



    #*************************************
    # TODO: Update the LCD using threading
    #*************************************
    
    if time.time() - lastCapTime >= 1: # only update the LCD every second, to minimize LCD writting delay
        lastCapTime = time.time()  # Update the last capture time
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict) # find the ids and their corners
        overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2RGB) # Convert back to RGB for imshow, as well as for the next step
        overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4) 

        if not ids is None: # markers found
            ids = ids.flatten()
            for (outline, id) in zip(corners, ids):
                markerCorners = outline.reshape((4,2))
                overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2) # draw the ids on the corner
        else:
            ids = np.array([])
   
        if not np.array_equal(ids,idsPast):
            updateLCD = True # need to update the LCD since the ids detected have changed
            idsPast = ids # reassign the past ids.
        else:
            updateLCD = False
            idsPast = ids
            
        if updateLCD: # only update the screen if necessary (number of markers has changed)
            updateLCD = False # reset the flag
            if not ids is None and ids.size >= 1: # write to the LCD that we found markers
                # Turn the screen green, and write how many markers were found and their IDs
                lcd.clear()
                lcd.color = [0,100,0]
                lcd.message = "Found " + str(ids.size) + " Aruco(s)\nWith IDs: " + str(ids[0:])
            else: # write to the screen that we found no markers
                # Turn the screen red, and write no markers found
                lcd.clear()
                lcd.color = [100,0,0]
                lcd.message = "No Markers Found"

##        cv2.imshow("Snapshot", overlay) # remove the snap shot every second

    # Check for 'ESC' to exit
    k = cv2.waitKey(1)
    if k % 256 == 27:  # ESC pressed
        print("Escape hit, closing...")
        break

# Release the camera, windows and lcd
lcd.clear()
lcd.color = [0,0,0]
cam.release()
cv2.destroyAllWindows()
