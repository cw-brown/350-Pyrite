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

COLOR_THRESH = 1
DETECTION_THRESH = 5
ARD_ADDR = 8  

i2cLCD = board.I2C()
i2cARD = smbus2.SMBus(1)
cap = cv.VideoCapture(0)

with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist = pickle.load(f)

dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
parameters = cv.aruco.DetectorParameters()

def get_color(cnrs, frame):
    color = None
    marker_corners = cnrs[0]
    x_min, y_min = np.min(marker_corners, axis=0).astype(int)
    x_max, y_max = np.max(marker_corners, axis=0).astype(int)
    offset = 70
    roi_x_min_left = max(0, x_min - offset)
    roi_x_max_left = x_min
    roi_x_min_right = x_max
    roi_x_max_right = min(frame.shape[1], x_max + offset)
    roi_y_min = y_min
    roi_y_max = y_max
    roi_right = frame[roi_y_min:roi_y_max, roi_x_min_right:roi_x_max_right]
    roi_left = frame[roi_y_min:roi_y_max, roi_x_min_left:roi_x_max_left]
    hsv_right = cv.cvtColor(roi_right, cv.COLOR_BGR2HSV) if roi_right.size > 0 else None
    hsv_left = cv.cvtColor(roi_left, cv.COLOR_BGR2HSV) if roi_left.size > 0 else None
    redLower = np.array([0, 150, 70], dtype=np.uint8)
    redUpper = np.array([10, 255, 255], dtype=np.uint8)
    redLower2 = np.array([170, 180, 50], dtype=np.uint8)
    redUpper2 = np.array([180, 255, 255], dtype=np.uint8)
    greenLower = np.array([70, 80, 40], dtype=np.uint8)
    greenUpper = np.array([90, 255, 255], dtype=np.uint8)
    red_mask_right = None
    green_mask_left = None
    if hsv_right is not None:
        red_mask_right_1 = cv.inRange(hsv_right, redLower, redUpper)
        red_mask_right_2 = cv.inRange(hsv_right, redLower2, redUpper2)
        red_mask_right = cv.bitwise_or(red_mask_right_1, red_mask_right_2)
    if hsv_left is not None:
        green_mask_left = cv.inRange(hsv_left, greenLower, greenUpper)
    if red_mask_right is not None and np.count_nonzero(red_mask_right) > 0:
        color = -90  # Red
    elif green_mask_left is not None and np.count_nonzero(green_mask_left) > 0:
        color = 90  # Green
    else:
        color = -50  # White
    return color

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera Error")
        continue
    
    frame = cv.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    corners, ids, rejected = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is not None and len(ids) > 0:
        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, dist)
        distances = [float(round(tvec[0][2] * 3.28, 1)) for tvec in tvecs]
        min_distance_index = np.argmin(distances)
        currDistance = distances[min_distance_index]
        
        if currDistance > DETECTION_THRESH:
            currDistance = 99.9
            currAngle = 99.9
            try:
                i2cARD.write_i2c_block_data(ARD_ADDR, 0, [currDistance, currAngle])
            except:
                continue
        else:
            closest_corners = [corners[min_distance_index]]
            if currDistance <= COLOR_THRESH:
                currAngle = get_color(closest_corners, frame)
            else:
                rvec = rvecs[min_distance_index][0]
                tvec = tvecs[min_distance_index][0]
                R, _ = cv.Rodrigues(rvec)
                angle = -1 * np.degrees(np.arctan2(tvec[0], tvec[2]))
                currAngle = float(round(angle, 1))
    else:
        currDistance = 99.9
        currAngle = 99.9
    
    message = [currDistance, currAngle]
    data = struct.pack('ff', message[0], message[1])
    try:
        i2cARD.write_i2c_block_data(ARD_ADDR, 1, list(data))
    except:
        continue

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
print("Exit")t