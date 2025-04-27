import numpy as np
import cv2 as cv
import pickle
import struct
import smbus2
print("hi")
COLOR_THRESH = 1
DETECTION_THRESH = 5
ARD_ADDR = 8  
i2cARD = smbus2.SMBus(1)  # Use I2C bus 1 for communication with Arduino

cap = cv.VideoCapture(0)

# Load camera calibration data
with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist = pickle.load(f)

dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
parameters = cv.aruco.DetectorParameters()

# Function to calculate the angle to the marker
def get_angle(rvec, tvec):
    # Compute rotation matrix
    R, _ = cv.Rodrigues(rvec)
    angle = -1 * np.degrees(np.arctan2(tvec[0], tvec[2]))
    return float(round(angle, 1))

# Function to calculate distance to marker
def get_distance(tvec):
    distance = float(round(tvec[2] * 3.28, 1))  # Convert from meters to feet (Z component of tvec)
    return distance

# Main loop for capturing frames and processing markers
while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera Error")
        continue

    frame = cv.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.imshow("Window", gray)

    corners, ids, rejected = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    # Initialize default values for currDistance and currAngle
    currDistance = 99.9
    currAngle = 99.9

    if ids is not None:
        # Get rotation and translation vectors for all markers
        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, dist)

        # Calculate distances for all markers and find the closest one
        distances = [float(round(tvec[0][2] * 3.28, 1)) for tvec in tvecs]
        min_distance_index = np.argmin(distances)

        # Get the distance and angle of the closest marker
        currDistance = get_distance(tvecs[min_distance_index][0])
        currAngle = get_angle(rvecs[min_distance_index][0], tvecs[min_distance_index][0])

        # Handle thresholding and send data to Arduino
        if currDistance > DETECTION_THRESH:
            currDistance = 99.9  # No marker found within threshold
            currAngle = 99.9     # No angle calculated
        else:
            # If within detection range, you can perform color detection if needed
            if currDistance <= COLOR_THRESH:
                currAngle = get_angle(rvecs[min_distance_index][0], tvecs[min_distance_index][0])

    # Send the distance and angle to the Arduino
    data = struct.pack('ff', currDistance, currAngle)
    # print(currDistance)
    # Send data to Arduino
    try:
        i2cARD.write_i2c_block_data(ARD_ADDR, 1, list(data))
    except:
        continue

    # Quit the loop when 'q' is pressed
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv.destroyAllWindows()
print("Exit")
