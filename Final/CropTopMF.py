import numpy as np
import cv2 as cv
import pickle
import struct
import smbus2
print("hi")
COLOR_THRESH = 1.5
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
def get_color(cnrs, frame):
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
##        print(hsv_left)
        # Define color ranges for red and green in HSV. Need an upper and lower for red, since the H value wraps around from 170 ish, to 0
##        red_range_1 = [(0, 120, 70), (5, 255, 255)]  # Lower red range
##        red_range_2 = [(170, 120, 70), (175, 255, 255)]  # Upper red range
##        green_range = [(50, 40, 40), (70, 255, 255)]  # Green range

##        redLower = np.array([0, 150, 20], dtype=np.uint8)
##        redUpper = np.array([12, 255, 255], dtype=np.uint8)
##        red_range_1 = [redLower, redUpper]
##        redLower2 = np.array([170, 180, 20], dtype=np.uint8)
##        redUpper2 = np.array([180, 255, 255], dtype=np.uint8)
##        red_range_2 = [redLower2, redUpper2]
##        greenLower = np.array([65, 80, 20], dtype=np.uint8)
##        greenUpper = np.array([85, 255, 255], dtype=np.uint8)
##        green_range = [greenLower, greenUpper]

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
                currAngle = get_color(corners, frame)

    # Send the distance and angle to the Arduino
    data = struct.pack('ff', currDistance, currAngle)
    print([currDistance, currAngle])
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
