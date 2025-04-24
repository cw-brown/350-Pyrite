import cv2
from PIL import Image
import numpy as np

from util import get_limits


# Aruco stuff
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
parameters = cv2.aruco.DetectorParameters()

cap = cv2.VideoCapture(0)

while True:

    ret, frame = cap.read()

    if not ret:
        print("Failed to capture frame")
        cap.release()
        break

    # Convert to Gray scale to find aruco
    grayImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Convert to HSV to find color
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Crop the frame when an Aruco Marker is detected
    # Detect Aruco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(grayImage, dictionary, parameters=parameters)


    # lowerLimit, upperLimit = get_limits(color=yellow)
    redLower = np.array([0, 150, 20], dtype=np.uint8)
    redUpper = np.array([12, 255, 255], dtype=np.uint8)
    redLower2 = np.array([170, 180, 20], dtype=np.uint8)
    redUpper2 = np.array([180, 255, 255], dtype=np.uint8)
    greenLower = np.array([65, 80, 20], dtype=np.uint8)
    greenUpper = np.array([90, 255, 255], dtype=np.uint8)
    # rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, dist)

    if ids is not None: # We see markers
        for i in range(len(ids)):
            color = None
            # Get the coordinates of the marker corners
            marker_corners = corners[i][0]
            x_min, y_min = np.min(marker_corners, axis=0).astype(int)
            x_max, y_max = np.max(marker_corners, axis=0).astype(int)

            # Define the region to the right and left of the marker (with an offset)
            offset = 70  # Pixels to shift the region to the right or left
            roi_x_min_left = max(0, x_min - offset)
            roi_x_max_left = x_min
            roi_x_min_right = x_max
            roi_x_max_right = min(frame.shape[1], x_max + offset)
            roi_y_min = y_min
            roi_y_max = y_max

            # Extract the region of interest (ROI) for the right side of the marker (red arrow)
            roi_right = hsvImage[roi_y_min:roi_y_max, roi_x_min_right:roi_x_max_right]
            roi_left = hsvImage[roi_y_min:roi_y_max, roi_x_min_left:roi_x_max_left]
            cropped_frame = hsvImage[roi_y_min:roi_y_max, roi_x_min_left:roi_x_max_right]
            # cv2.imshow("Aruco Detection", cropped_frame)
            # croppedHSV = cv2.cvtColor(cropped_frame,cv2.COLOR_BGR2HSV)
            # cv2.imshow("Aruco Detection", croppedHSV)
            # Convert both ROIs to HSV color space
            # hsv_right = cv.cvtColor(roi_right, cv.COLOR_BGR2HSV) if roi_right.size > 0 else None
            # hsv_left = cv.cvtColor(roi_left, cv.COLOR_BGR2HSV) if roi_left.size > 0 else None
            redMask = cv2.inRange(cropped_frame, redLower, redUpper)
            redMask2 = cv2.inRange(cropped_frame, redLower2, redUpper2)
            greenMask = cv2.inRange(cropped_frame, greenLower, greenUpper)
            cv2.imshow("Red Mask", redMask)
            cv2.imshow("Red Mask 2", redMask2)
            cv2.imshow("Green Mask", greenMask)


    

    # mask = cv2.inRange(hsvImage, upperLimit, lowerLimit)

    redMask = cv2.inRange(hsvImage, redLower, redUpper)
    redMask2 = cv2.inRange(hsvImage, redLower2, redUpper2)
    greenMask = cv2.inRange(hsvImage, greenLower, greenUpper)

    # cv2.imshow("Red Mask", redMask)
    # cv2.imshow("Red Mask 2", redMask2)
    # cv2.imshow("Green Mask", greenMask)

    # mask_ = Image.fromarray(mask)
    mask_ = Image.fromarray(redMask)

    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox

        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

    # cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()

