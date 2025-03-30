import cv2 as cv
import numpy as np

# ArUco marker dictionary and detector parameters
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
parameters = cv.aruco.DetectorParameters()

# Start video capture
cap = cv.VideoCapture(0)  # Capture from the camera

# Capture a single frame
ret, frame = cap.read()  # Capture a single frame
if not ret:
    print("Failed to capture frame")
    cap.release()
    cv.destroyAllWindows()
    exit()

# Convert the frame to grayscale
gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

# Detect ArUco markers
corners, ids, rejected = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)

if ids is not None:
    for i in range(len(ids)):
        # Get the coordinates of the marker corners
        marker_corners = corners[i][0]
        x_min, y_min = np.min(marker_corners, axis=0).astype(int)
        x_max, y_max = np.max(marker_corners, axis=0).astype(int)

        # Draw the detected marker on the frame
        cv.aruco.drawDetectedMarkers(frame, corners, ids)

        # Define the region to the right of the marker (with an offset)
        offset = 20  # Pixels to shift the region to the right
        roi_x_min = max(0, x_max + offset)
        roi_x_max = min(frame.shape[1], roi_x_min + (x_max - x_min))
        roi_y_min = y_min
        roi_y_max = y_max

        # Extract the region of interest (ROI) to the right of the marker
        roi = frame[roi_y_min:roi_y_max, roi_x_min:roi_x_max]

        # Convert the ROI to HSV color space
        hsv_roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

        # Define the color range (e.g., red)
        color_range = [(0, 120, 70), (10, 255, 255)]  # Red color range in HSV

        # Threshold the HSV image to detect red color
        mask = cv.inRange(hsv_roi, np.array(color_range[0]), np.array(color_range[1]))
        result = cv.bitwise_and(roi, roi, mask=mask)

        # Replace the ROI with the color-detected result in the original frame
        frame[roi_y_min:roi_y_max, roi_x_min:roi_x_max] = result

# Show the final frame with both the ArUco marker and color-detected region in one window
cv.imshow("ArUco and Color Detection", frame)

# Wait for the user to press a key and then close the window
cv.waitKey(0)
cv.destroyAllWindows()

cap.release()
