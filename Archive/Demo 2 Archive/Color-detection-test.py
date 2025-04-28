import cv2 as cv
import numpy as np

# ArUco marker dictionary and detector parameters
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
parameters = cv.aruco.DetectorParameters()

# Start video capture
cap = cv.VideoCapture(0)  # Capture from the camera

if not cap.isOpened():
    print("Error: Could not open video capture.")
    exit()

while True:
    # Capture a single frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    # Convert the frame to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    color_detected = "No color detected"  # Default message

    if ids is not None:
        for i in range(len(ids)):
            # Get the coordinates of the marker corners
            marker_corners = corners[i][0]
            x_min, y_min = np.min(marker_corners, axis=0).astype(int)
            x_max, y_max = np.max(marker_corners, axis=0).astype(int)

            # Draw the detected marker on the frame
            cv.aruco.drawDetectedMarkers(frame, corners, ids)

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
            hsv_right = cv.cvtColor(roi_right, cv.COLOR_BGR2HSV)
            hsv_left = cv.cvtColor(roi_left, cv.COLOR_BGR2HSV)

            # Define color ranges for red and green in HSV
            red_range_1 = [(0, 120, 70), (10, 255, 255)]  # Lower red range
            red_range_2 = [(170, 120, 70), (180, 255, 255)]  # Upper red range
            green_range = [(40, 40, 40), (80, 255, 255)]  # Green range

            # Threshold the HSV images to detect red and green colors
            red_mask_right_1 = cv.inRange(hsv_right, np.array(red_range_1[0]), np.array(red_range_1[1]))
            red_mask_right_2 = cv.inRange(hsv_right, np.array(red_range_2[0]), np.array(red_range_2[1]))
            red_mask_right = cv.bitwise_or(red_mask_right_1, red_mask_right_2)  # Combine both red ranges

            green_mask_left = cv.inRange(hsv_left, np.array(green_range[0]), np.array(green_range[1]))

            # Check if any pixels are detected for red or green color in the right or left regions
            if np.count_nonzero(red_mask_right) > 0:
                color_detected = "Red"
            elif np.count_nonzero(green_mask_left) > 0:
                color_detected = "Green"

    # Display the detected color on the frame
    cv.putText(frame, color_detected, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv.LINE_AA)

    # Show the live feed with ArUco detection and the color message
    cv.imshow("ArUco and Color Detection", frame)

    # Exit the loop if 'q' is pressed
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv.destroyAllWindows()
