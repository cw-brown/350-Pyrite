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

            # Define color ranges for red and green in HSV
            red_range_1 = [(0, 120, 70), (10, 255, 255)]  # Lower red range
            red_range_2 = [(170, 120, 70), (180, 255, 255)]  # Upper red range
            green_range = [(40, 40, 40), (80, 255, 255)]  # Green range

            # Threshold the HSV image to detect red and green colors
            red_mask_1 = cv.inRange(hsv_roi, np.array(red_range_1[0]), np.array(red_range_1[1]))
            red_mask_2 = cv.inRange(hsv_roi, np.array(red_range_2[0]), np.array(red_range_2[1]))
            red_mask = cv.bitwise_or(red_mask_1, red_mask_2)  # Combine both red ranges

            green_mask = cv.inRange(hsv_roi, np.array(green_range[0]), np.array(green_range[1]))

            # Check if any pixels are detected for red or green color
            if np.count_nonzero(red_mask) > 0:
                color_detected = "Red color detected!"
            elif np.count_nonzero(green_mask) > 0:
                color_detected = "Green color detected!"

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
