import cv2 as cv
import numpy as np

# Start video capture
cap = cv.VideoCapture(0)

# Define HSV color ranges
color_ranges = {
    "red": [(0, 120, 70), (10, 255, 255)],  # Red (Hue can be 0-10 or 170-180)
    "blue": [(100, 150, 50), (140, 255, 255)],  # Blue
    "green": [(40, 50, 50), (90, 255, 255)],  # Green
}

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to HSV color space
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    detected_color = "None"
    for color, (lower, upper) in color_ranges.items():
        mask = cv.inRange(hsv, np.array(lower), np.array(upper))
        if cv.countNonZero(mask) > 500:  # Adjust threshold as needed
            detected_color = color
            break

    # Display detected color on the frame
    cv.putText(frame, f"Color: {detected_color}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 
               1, (255, 255, 255), 2, cv.LINE_AA)

    cv.imshow("Color Detection", frame)

    # Exit when 'q' is pressed
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()