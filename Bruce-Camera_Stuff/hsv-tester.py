import cv2

# Global variables to store frame
frame = None
hsv_frame = None

# Mouse callback function
def show_hsv_values(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if hsv_frame is not None:
            # Get the HSV values at the clicked point
            h, s, v = hsv_frame[y, x]
            print(f"HSV at ({x}, {y}): H={h}, S={s}, V={v}")

# Start video capture
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not access webcam.")
    exit()

# Create a window and set the mouse callback
cv2.namedWindow("Webcam Feed")
cv2.setMouseCallback("Webcam Feed", show_hsv_values)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame.")
        break

    # Convert to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Show the webcam feed
    cv2.imshow("Webcam Feed", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
