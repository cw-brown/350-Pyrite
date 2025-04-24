import cv2

# Open a connection to the webcam (0 = default camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    # If frame was read correctly, display it
    if not ret:
        print("Error: Failed to read frame.")
        break

    # Show the frame in a window
    cv2.imshow("Webcam Feed", frame)

    # Break loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release webcam and close the window
cap.release()
cv2.destroyAllWindows()
