import cv2 as cv
import numpy as np
import pickle

# Load camera calibration
with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist = pickle.load(f)

# ArUco setup
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
parameters = cv.aruco.DetectorParameters()

# Start camera
cap = cv.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    frame = cv.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    corners, ids, _ = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is not None and len(corners) > 0:
        marker_corners = corners[0][0]
        x_min, y_min = np.min(marker_corners, axis=0).astype(int)
        x_max, y_max = np.max(marker_corners, axis=0).astype(int)

        # Tight vertical crop, wide horizontal crop
        vertical_pad = 10
        horizontal_pad = 100

        x_min = max(0, x_min - horizontal_pad)
        y_min = max(0, y_min - vertical_pad)
        x_max = min(frame.shape[1], x_max + horizontal_pad)
        y_max = min(frame.shape[0], y_max + vertical_pad)

        cropped_frame = frame[y_min:y_max, x_min:x_max]
        cv.imshow("Cropped Marker View", cropped_frame)

    # Show full frame for context
    cv.imshow("Full Frame", frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
