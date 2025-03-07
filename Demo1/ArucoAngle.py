import numpy as np
import cv2 as cv
from time import sleep, time
import pickle

# Load camera matrix and distortion coefficients
with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist = pickle.load(f)

# Aruco marker dictionary and detector parameters
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
parameters = cv.aruco.DetectorParameters()

cap = cv.VideoCapture(0)

last_print_time = time()
print_interval = 1  # Change this to set how often the angle is printed (in seconds)

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    
    # Undistort frame
    frame = cv.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
    
    # Convert to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Detect Aruco markers
    corners, ids, rejected = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    if ids is not None:
        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, dist)
        
        for i in range(len(ids)):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]
            
            # Compute rotation matrix
            R, _ = cv.Rodrigues(rvec)
            
            # Angle between camera axis and marker
            angle = np.degrees(np.arctan2(tvec[0], tvec[2]))
            
            # Draw marker and axis
            cv.aruco.drawDetectedMarkers(frame, corners)
            cv.drawFrameAxes(frame, cameraMatrix, dist, rvec, tvec, 0.03)
            
            # Print angle every specified interval
            if time() - last_print_time >= print_interval:
                print(f"Marker ID {ids[i][0]}: Angle = {angle:.2f} degrees")
                last_print_time = time()
    
    # Show the frame
    cv.imshow("Aruco Detection", frame)
    
    # Exit on key press
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
