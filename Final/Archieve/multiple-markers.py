import cv2
import cv2.aruco as aruco
import numpy as np

cap = cv2.VideoCapture(0)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())

marker_length = 0.05  # meters (adjust according to your marker size)
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
camera_matrix = np.array([[frame_width, 0, frame_width / 2],
                          [0, frame_width, frame_height / 2],
                          [0, 0, 1]])
dist_coeffs = np.zeros((4, 1))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        distances = []  # List to store distances of markers
        # Calculate distances for all markers
        for i in range(len(ids)):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_length, camera_matrix, dist_coeffs)
            distance_meters = tvec[0][0][2]  # Distance in meters
            distances.append(distance_meters)

        # Find the index of the closest marker
        closest_idx = np.argmin(distances)
        
        # Draw markers
        for i in range(len(ids)):
            distance_meters = distances[i]
            distance_feet = distance_meters * 3.28084  # Convert meters to feet
            c = corners[i][0]
            
            # Set color based on closest marker
            if i == closest_idx:
                color = (0, 255, 0)  # Green for closest
            else:
                color = (0, 0, 255)  # Red for others

            # Display the ID and distance in feet
            cv2.putText(frame, f"ID:{ids[i][0]} D:{distance_feet:.2f}ft", 
                        (int(c[0][0]), int(c[0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Draw the detected markers with the specified color
            aruco.drawDetectedMarkers(frame, corners, ids, borderColor=color)

    cv2.imshow('ArUco Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
