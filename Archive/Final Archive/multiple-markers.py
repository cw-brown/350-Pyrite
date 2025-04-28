import cv2
import cv2.aruco as aruco
import numpy as np

cap = cv2.VideoCapture(0)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())
print("hi")
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
        # Estimate pose for all detected markers
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        
        # Calculate the distance for each marker (in feet)
        distances = [float(round(tvec[0][2] * 3.28084, 1)) for tvec in tvecs]
        
        # Find the index of the closest marker
        closest_idx = np.argmin(distances)
        
        # Access the closest marker directly
        closest_corners = corners[closest_idx]
        closest_id = ids[closest_idx][0]
        closest_distance = distances[closest_idx]
        
        # Draw the closest marker (green)
        color = (0, 255, 0)  # Green for the closest marker
        c = closest_corners[0]
        
        # Display the distance in feet for the closest marker
        cv2.putText(frame, f"D:{closest_distance:.2f}ft", 
                    (int(c[0][0]), int(c[0][1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Draw the detected marker with the specified color
        # Ensure the 'ids' argument is a numpy array
        aruco.drawDetectedMarkers(frame, [closest_corners], np.array([closest_id]), borderColor=color)

    cv2.imshow('ArUco Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
