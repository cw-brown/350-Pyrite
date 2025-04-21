import numpy as np
import cv2 as cv
import pickle

# Load camera calibration data
with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist = pickle.load(f)

# Aruco marker setup
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
parameters = cv.aruco.DetectorParameters()

# Initialize camera
cap = cv.VideoCapture(0)

# ---- Color Detection Function with Locking ----
def get_color_code(cnrs, frame, ids):
    # Initialize locked_color on first call
    if not hasattr(get_color_code, "locked_color"):
        get_color_code.locked_color = 0

    # If no marker, reset lock and return 0
    if ids is None or len(ids) == 0:
        get_color_code.locked_color = 0
        return 0

    # Process only the first detected marker
    marker_corners = cnrs[0][0]
    x_min, y_min = np.min(marker_corners, axis=0).astype(int)
    x_max, y_max = np.max(marker_corners, axis=0).astype(int)

    offset = 70
    roi_left  = frame[y_min:y_max, max(0, x_min-offset):x_min]
    roi_right = frame[y_min:y_max, x_max:min(frame.shape[1], x_max+offset)]

    # Gaussian blur to reduce noise
    roi_left_blur  = cv.GaussianBlur(roi_left,  (7, 7), 0) if roi_left.size  > 0 else None
    roi_right_blur = cv.GaussianBlur(roi_right, (7, 7), 0) if roi_right.size > 0 else None

    hsv_left  = cv.cvtColor(roi_left_blur,  cv.COLOR_BGR2HSV) if roi_left_blur  is not None else None
    hsv_right = cv.cvtColor(roi_right_blur, cv.COLOR_BGR2HSV) if roi_right_blur is not None else None

    # HSV ranges
    red_range_1   = [(0, 120, 70),   (5,   255, 255)]
    red_range_2   = [(170, 120, 70), (179, 255, 255)]
    green_range   = [(50, 40, 40),   (70,  255, 255)]
    area_thresh   = 300

    raw_color = 0  # default: no arrow

    # Detect red on the right
    if hsv_right is not None:
        r1 = cv.inRange(hsv_right, np.array(red_range_1[0]), np.array(red_range_1[1]))
        r2 = cv.inRange(hsv_right, np.array(red_range_2[0]), np.array(red_range_2[1]))
        red_mask = cv.bitwise_or(r1, r2)
        red_cnts, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        red_area = sum(cv.contourArea(c) for c in red_cnts)
        if red_area > area_thresh:
            raw_color = 1
        # optional debug:
        if np.count_nonzero(red_mask) > 0:
            mean_r = cv.mean(hsv_right, mask=red_mask)
            print("    Red mean HSV:", tuple(map(int, mean_r[:3])))

    # Detect green on the left
    if hsv_left is not None:
        gmask = cv.inRange(hsv_left, np.array(green_range[0]), np.array(green_range[1]))
        green_cnts, _ = cv.findContours(gmask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        green_area = sum(cv.contourArea(c) for c in green_cnts)
        if green_area > area_thresh:
            raw_color = 2
        # optional debug:
        if np.count_nonzero(gmask) > 0:
            mean_g = cv.mean(hsv_left, mask=gmask)
            print("    Green mean HSV:", tuple(map(int, mean_g[:3])))

    # Update lock only on a real arrow detection
    if raw_color in (1, 2):
        get_color_code.locked_color = raw_color

    return get_color_code.locked_color

# ---- Main Loop ----
print("Press 'q' to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera Error")
        break

    frame = cv.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
    gray  = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    corners, ids, _ = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    color_code = get_color_code(corners, frame, ids)
    print("Detected color code:", color_code)

    if ids is not None:
        cv.aruco.drawDetectedMarkers(frame, corners, ids)

    cv.imshow("Color Detection Test", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
print("Exit")
