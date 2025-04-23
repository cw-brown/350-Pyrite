def get_color_code(cnrs, frame, ids):
    for i in range(len(ids)):
        marker_corners = cnrs[i][0]
        x_min, y_min = np.min(marker_corners, axis=0).astype(int)
        x_max, y_max = np.max(marker_corners, axis=0).astype(int)

        offset = 70
        roi_left = frame[y_min:y_max, max(0, x_min - offset):x_min]
        roi_right = frame[y_min:y_max, x_max:min(frame.shape[1], x_max + offset)]

        # Blur to reduce noise
        roi_left_blur = cv.GaussianBlur(roi_left, (7, 7), 0)
        roi_right_blur = cv.GaussianBlur(roi_right, (7, 7), 0)

        hsv_left = cv.cvtColor(roi_left_blur, cv.COLOR_BGR2HSV)
        hsv_right = cv.cvtColor(roi_right_blur, cv.COLOR_BGR2HSV)

        # Red ranges (combined)
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([179, 255, 255])

        # Green range
        green_lower = np.array([45, 100, 100])
        green_upper = np.array([85, 255, 255])

        # Create masks
        red_mask1 = cv.inRange(hsv_right, red_lower1, red_upper1)
        red_mask2 = cv.inRange(hsv_right, red_lower2, red_upper2)
        red_mask = cv.bitwise_or(red_mask1, red_mask2)

        green_mask = cv.inRange(hsv_left, green_lower, green_upper)

        # Filter small blobs
        red_cnts, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        green_cnts, _ = cv.findContours(green_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        red_large = any(cv.contourArea(c) > 200 for c in red_cnts)
        green_large = any(cv.contourArea(c) > 200 for c in green_cnts)

        if red_large:
            return 1  # Red
        elif green_large:
            return 2  # Green
        else:
            return 0  # Marker present, no clear arrow

    return 0  # No marker
