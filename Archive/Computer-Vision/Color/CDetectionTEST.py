# find the color on the marker
def get_color(cnrs, frame, ids):
    for i in range(len(ids)):
        # Get the coordinates of the marker corners
        marker_corners = cnrs[i][0]
        x_min, y_min = np.min(marker_corners, axis=0).astype(int)
        x_max, y_max = np.max(marker_corners, axis=0).astype(int)

        # Define the region to the right and left of the marker (with an offset)
        offset = 70  # Pixels to shift the region to the right or left
        roi_x_min_left = max(0, x_min - offset)
        roi_x_max_left = x_min
        roi_x_min_right = x_max
        roi_x_max_right = min(frame.shape[1], x_max + offset)
        roi_y_min = y_min
        roi_y_max = y_max

        # Extract ROI for both sides
        roi_right = frame[roi_y_min:roi_y_max, roi_x_min_right:roi_x_max_right]
        roi_left = frame[roi_y_min:roi_y_max, roi_x_min_left:roi_x_max_left]

        # Convert to HSV
        hsv_right = cv.cvtColor(roi_right, cv.COLOR_BGR2HSV) if roi_right.size > 0 else None
        hsv_left = cv.cvtColor(roi_left, cv.COLOR_BGR2HSV) if roi_left.size > 0 else None

        # Color ranges
        red_range_1 = [(0, 120, 70), (5, 255, 255)]
        red_range_2 = [(170, 120, 70), (180, 255, 255)]
        green_range = [(50, 40, 40), (70, 255, 255)]

        # Threshold
        red_mask_right = None
        green_mask_left = None
        if hsv_right is not None:
            red_mask_right_1 = cv.inRange(hsv_right, np.array(red_range_1[0]), np.array(red_range_1[1]))
            red_mask_right_2 = cv.inRange(hsv_right, np.array(red_range_2[0]), np.array(red_range_2[1]))
            red_mask_right = cv.bitwise_or(red_mask_right_1, red_mask_right_2)
        
        if hsv_left is not None:
            green_mask_left = cv.inRange(hsv_left, np.array(green_range[0]), np.array(green_range[1]))
        
        # Decision logic
        if red_mask_right is not None and np.count_nonzero(red_mask_right) > 0:
            print("Detected Color Code: 1 (Red)")
            return 1  # Red arrow (right)

        elif green_mask_left is not None and np.count_nonzero(green_mask_left) > 0:
            print("Detected Color Code: 2 (Green)")
            return 2  # Green arrow (left)

    print("Detected Color Code: 0 (No color)")
    return 0  # No color detected
