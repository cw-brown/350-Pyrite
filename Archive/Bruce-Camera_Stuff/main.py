import cv2
from PIL import Image
import numpy as np

from util import get_limits


# yellow = [0, 0, 0]  # red in BGR colorspace
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        cap.release()
        break

    

    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # lowerLimit, upperLimit = get_limits(color=yellow)
    redLower = np.array([0, 150, 20], dtype=np.uint8)
    redUpper = np.array([12, 255, 255], dtype=np.uint8)
    redLower2 = np.array([170, 180, 20], dtype=np.uint8)
    redUpper2 = np.array([180, 255, 255], dtype=np.uint8)
    greenLower = np.array([65, 80, 20], dtype=np.uint8)
    greenUpper = np.array([90, 255, 255], dtype=np.uint8)

    # mask = cv2.inRange(hsvImage, upperLimit, lowerLimit)

    redMask = cv2.inRange(hsvImage, redLower, redUpper)
    redMask2 = cv2.inRange(hsvImage, redLower2, redUpper2)
    greenMask = cv2.inRange(hsvImage, greenLower, greenUpper)

    cv2.imshow("Red Mask", redMask)
    cv2.imshow("Red Mask 2", redMask2)
    cv2.imshow("Green Mask", greenMask)

    # mask_ = Image.fromarray(mask)
    mask_ = Image.fromarray(redMask)

    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox

        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

    # cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()

