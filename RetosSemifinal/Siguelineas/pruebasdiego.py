# Source original: https://gist.github.com/flyboy74/21e38dec4bbb77eaf8d729e6deba8d14

import time
import cv2
import numpy as np

# Constants
ROI_START_Y = 200
ROI_END_Y = 250
ROI_START_X = 0
ROI_END_X = 639
BLACKLINE_COLOR_RANGE_LOW = (0, 0, 0)
BLACKLINE_COLOR_RANGE_HIGH = (50, 50, 50)
KERNEL_SIZE = (3, 3)
EROSION_ITERATIONS = 5
DILATION_ITERATIONS = 9
LINE_COLOR = (255, 0, 0)
LINE_THICKNESS = 3
SETPOINT = 320


# Open the camera
camera = cv2.VideoCapture(0)
if not camera.isOpened():
    print("Could not open camera")
    exit(1)

# Allow the camera to warm up
time.sleep(0.1)

try:
    while True:
        # Capture frame-by-frame
        ret, image = camera.read()
        if not ret:
            break

        # Define region of interest
        # roi = image[ROI_START_Y:ROI_END_Y, ROI_START_X:ROI_END_X]

        # Detect black line
        blackline_mask = cv2.inRange(image, BLACKLINE_COLOR_RANGE_LOW, BLACKLINE_COLOR_RANGE_HIGH)    # Use roi to improve performance
        kernel = np.ones(KERNEL_SIZE, np.uint8)
        blackline_mask = cv2.erode(blackline_mask, kernel, iterations=EROSION_ITERATIONS)
        blackline_mask = cv2.dilate(blackline_mask, kernel, iterations=DILATION_ITERATIONS)

        # Find contours
        contours, _ = cv2.findContours(blackline_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw a line in the middle of the largest contour and calculate error
        if contours:
            blackbox = cv2.minAreaRect(contours[0])
            
            (x_min, y_min), (w_min, h_min), ang = blackbox
            if ang < -45:
                ang = 90 + ang
            if w_min < h_min and ang > 0:
                ang = (90 - ang) * -1
            if w_min > h_min and ang < 0:
                ang = 90 + ang
            error = int(x_min - SETPOINT)
            ang = int(ang)
            box = cv2.boxPoints(blackbox)
            box = np.int0(box)
            cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
            cv2.putText(image, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(image, str(error), (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.line(image, (int(x_min), ROI_START_Y), (int(x_min), ROI_END_Y), LINE_COLOR, LINE_THICKNESS)

        # Display the resulting frame
        cv2.imshow("Original with line", image)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # When everything done, release the capture
    camera.release()
    cv2.destroyAllWindows()
