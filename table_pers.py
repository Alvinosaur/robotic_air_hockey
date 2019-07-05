import cv2 as cv
import numpy as np
from TrackPuck import Puck

cap = cv.VideoCapture(1);
cap.open(1)
PuckTracker = Puck()

def track_table():
    _, frame = cap.read()
    #frame = cv.imread("./table3.jpg")
    cv.circle(frame, (203, 104), 5, (0, 0, 255), -1)
    cv.circle(frame, (479, 90), 5, (0,  0, 255), -1)
    cv.circle(frame, (527, 466), 5, (0, 0, 255), -1)
    cv.circle(frame, (180, 480), 5, (0, 0, 255), -1)

    pts1 = np.float32([[203, 104], [479, 90], [527, 466], [180, 480]])
    pts2 = np.float32([[0, 380], [0, 0], [460, 0], [460, 380]])
    matrix = cv.getPerspectiveTransform(pts1, pts2)

    result = cv.warpPerspective(frame, matrix, (460, 360))

    cv.imshow("Frame", frame)
    cv.imshow("Perspective transformation", result)

    PuckTracker.frame = result
    PuckTracker.track()
    key = cv.waitKey(1)
    if key == 27:
        return

while True:
    track_table()
cap.release()
cv.destroyAllWindows()
