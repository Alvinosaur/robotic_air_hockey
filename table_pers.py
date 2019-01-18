import cv2 as cv
import numpy as np
#import puck_tracker2

cap = cv.VideoCapture(1);
cap.open(1)
def track_table():
    _, frame = cap.read()
    #frame = cv.imread("./table3.jpg")
    cv.circle(frame, (30, 50), 5, (0, 0, 255), -1)
    cv.circle(frame, (750, 100), 5, (0, 0, 255), -1)
    cv.circle(frame, (30, 475), 5, (0, 0, 255), -1)
    cv.circle(frame, (750, 550), 5, (0, 0, 255), -1)

    pts1 = np.float32([[30, 50], [750, 100], [30, 475], [750, 550]])
    pts2 = np.float32([[0, 0], [800, 0], [0, 400], [800, 400]])
    matrix = cv.getPerspectiveTransform(pts1, pts2)

    result = cv.warpPerspective(frame, matrix, (800, 400))

    cv.imshow("Frame", frame)
    cv.imshow("Perspective transformation", result)

    #puck_tracker2.track_puck(result)
    key = cv.waitKey(1)
    if key == 27:
        return

while True:
    track_table()
cap.release()
cv.destroyAllWindows()
