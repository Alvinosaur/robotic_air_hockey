import sys
print(sys.path);
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')


import cv2 as cv
import numpy as np
import puck_tracker2
import mahotas

CAM_INDEX = 1;
cap = cv.VideoCapture(CAM_INDEX);
cap.open(CAM_INDEX) #Open the camera

def captureTable(cam):
    if cv.waitKey(1) & 0xFF == ord("q"):
        cv.destroyAllWindows()
        cam.release()
        return
    ret, frame = cam.read()
    #frame = cv.imread("./table2.jpg")
    #cv.imshow("Frame", frame)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    #cv.imshow("Gray", gray)
    blurred = cv.GaussianBlur(gray, (21, 21), 0)
    blurred = cv.bitwise_not(blurred)
    #cv.imshow("b", blurred)

    T = mahotas.thresholding.otsu(blurred)
    #T = 160
    print("Otsu's Threshold: {}".format(T))
    thresh = blurred.copy()
    thresh[thresh > T] = 255
    thresh[thresh < T] = 0

    #cv.imshow("Thresh", thresh)

    canny = cv.Canny(thresh, 130, 170)
    #cv.imshow("Canny", canny)

    im2, contours, hierarchy = cv.findContours(canny.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        cnt = max(contours, key = cv.contourArea)
        epsilon = 0.01*cv.arcLength(cnt,True)
        approx = cv.approxPolyDP(cnt,epsilon,True)
        if len(approx) == 4:
            cv.drawContours(frame, [approx], -1, (0, 255, 0), 2)
        else:
            print("no table found")
    cv.imshow("Contours", frame)
    #puck_tracker2.track_puck(frame)

'''
    lower_white = np.array([0,0,0], dtype=np.uint8)
    upper_white = np.array([0,0,255], dtype=np.uint8)
    white_mask = cv.inRange(hsv, lower_white, upper_white)

    lower_red = np.uint8([[[110, 50, 50]]])
    upper_red = np.uint8([[[255, 130, 130]]])
    lower_red_hsv = cv.cvtColor(lower_red,cv.COLOR_BGR2HSV)
    upper_red_hsv = cv.cvtColor(upper_red,cv.COLOR_BGR2HSV)
    red_mask = cv.inRange(hsv, lower_red_hsv, upper_red_hsv)

    mask = white_mask + red_mask

    res = cv.bitwise_not(frame)
    res = cv.bitwise_and(res,res, mask= mask)
    cv.imshow("Res", res)

    blurred = cv.GaussianBlur(res, (5, 5), 0)
    canny = cv.Canny(blurred, 230, 255)
    cv.imshow("Canny", canny)

    im2, contours, hierarchy = cv.findContours(canny.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    frame2 = frame.copy()
    cv.drawContours(frame2, contours, -1, (0, 255, 0), 2)
    cv.imshow("Contours", frame2)
'''

while True:
    captureTable(cap)
