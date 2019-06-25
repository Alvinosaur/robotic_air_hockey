'''import cv2
import numpy

CAM_INDEX = 0; #default camera index is 0
RGBMIN = 0;
RGBMAX = 255;
SLIDERS = ['R_MAX', 'G_MAX', 'B_MAX', 'R_MIN', 'G_MIN', 'B_MIN'];

def track_puck(cam, averaging_func):
    if cv2.waitKey(1) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        cap.release()
        return
    ret, frame = cam.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #cv2.imshow("HSV", hsv)

    location_array = [] #array of (x,y,r) to average out
    upperBlue = numpy.uint8([[[110,50,50]]])
    hsv_upper = cv2.cvtColor(upperBlue, cv2.COLOR_BGR2HSV)
    lowerBlue = numpy.uint8([[[255,130,130]]])
    hsv_lower = cv2.cvtColor(lowerBlue, cv2.COLOR_BGR2HSV)

        #upperBound = numpy.array()
    threshold = cv2.inRange(hsv, hsv_lower, hsv_upper)
    res = cv2.bitwise_and(frame, frame, mask = threshold)

    cv2.imshow("Res", res)

    blurred = cv2.GaussianBlur(res, (5, 5), 0)
    canny = cv2.Canny(blurred, 230, 255)

    cv2.imshow("Canny", canny)

    im2, contours, hierarchy = cv2.findContours(canny.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    frame2 = frame.copy()
    cv2.drawContours(frame2, contours, -1, (0, 255, 0), 2)
    cv2.imshow("Contours", frame2)
    if len(contours) is not 0:
        c = max(contours, key = cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        centerOfPuck = (int((2*x + w)//2), int((2 * y + h)//2))
        cv2.circle(frame, centerOfPuck, 1, (0, 255, 0), 10)
    cv2.imshow("Objects", frame)

#track(obj_array)
cap = cv2.VideoCapture(CAM_INDEX);
cap.open(CAM_INDEX) #Open the camera
while True:
    track_puck(cap, None)
'''

import cv2 as cv
import json
import numpy as np

CAM_INDEX = 1; # default camera index is 0
OBJ_HIGHLIGHT_COLOR = (0, 255, 0)  # Green
OBJ_HIGHLIGHT_LINE_WIDTH = 2
OBJ_CENTER_COLOR = (255, 0, 0)  # RED


def track_puck(cam, thresholds):
    if cv.waitKey(1) & 0xFF == ord("q"):
        cv.destroyAllWindows()
        cam.release()
        return
    _, frame = cam.read()
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    #applying a mask to get only lower_green
    for obj_name in thresholds.keys():
        thresholds[obj_name]['mask'] = cv.inRange(hsv, 
            np.array(thresholds[obj_name]['lower']), 
            np.array(thresholds[obj_name]['upper']))

        filtered_img = cv.bitwise_and(frame, frame, mask = thresholds[obj_name]['mask'])

        # Find target using edge detection and contours
        blurred = cv.GaussianBlur(filtered_img, (5, 5), 0)
        canny = cv.Canny(blurred, 230, 255)
        _, contours, hierarchy = cv.findContours(canny.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # -1 to draw all detected contours
        cv.drawContours(frame, contours, -1, OBJ_HIGHLIGHT_COLOR, OBJ_HIGHLIGHT_LINE_WIDTH)

        #find center of puck if contour exists
        if len(contours) is not 0:
            c = max(contours, key = cv.contourArea)
            x, y, w, h = cv.boundingRect(c)
            cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            centerOfPuck = (int((2*x + w)//2), int((2*y + h)//2))
            cv.circle(frame, centerOfPuck, 1, OBJ_CENTER_COLOR, 10)

        cv.imshow(obj_name, frame)

try:
    with open('json_files/object_hsv_filters.json', 'r') as f:
        object_hsv_params = json.load(f)
except IOError:
    print('Need to calibrate camera first! Please run \'calibrate_camera.py\'')
    exit(-1)

# Open the camera
cap = cv.VideoCapture(CAM_INDEX)
cap.open(CAM_INDEX)

while True:
    track_puck(cap, object_hsv_params)