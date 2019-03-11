import cv2
import numpy as np
import math

def nothing(temp):
    return

def distance((x1, y1), (x2, y2)):
    dist = math.sqrt((math.fabs(x2-x1))**2+((math.fabs(y2-y1)))**2)
    return dist

def track():
    rgbMax = 255
    rgbMin = 0

    sliderForFirstObject = "Slider for First Object"
    cv2.namedWindow(sliderForFirstObject)

    sliderForSecondObject = "Slider for Second Object"
    cv2.namedWindow(sliderForSecondObject)

    cam_index = 0 # Default camera is at index 0.
    cap = cv2.VideoCapture(cam_index) # Video capture object

    sliders = ['R_MAX', 'G_MAX', 'B_MAX', 'R_MIN', 'G_MIN', 'B_MIN']

    for slider in sliders:
        cv2.createTrackbar(slider, sliderForFirstObject, rgbMin, rgbMax, nothing)

    for slider in sliders:
        cv2.createTrackbar(slider, sliderForSecondObject, rgbMin, rgbMax, nothing)

    # Sliders for tracking Puck. Set default values after getting the puck!!
    #cv2.setTrackbarPos("R_MAX", sliderForPuck, 93)
    #cv2.setTrackbarPos("R_MIN", sliderForPuck, 36)
    #cv2.setTrackbarPos("G_MAX", sliderForPuck, 188)
    #cv2.setTrackbarPos("G_MIN", sliderForPuck, 51)
    #cv2.setTrackbarPos("B_MAX", sliderForPuck, 198)
    #cv2.setTrackbarPos("B_MIN", sliderForPuck, 73)

    cap.open(cam_index) # Enable the camera
    while True:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            cap.release()
            break
        ret, frame = cap.read()

        thresholds = []
        for slider in sliders:
            thresholds.append(cv2.getTrackbarPos(slider, sliderForFirstObject))

        thresholds2 = []
        for slider in sliders:
            thresholds2.append(cv2.getTrackbarPos(slider, sliderForSecondObject))

        upperBound = np.array(thresholds[0:3])
        lowerBound = np.array(thresholds[3:6])

        upperBound2 = np.array(thresholds2[0:3])
        lowerBound2 = np.array(thresholds2[3:6])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        threshold = cv2.inRange(hsv, lowerBound, upperBound)
        threshold2 = cv2.inRange(hsv, lowerBound2, upperBound2)

        res = cv2.bitwise_and(frame, frame, mask = threshold)
        res2 = cv2.bitwise_and(frame, frame, mask = threshold2)

        im2, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        im3, contours2, hierarchy2 = cv2.findContours(threshold2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        biggestRectangleArea = 0
        biggestRectangleCoordinates = (0,0,0,0)
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            if(w * h > biggestRectangleArea):
                biggestRectangleCoordinates = (x, y, w, h)
                biggestRectangleArea = w*h
        x = biggestRectangleCoordinates[0]
        y = biggestRectangleCoordinates[1]
        w = biggestRectangleCoordinates[2]
        h = biggestRectangleCoordinates[3]
        cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,0),2)
        centerOfFirstObject = ((x+w)//2, (y+h)//2)
        #print(centerOfFirstObject)

        biggestRectangleArea2 = 0
        biggestRectangleCoordinates2 = (0, 0, 0, 0)
        for contour in contours2:
            x2,y2,w2,h2 = cv2.boundingRect(contour)
            if(w2 * h2 > biggestRectangleArea2):
                biggestRectangleCoordinates2 = (x2, y2, w2, h2)
                biggestRectangleArea2 = w2*h2
   #     print(x2, y2, w2, h2)
        x2 = biggestRectangleCoordinates2[0]
        y2 = biggestRectangleCoordinates2[1]
        w2 = biggestRectangleCoordinates2[2]
        h2 = biggestRectangleCoordinates2[3]
        cv2.rectangle(res,(x2,y2),(x2+w2,y2+h2),(255,0, 0),2)
        centerOfSecondObject = ((x2+w2)//2, (y2+h2)//2)

        hypotenuse = distance(centerOfFirstObject, centerOfSecondObject)
        horizontal = distance(centerOfFirstObject, (centerOfSecondObject[0], centerOfFirstObject[1]))
        thirdline = distance(centerOfSecondObject, (centerOfSecondObject[0], centerOfFirstObject[1]))
        try:
            angle = np.arcsin((thirdline/hypotenuse))* 180/math.pi
        except:
            angle = 0
        print(angle)

        #displays all windows
    #    cv2.imshow('Input', frame)  
        cv2.imshow("Objects", frame)

track()
