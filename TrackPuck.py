import cv2
import numpy as np
import time

class Puck(object):
    def __init__(self):
        self.frame = None
        self.x = None
        self.y = None
        self.dy = 0
        self.dx = 0
        self.time = 0

    def track(self):
        upperBound = np.array([[[255, 255, 255]]])
        lowerBound = np.array([[[103, 113, 91]]])

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        threshold = cv2.inRange(hsv, lowerBound, upperBound)
        threshold = cv2.erode(threshold, None, iterations=2)
        threshold = cv2.dilate(threshold, None, iterations=2)

        res = cv2.bitwise_and(self.frame, self.frame, mask = threshold)

        blurred = cv2.GaussianBlur(res, (5, 5), 0)
        canny = cv2.Canny(blurred, 230, 255)

        im2, contours, hierarchy = cv2.findContours(canny.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            curr_time = time.time()
            cv2.rectangle(self.frame,(x,y),(x+w,y+h),(0,255,0),2)
            (curr_x, curr_y) = (int((2*x + w)//2), int((2*y + h)//2))
            cv2.circle(self.frame, (curr_x, curr_y), 1, (0, 255, 0), 10)
            if self.x == None:
                self.x = curr_x
                self.y = curr_y
            move_x = curr_x - self.x
            move_y = curr_y - self.y
            time_diff = curr_time - self.time
            print(time_diff)
            if time_diff != 0:
                self.dx = move_x / time_diff
                self.dy = move_y / time_diff
            self.time = curr_time
            self.x = curr_x
            self.y = curr_y
        print("X velocity: ", self.dx)
        print("Y velocity: ", self.dy)
            #displays all windows
        cv2.imshow('Input', self.frame)
        #    cv2.imshow("Puck", res)
