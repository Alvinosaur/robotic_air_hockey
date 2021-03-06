import numpy as np
import cv2 
import cv2.cv as cv
im_width = 320
im_height = 240
cap = cv2.VideoCapture(0)
cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, im_width)
cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, im_height)
cv.NamedWindow(“Video”, 0)

color_bounds = dict()
color_bounds['green'] = {'upper': cv.Scalar(0, 255, 0)}

# The order of the colors is blue, green, red
lower_color_bounds = cv.Scalar(100, 0, 0)
upper_color_bounds = cv.Scalar(225,80,80)
print ‘Press <q> to quit’
while(True):
    ret,frame = cap.read()
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(frame,lower_color_bounds,upper_color_bounds )
    mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    frame = frame & mask_rgb
    cv2.imshow(‘Video’,frame)
    if(cv2.waitKey(1) & 0xFF == ord(‘q’)):
    cv.DestroyWindow(“video”) 
    break
cap.release()
cv2.destroyAllWindows()
