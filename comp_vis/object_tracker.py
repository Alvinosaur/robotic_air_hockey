from collections import namedtuple
import cv2 as cv
import json
import numpy as np
import time

from pose_2D_kalman_filter import Pose2D


DEFAULT_HSV_FILE = 'json_files/object_hsv_filters.json'
DEFAULT_CAM_FILE = 'json_files/camera_params.json'
DEFAULT_PIX_DIST_FILE = 'json_files/pixel_to_dist.json'
DEFAULT_CAM_INDEX = 1


def parse_params(hsv_filter_file=DEFAULT_HSV_FILE, cam_file=DEFAULT_CAM_FILE,
                pix_dist_file=DEFAULT_PIX_DIST_FILE):
    try:
        with open(hsv_filter_file, 'r') as f:
            obj_filters = json.load(f)
        with open(cam_file, 'r') as f:
            cam_params = json.load(f)
        with open(pix_dist_file, 'r') as f:
            pix_to_dist = json.load(f)
    except IOError as e:
        print('Need to calibrate camera first! Missing file error: %s' % e)
        obj_filters = None
        cam_params = None
    
    return obj_filters, cam_params, pix_to_dist


class ObjTracker:
    OBJ_HIGHLIGHT_COLOR = (0, 255, 0)  # Green
    OBJ_HIGHLIGHT_LINE_WIDTH = 2
    OBJ_CENTER_COLOR = (255, 0, 0)  # RED

    def __init__(self, cam_index, obj_name, cam_params, pix_to_dist,
                 hsv_lower_thresh, hsv_upper_thresh):
        # camera
        self.cam_index = cam_index
        self.cap = cv.VideoCapture(cam_index)
        self.is_cam_open = False
        self.prev_time = 0
        self.camera_matrix = np.array(cam_params['camera_matrix'])
        self.dist_coefs = np.array(cam_params['dist_coefs'])
        self.pix_to_dist = pix_to_dist

        # object
        self.obj_name = obj_name
        self.pose = None  # Pose2D(), init once detect object

        # image processing
        self.hsv_upper_thresh = hsv_upper_thresh
        self.hsv_lower_thresh = hsv_lower_thresh

    def close_cam(self):
        cv.destroyAllWindows()
        self.cap.release()
        return


    # only call if table senses that puck is in play
    # don't track puck if table detects puck has been scored
    def track_obj(self, visualize=False):
        if not self.is_cam_open:
            self.is_cam_open = True
            self.cap.open(self.cam_index)

        _, frame = self.cap.read()
        h, w = frame.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coefs, (w, h), 1, (w, h))

        frame = cv.undistort(frame, self.camera_matrix, self.dist_coefs, None, newcameramtx)

        # crop image
        x, y, w, h = roi
        frame = frame[y:y+h, x:x+w]

        # Filter HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv_mask = cv.inRange(hsv, self.hsv_lower_thresh, self.hsv_upper_thresh)

        filtered_img = cv.bitwise_and(frame, frame, mask=hsv_mask)
        cur_time = time.time()
        dt = cur_time - self.prev_time  # seconds

        # Find target using edge detection and contours
        blurred = cv.GaussianBlur(filtered_img, (5, 5), 0)
        canny = cv.Canny(blurred, 230, 255)
        _, contours, hierarchy = (
            cv.findContours(canny.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE))

        # Find center of obj if contour exists
        if len(contours) != 0:
            c = max(contours, key = cv.contourArea)
            x, y, w, h = cv.boundingRect(c)
            cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            true_x, true_y = x * self.pix_to_dist, y * self.pix_to_dist
            center = (int((2*x + w)//2), int((2*y + h)//2))
            if visualize: 
                cv.circle(frame, center, 1, ObjTracker.OBJ_CENTER_COLOR, 10)
                cv.drawContours(frame, [c], 0,
                                ObjTracker.OBJ_HIGHLIGHT_COLOR, 
                                ObjTracker.OBJ_HIGHLIGHT_LINE_WIDTH)
            
            if self.pose != None:
                vx = (self.pose.getX() - true_x) / dt 
                vy = (self.pose.getY() - true_y) / dt
                est_state = np.array([[true_x], [true_y], [vx], [vy]])
                self.pose.estimate_state(est_state, dt)
                print(self.pose.X)
            else:
                self.pose = Pose2D(self.obj_name, true_x, true_y)

        else:  
            # maybe use other heuristics to determine if we actually detected puck
            # maybe in calibration, it finds the most likely width and height of 
            # target object and can filter contours based on this prediction
            if self.pose != None:
                self.pose.estimate_state(None, dt)

        if visualize:
            cv.imshow(self.obj_name, frame)

        self.prev_time = cur_time


def test():
    obj_name = 'Second Joint Filter'
    hsv_filters, cam_params, pix_to_dist = parse_params()
    obj = ObjTracker(DEFAULT_CAM_INDEX, obj_name, cam_params, pix_to_dist,
                     np.array(hsv_filters[obj_name]['lower']),
                     np.array(hsv_filters[obj_name]['upper']))

    while True:
        obj.track_obj(True)
        if cv.waitKey(1) & 0xFF == ord("q"):
            obj.close_cam()

test()

            