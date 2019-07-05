import cv2 as cv
import json
import os
import numpy as np


DEFAULT_CAM_FILE = 'json_files/camera_params.json'
DEFAULT_CAM_INDEX = 1
pattern_size = (7, 7)
dist_btween_corners = 29.4  # cm


def save_dist(pix_to_dist_ratio):
    if not os.path.exists('json_files'):
        os.mkdir('json_files')
    try:
        with open('json_files/pixel_to_dist.json', 'w+') as outfile:  
            json.dump(pix_to_dist_ratio, outfile)
        print('Saved pixel-distance ratio to \'json_files/pixel_to_dist.json\'!')
    except Exception as e:
        print('Couldn\'t save parameters due to: ')
        print(e)
        print('but here is it printed out:')
        print(pix_to_dist_ratio)


def parse_params(cam_file=DEFAULT_CAM_FILE):
    try:
        with open(cam_file, 'r') as f:
            cam_params = json.load(f)
    except IOError as e:
        print('Need to calibrate camera first! Missing file error: %s' % e)
        cam_params = None
    
    return cam_params


cap = cv.VideoCapture(DEFAULT_CAM_INDEX)
cam_params = parse_params()
if cam_params is None: exit(-1)
camera_matrix = np.array(cam_params['camera_matrix'])
dist_coefs = np.array(cam_params['dist_coefs'])


while(True):
    ret, frame = cap.read()
    h, w = frame.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(
        camera_matrix, dist_coefs, (w, h), 1, (w, h))

    frame = cv.undistort(frame, camera_matrix, dist_coefs, None, newcameramtx)

    # crop image
    x, y, w, h = roi
    frame = frame[y:y+h, x:x+w]
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.imshow("Camera's View", gray)

    if cv.waitKey(1) & 0XFF == ord('s'):
        found, corners = cv.findChessboardCorners(gray, pattern_size)
        if found:
            print('Success! Processing...')
            max_pix_dist = 0
            for i in range(len(corners)):
                for j in range(i+1, len(corners)):
                    (x1, y1) = np.ndarray.tolist(corners[i][0])
                    (x2, y2) = np.ndarray.tolist(corners[j][0])
                    dist = ((x2 - x1)**2 + (y2 - y2)**2)**0.5 
                    if dist > max_pix_dist: max_pix_dist = dist
            pix_to_dist_ratio = dist_btween_corners / float(max_pix_dist)
            save_dist(pix_to_dist_ratio)
            break
        else:
            print('Try Again.')


# When everything done, release the capture
cap.release()
cv.destroyAllWindows()