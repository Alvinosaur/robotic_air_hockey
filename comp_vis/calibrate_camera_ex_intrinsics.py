#!/usr/bin/env python

'''
NOTE: Taken from opencv repository (Path: opencv/samples/python/calibrate.py)
camera calibration for distorted images with chess board samples
reads distorted images, calculates the calibration and write undistorted images
usage:
    calibrate.py [--debug <output path>] [--square_size] [<image mask>]
default values:
    --debug:    ./output/
    --square_size: 1.0
    <image mask> defaults to imgs/calib*.jpg
'''

# Python 2/3 compatibility
from __future__ import print_function

import json
import numpy as np
import cv2 as cv

# local modules
from common import splitfn

# built-in modules
import os

def save_camera_params(params):
    if not os.path.exists('json_files'):
        os.mkdir('json_files')
    try:
        with open('json_files/camera_params.json', 'w+') as outfile:  
            json.dump(params, outfile)
        print('Saved filters to \'json_files/camera_params.json\'!')
    except Exception as e:
        print('Couldn\'t save parameters due to: ')
        print(e)
        print('but here are them printed out:')
        print(params)

def main():
    import sys
    import getopt
    from glob import glob

    args, img_mask = getopt.getopt(sys.argv[1:], '', ['debug=', 'square_size=', 'threads='])
    args = dict(args)
    args.setdefault('--debug', './output/')
    args.setdefault('--square_size', 1.0)
    args.setdefault('--threads', 4)
    if not img_mask:
        img_mask = 'calib_images/img??.png'  # default
    else:
        img_mask = img_mask[0]

    img_names = glob(img_mask)
    debug_dir = args.get('--debug')
    if debug_dir and not os.path.isdir(debug_dir):
        os.mkdir(debug_dir)
    square_size = float(args.get('--square_size'))

    pattern_size = (7, 7)
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
    h, w = cv.imread(img_names[0], cv.IMREAD_GRAYSCALE).shape[:2]  # TODO: use imquery call to retrieve results

    def processImage(fn):
        print('processing %s... ' % fn)
        img = cv.imread(fn, 0)
        if img is None:
            print("Failed to load", fn)
            return None

        assert w == img.shape[1] and h == img.shape[0], ("size: %d x %d ... " % (img.shape[1], img.shape[0]))
        found, corners = cv.findChessboardCorners(img, pattern_size)
        if found:
            term = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT, 30, 0.1)
            cv.cornerSubPix(img, corners, (5, 5), (-1, -1), term)

        if not found:
            print('chessboard not found')
            return None

        print('           %s... OK' % fn)
        return (corners.reshape(-1, 2), pattern_points)

    threads_num = int(args.get('--threads'))
    if threads_num <= 1:
        chessboards = [processImage(fn) for fn in img_names]
    else:
        print("Run with %d threads..." % threads_num)
        from multiprocessing.dummy import Pool as ThreadPool
        pool = ThreadPool(threads_num)
        chessboards = pool.map(processImage, img_names)

    chessboards = [x for x in chessboards if x is not None]
    for (corners, pattern_points) in chessboards:
        img_points.append(corners)
        obj_points.append(pattern_points)

    # calculate camera distortion
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, (w, h), None, None)

    params = dict()
    params['rms'] = rms
    params['camera_matrix'] = np.ndarray.tolist(camera_matrix)
    params['dist_coefs'] = np.ndarray.tolist(dist_coefs.ravel())
    save_camera_params(params)

    # undistort the image with the calibration
    print('')
    # for fn in img_names if debug_dir else []:
    #     path, name, ext = splitfn(fn)
    #     img_found = os.path.join(debug_dir, name + '_chess.png')
    #     outfile = os.path.join(debug_dir, name + '_undistorted.png')

    #     img = cv.imread(img_found)
    #     if img is None:
    #         continue

    #     h, w = img.shape[:2]
    #     newcameramtx, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (w, h), 1, (w, h))

    #     dst = cv.undistort(img, camera_matrix, dist_coefs, None, newcameramtx)

    #     # crop and save the image
    #     x, y, w, h = roi
    #     dst = dst[y:y+h, x:x+w]

    #     print('Undistorted image written to: %s' % outfile)
    #     cv.imwrite(outfile, dst)

    # print('Done')


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
