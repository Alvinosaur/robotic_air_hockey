import cv2 as cv
import json
import numpy as np
import os


def save_thresholds(thresholds):
    if not os.path.exists('json_files'):
        os.mkdir('json_files')
    try:
        with open('json_files/object_hsv_filters2.json', 'w+') as outfile:  
            json.dump(thresholds, outfile)
        print('Saved filters to \'json_files/object_hsv_filters.json\'!')
    except Exception as e:
        print('Couldn\'t save parameters due to: ')
        print(e)
        print('but here are them printed out:')
        print(thresholds)

def nothing(temp):
    return


def run_calibration_gui():
    """
    Purpose is to have vision tracking update color thresholds to match
    a player's puck color and lighting conditions. 

    Once finish calibrating, store settings in a json file.

    """
    H_SCALE = 0.5
    cam_index = 1 # Default camera is at index 0.
    cap = cv.VideoCapture(cam_index) # Video capture object

    # Ordered from min to max HSV
    param_names = ['H_MIN', 'S_MIN', 'V_MIN', 'H_MAX', 'S_MAX', 'V_MAX']
    # maps threshold param from min to max
    param_bounds = {
        'H_MIN': (0, 359),  # (min, max)
        'H_MAX': (0, 359),
        'S_MIN': (0, 255),
        'S_MAX': (0, 255),
        'V_MIN': (0, 255),
        'V_MAX': (0, 255)
    }
    tracked_object_names = ['Puck Filter', 'First Joint Filter', 
                        'Second Joint Filter']

    # for each desired object color to be detected
    for obj_name in tracked_object_names:
        # create window to hold sliders
        cv.namedWindow(obj_name)
        
        # create slider for min/max thresholds of HSV
        for param in param_names:
            cv.createTrackbar(param, obj_name, param_bounds[param][0], 
                                param_bounds[param][1], nothing)

    # Show HSV guide
    hsv_guide_img = cv.imread('hsv_guide.jpg')
    cv.imshow('HSV Guide (NOTE: S, V scaled by 255)', hsv_guide_img)

    # Enable the camera
    cap.open(cam_index) 
    while True:
        # Handle user closing gui
        if cv.waitKey(1) & 0xFF == ord("q"):
            save_thresholds(thresholds)
            cv.destroyAllWindows()
            cap.release()
            break

        _, frame = cap.read()

        # record all the current values chosen by user
        thresholds = dict()
        for obj_name in tracked_object_names:
            thresholds[obj_name] = dict()
            temp = []

            for param_name in param_names:
                value = cv.getTrackbarPos(param_name, obj_name)
                # follow hsv guide, but scale to match opencv's scale
                if 'H' in param_name: temp.append(value * H_SCALE)
                else: temp.append(value)
            
            # 3 x 1 vec of min/max HSV
            thresholds[obj_name]['lower'] = temp[0:3]  # json-serializable list
            thresholds[obj_name]['upper'] = temp[3:6]
            lowerBound = np.array(thresholds[obj_name]['lower'], int)
            upperBound = np.array(thresholds[obj_name]['upper'], int)

            # extract HSV from camera frame
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            image_filter = cv.inRange(hsv, lowerBound, upperBound)

            # apply user-chosen thresholds to filter image
            new_image = cv.bitwise_and(frame, frame, mask = image_filter)
            cv.imshow(obj_name, new_image)


if __name__ == '__main__':
    run_calibration_gui()