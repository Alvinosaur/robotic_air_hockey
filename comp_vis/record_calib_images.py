import numpy as np
import cv2
import os

DEFAULT_CAM_INDEX = 1
cap = cv2.VideoCapture(DEFAULT_CAM_INDEX)
pattern_size = (7, 7)

images_left = 20  # stop when this reaches 0
dir_path = "calib_images"
if not os.path.exists(dir_path): 
    os.makedirs(dir_path)
general_img_name = "img"
print("Press 's' to save image if it is good for calibation. Need at least 20 images")

while(images_left > 0):
    img_name = general_img_name + str(images_left) + '.png'
    full_path = os.path.join(dir_path, img_name)
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow(img_name, gray)

    # Display the resulting frame

    if cv2.waitKey(1) & 0XFF == ord('s'):
        found, corners = cv2.findChessboardCorners(gray, pattern_size)
        if found:
            images_left -= 1
            print('Success! %d left to take.' % images_left)
            cv2.imwrite(full_path, gray)
            cv2.destroyAllWindows()
        else:
            print('Try Again.')

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()