import cv2
import numpy

CAM_INDEX = 0; #default camera index is 0
RGBMIN = 0;
RGBMAX = 255;
SLIDERS = ['R_MAX', 'G_MAX', 'B_MAX', 'R_MIN', 'G_MIN', 'B_MIN'];

def voidFunc(arg1):
    return;

'''
    track: tracks objects defined by argument obj_array
        obj_array: array of objects to track of form
            [obj1, obj2, obj3...] where each object is of form:
        obj: [name: String, r_max_default: int, g_max_default: int,
            b_max_default: int, r_min_default: int
            , g_min_default: int, b_min_default: int,
            shape: "RECTANGLE" or "CIRCLE" with default as "RECTANGLE",
            color: (b, g, r) where b,g,r are between 0, 255]
'''
def track(obj_array):
    name, r_maxd, g_maxd, b_maxd, r_mind, g_mind, b_mind, shape, colors = [0, 1, 2, 3, 4, 5, 6, 7, 8];

    for object in obj_array:
        #cv2.setTrackbarPos(SLIDERS[r_maxd - 1], object[name], object[r_maxd]);
        #cv2.setTrackbarPos(SLIDERS[r_mind - 1], object[name], object[r_mind]);
        #cv2.setTrackbarPos(SLIDERS[g_maxd - 1], object[name], object[g_maxd]);
        #cv2.setTrackbarPos(SLIDERS[g_mind - 1], object[name], object[g_mind]);
        #cv2.setTrackbarPos(SLIDERS[b_maxd - 1], object[name], object[b_maxd]);
        #cv2.setTrackbarPos(SLIDERS[b_mind - 1], object[name], object[b_mind]);

        #TESTING FOR DEFAULT color
        cv2.namedWindow(object[name]);
        for slider in SLIDERS:
            cv2.createTrackbar(slider, object[name], RGBMIN, RGBMAX, voidFunc);

    cap = cv2.VideoCapture(CAM_INDEX);
    cap.open(CAM_INDEX) #Open the camera
    while True:
        #Close camera and corresponding windows
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            cap.release()
            return
        ret, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for object in obj_array:
            thresholds = []
            for slider in SLIDERS:
                thresholds.append(cv2.getTrackbarPos(slider, object[name]))
            upperBound = numpy.array(thresholds[0:3])
            lowerBound = numpy.array(thresholds[3:6])

            threshold = cv2.inRange(hsv, lowerBound, upperBound)
            res = cv2.bitwise_and(frame, frame, mask = threshold)
            im2, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
            center = ((x+w)//2, (y+h)//2)
            if(object[shape] == "CIRCLE"):
                radius = radius()
                cv2.circle(frame, circle, radius, object[colors], 2);
            else:
                cv2.rectangle(frame,(x,y),(x+w,y+h), object[colors], 2)
            print(object[name] + ":" + str(center))
        cv2.imshow("Objects", frame)

'''
    track_puck: tracks the puck
        arg cam: open opencv Video feed object
            --cap = cv2.VideoCapture(CAM_INDEX);
            --cap.open(CAM_INDEX)
        arg col_array: array of colors defining the puck's concentric circles
        arg averaging_func: function to take the locations picked up by the tracker
            and average out to get one single accurate location of the puck
            --takes in an array: [(x,y,r)] in order of col_array,
                and adds at least one (x,y,r) from circles detected
'''
def track_puck(cam, col_array, averaging_func):
    name, r_maxd, g_maxd, b_maxd, r_mind, g_mind, b_mind, shape, colors = [0, 1, 2, 3, 4, 5, 6, 7, 8];

    if cv2.waitKey(1) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        cap.release()
        return
    ret, frame = cam.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cv2.imshow("HSV", hsv)

    location_array = [] #array of (x,y,r) to average out

    #Color Detection with Largest minEnclosingCircle
    for color in col_array:
        thresholds = []
        for bound in [1,2,3,4,5,6]:
            thresholds.append(color[bound])
        upperBound = numpy.array(thresholds[0:3])
        lowerBound = numpy.array(thresholds[3:6])

        threshold = cv2.inRange(hsv, lowerBound, upperBound)
        res = cv2.bitwise_and(frame, frame, mask = threshold)
        im2, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
        #cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        centerOfPuck = (int((x+w)//2), int((y+h)//2))
        location_array.append((centerOfPuck[0], centerOfPuck[1], 0))

        biggestRadius = 0;
        biggestCircleDims = (0,0,0) #(x,y,r)
        for contour in contours:
            (x,y), r = cv2.minEnclosingCircle(contour);
            center = (int(x), int(y));
            radius = int(r);
            if(radius > biggestRadius):
                biggestRadius = radius;
                biggestCircleDims = (center[0],center[1],radius);
        if(biggestRadius == 0):
            print("ERROR: biggest radius is 0")
            return;
        x = biggestCircleDims[0];
        y = biggestCircleDims[1];
        r = biggestRadius;
        center = (x,y)
        if(color[shape] == "CIRCLE"):
            cv2.circle(frame, center, r, color[colors], 2);
        #else:
            #cv2.rectangle(frame,(x-r,y-r),(x+r,y+r), color[colors], 2)
        location_array.append((x,y,r))

    #Hough Circle Detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
    gray = cv2.GaussianBlur(gray, (5,5), 0);
    rows = gray.shape[0];
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows/8,
        param1=200, param2=70, minRadius=0, maxRadius=0);
    if circles is not None:
        circles = numpy.uint16(numpy.around(circles));
        for i in circles[0,:]:
            center = (int(i[0]), int(i[1]));
            radius = int(i[2]);
            #cv2.circle(frame, center, 1, (0,100,100), 4);
            #cv2.circle(frame, center, radius, (255,0,255), 2); #pink circles
            location_array.append((center[0], center[1], radius));

    x_avr = 0;
    y_avr = 0;
    r_avr = 0;
    tot = 0;
    r_tot = 0;
    for loc in location_array:
        x_avr = x_avr + loc[0];
        y_avr = y_avr + loc[1];
        r_avr = r_avr + loc[2];
        tot = tot + 1;
        if(loc[2] != 0):
            r_tot = r_tot + 1;
    x_avr = int(x_avr/tot);
    y_avr = int(y_avr/tot);
    r_avr = int(r_avr/r_tot);
    cv2.circle(frame, (x_avr, y_avr), 1, (100, 100, 0), 4);
    cv2.circle(frame, (x_avr, y_avr), r_avr, (100, 100, 0), 2);
    print(str(location_array));
    cv2.imshow("Objects", frame)
puck_obj_blue = ["PUCK", 255, 255, 255, 178, 0, 0, "CIRCLE", (255, 0, 0)] #blue tracker for blue circle
#puck_obj_green = ["PUCK2", 255, 255, 255, 0, 178, 0, "CIRCLE", (0, 255, 0)] #green tracker for green circle
obj_array = [puck_obj_blue]
#track(obj_array)
cap = cv2.VideoCapture(CAM_INDEX);
cap.open(CAM_INDEX) #Open the camera
while True:
    track_puck(cap, obj_array, None)
