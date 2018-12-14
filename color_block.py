import cv2
import numpy as np
import math
import time
import atexit
from pid import *
from geometry import *
from equalization import *

ON_RPI = 1
CAMERA_NO = 1
IMG_WIDTH = 640
IMG_HEIGHT = 480
NORMAL_SPEED = 3
THRUST_SPEED = 7
cam = cv2.VideoCapture(CAMERA_NO)
controller = PID_controller([1000, 16, 200])
ctrl = 0

if ON_RPI:
    from ARSCHLOCH import *
    arschloch = ARSCHLOCH()
    arschloch.turn_on()
    arschloch.set_fan(170)
    arschloch.callibrate_ESC()
    atexit.register(arschloch.turn_off)
    arschloch.accelerate(NORMAL_SPEED)

def evaluate_function(angle_part, translate_part, x, y):
    x = abs(x)
    return math.exp(-0*y/IMG_HEIGHT/20.)*(
        (1 - 1.*x/IMG_WIDTH/2.0)*angle_part + 
         1.*x/IMG_WIDTH/2.0 * translate_part)


while True:
    # No. of vertical slices
    resolution = 16

    # Read image from USB camera
    _, img = cam.read()

    # Resize image to lower resolution to ease the load of computation
    img = cv2.resize(img,(IMG_WIDTH,IMG_HEIGHT))

    # Select region of interest, which is the lower part of the image
    img = img[int(0.7*IMG_HEIGHT):IMG_HEIGHT, int(0 * IMG_WIDTH):int(1 * IMG_WIDTH)]

    # Extract positions of center of shape of black region in each slice to get a polygon
    # approximation of the line
    path, poly = extract_polygon(img, resolution)

    if not len(path) == 0:
        # Check if convex hull area formed by curve is too large
        # Too large convex hull area means the result is not corrected and should be discarded
        n = len(path)
        hull = cv2.convexHull(np.array(path))
        area = cv2.contourArea(hull)
        AREA = IMG_HEIGHT * IMG_WIDTH

        print(area/1./AREA)
        if AREA / 50.0 < area:
            if __debug__:
                print("Noise Detected ! ! !")
                cv2.drawContours(img,[hull],0,(0,0,255),-1)            
        else:
            # Calculate center of mass of detected polygon
            curve_cm = [0, 0]
            for i in range(0, len(path)):
                curve_cm[0] += 1.*path[i][0]/len(path)
                curve_cm[1] += 1.*path[i][1]/len(path)

            # Calculate angle of tangent line by calculating tengent vector of polygon
            vec_sec = [path[0][0]-path[n-1][0], path[0][1]-path[n-1][1]]
            ang_sec = 90 - np.angle(vec_sec[0] - vec_sec[1] * 1J, deg=True)

            # Calculate normalized x position and tangent angle
            translate_part = 90 - np.interp(curve_cm[0], [0, IMG_WIDTH], [180, 0])
            angle_part = ang_sec

            # Evaluate turning angle of servo motor with previous results
            ctrl_estimate = evaluate_function(angle_part, translate_part, \
                            curve_cm[0] - IMG_WIDTH/2.0, IMG_HEIGHT - curve_cm[1])

            # Push information to PID controller to get information
            controller.step(ctrl_estimate)
            ctrl = controller.get_ctrl()

            if __debug__:
                print("Translate Part : " + str(translate_part))
                print("Angle Part : " + str(angle_part))
                print("ctrl : " + str(ctrl))
                #cv2.drawContours(img,[hull],0,(255,0,0),-1)

        if __debug__:
            for i in range(len(path) - 1):
                cv2.line(img, path[i] ,path[i + 1],(0, 255, 0),5)
            for i in range(len(poly) - 1):
                cv2.drawContours(img,poly[i],0,(255,0,0),1)            
            cv2.imshow("cam",img)
            cv2.waitKey(10)

        if ON_RPI:
            arschloch.steer(90 - ctrl)

