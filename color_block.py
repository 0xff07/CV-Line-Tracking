import cv2
import numpy as np
import math
import time
import atexit
from pid import *
from geometry import *
from equalization import *

ON_RPI = 1
CAMERA_NO = 0
IMG_WIDTH = 320
IMG_HEIGHT = 240
NORMAL_SPEED = 2
THRUST_SPEED = 7
cam = cv2.VideoCapture(CAMERA_NO)
controller = PID_controller([1000, 16, 45])
ctrl_last = 0
ctrl = 0
no_flip = 0 
dist = 100000

if ON_RPI:
    from ARSCHLOCH import *
    arschloch = ARSCHLOCH()
    arschloch.turn_on()
    arschloch.callibrate_ESC()
    atexit.register(arschloch.turn_off)
    arschloch.accelerate(0)
    time.sleep(0.7)
    arschloch.accelerate(0)

def evaluate_function(angle_part, translate_part, x, y):
    x = abs(x)
    return math.exp(-1*y/IMG_HEIGHT/20.)*(
        (1 - 1.*x/IMG_WIDTH/2.0)*angle_part + 
         1.*x/IMG_WIDTH * translate_part)


while True:
    resolution = 16
    dist = arschloch.get_distance()
    _, img = cam.read()
    img = cv2.resize(img,(IMG_WIDTH,IMG_HEIGHT))
    img = img[int(0*IMG_HEIGHT):IMG_HEIGHT, int(0 * IMG_WIDTH):int(1 * IMG_WIDTH)]
    img = CLACHE_equalization(img)
    path, poly = extract_polygon(img, resolution)

    if not len(path) == 0:
        n = len(path)
        hull = cv2.convexHull(np.array(path))
        area = cv2.contourArea(hull)
        AREA = IMG_HEIGHT * IMG_WIDTH

        if AREA / 80.0 < area or area < 1:
            ctrl = ctrl_last
            if __debug__:
                print("Noise Detected ! ! !")
                cv2.drawContours(img,[hull],0,(0,0,255),-1)            
        else:
            curve_cm = [0, 0]
            for i in range(0, len(path)):
                curve_cm[0] += 1.*path[i][0]/len(path)
                curve_cm[1] += 1.*path[i][1]/len(path)

            vec_sec = [path[0][0]-path[n-1][0], path[0][1]-path[n-1][1]]
            ang_sec = 90 - np.angle(vec_sec[0] - vec_sec[1] * 1J, deg=True)

            translate_part = 90 - np.interp(curve_cm[0], [0, IMG_WIDTH], [135, 45])
            angle_part = ang_sec

            ctrl_estimate = evaluate_function(angle_part, translate_part, \
                            curve_cm[0] - IMG_WIDTH/2.0, IMG_HEIGHT - curve_cm[1])

            controller.step(ctrl_estimate)
            ctrl = controller.get_ctrl()

            if abs(ctrl -  ctrl_last) < 2:
                no_flip  += 1
            else:
                no_flip = 0
            accelerate = no_flip / 30

            ctrl_last = ctrl

            if __debug__:
                print("Translate Part : " + str(translate_part))
                print("Angle Part : " + str(angle_part))
                print("flip no : " + str(no_flip))
                print("ctrl : " + str(ctrl))
                print("dist : " + str(dist))

        if __debug__:
            for i in range(len(path) - 1):
                cv2.line(img, path[i] ,path[i + 1],(0, 255, 0),5)
            for i in range(len(poly) - 1):
                cv2.drawContours(img,poly[i],0,(255,0,0),1)            
            cv2.drawContours(img,[hull],0,(255,0,0),-1)
            cv2.imshow("cam",img)
            cv2.waitKey(10)

        if ON_RPI:
            arschloch.steer(90 - ctrl)

