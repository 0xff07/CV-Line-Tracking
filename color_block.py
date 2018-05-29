import cv2
import numpy as np
import math
import Queue
import time
import os
from pid import *
from geometry import *

ON_RPI = 0

CAMERA_NO = 0
IMG_WIDTH = 320
IMG_HEIGHT = 240
ERVO_PIN = 12

if ON_RPI:
    from pwm import *

HSV_LB = np.array([0,0,0])
HSV_UB = np.array([1,255,1])

def evaluate_function(angle_part, translate_part, x, y):
    x = abs(x)
    return math.exp(-1*y/IMG_HEIGHT/20.)*(
        (1 - 1.*x/IMG_WIDTH/2.0)*angle_part + 
         1.*x/IMG_WIDTH * translate_part)
    #return angle_part

def servo_test(pwm, SERVO_MID, SERVO_OFFSET):
    while True:
        pwm["SERVO"].ChangeDutyCycle(SERVO_MID)
        time.sleep(1)
        pwm["SERVO"].ChangeDutyCycle(SERVO_MID + SERVO_OFFSET)
        time.sleep(1)
        pwm["SERVO"].ChangeDutyCycle(SERVO_MID)
        time.sleep(1)
        pwm["SERVO"].ChangeDutyCycle(SERVO_MID - SERVO_OFFSET)
        time.sleep(1)
 

cam = cv2.VideoCapture(CAMERA_NO)
controller = PID_controller([1000, 16, 0])
if ON_RPI:
    pwm = PWM_init({"SERVO":12})
    os.system("python arduino_start.py")

# 11.75 ~ 2
SERVO_MID = 7.5
SERVO_OFFSET = 4

ctrl_last = SERVO_MID
ctrl = SERVO_MID
while True:
    try:
        resolution = 32
        _, img = cam.read()
        img = cv2.resize(img,(IMG_WIDTH,IMG_HEIGHT))
        img = img[int(0.45*IMG_HEIGHT):IMG_HEIGHT, int(0.1 * IMG_WIDTH):int(0.9 * IMG_WIDTH)]
        path, poly = extract_polygon(img, resolution)
        if not len(path) == 0:
            n = len(path)
            hull = cv2.convexHull(np.array(path))
            area = cv2.contourArea(hull)
            #print "Hull Area : " + str(area)

            if not ON_RPI:
                for i in range(len(path) - 1):
                    cv2.line(img, path[i] ,path[i + 1],(0, 255, 0),5)
                for i in range(len(poly) - 1):
                    cv2.drawContours(img,poly[i],0,(255,0,0),1)            
                cv2.drawContours(img,[hull],0,(255,0,0),-1)            
                cv2.imshow("cam",img)
                cv2.waitKey(10)

            if IMG_HEIGHT * IMG_HEIGHT / 60.0 < area:
                ctrl = ctrl_last
                print "Noise Detected ! ! !"
            else:
                curve_cm = [0, 0]
                for i in range(0, len(path)):
                    curve_cm[0] += 1.*path[i][0]/len(path)
                    curve_cm[1] += 1.*path[i][1]/len(path)
            
                vec_sec = [path[0][0]-path[n-1][0], path[0][1]-path[n-1][1]]
                ang_sec = 90 - np.angle(vec_sec[0] - vec_sec[1] * 1J, deg=True)

                translate_part = 90 - np.interp(curve_cm[0], [0, IMG_WIDTH], [180, 0])
                angle_part = ang_sec

                ctrl_estimate = evaluate_function(angle_part, translate_part, \
                                curve_cm[0] - IMG_WIDTH/2.0, IMG_HEIGHT - curve_cm[1])
                controller.step(ctrl_estimate)
                ctrl = controller.get_ctrl()
                ctrl_last = ctrl

                if __debug__:
                    print "curve CM : " + str(curve_cm)
                    print "secant vec" + str(vec_sec)
                    print "Translate Part : " + str(translate_part)
                    print "Angle Part : " + str(angle_part)
                    print "ctrl estimate: " + str(ctrl_estimate)
                    controller.dump()


            servo_duty = np.interp(ctrl, [-90, 90], [SERVO_MID - SERVO_OFFSET, SERVO_MID + SERVO_OFFSET])
            
            print "SERVO_DUTY : " + str(servo_duty)

            if ON_RPI:
                pwm["SERVO"].ChangeDutyCycle(servo_duty)

    except KeyboardInterrupt:
        if ON_RPI:
            pwm["SERVO"].ChangeDutyCycle(SERVO_MID)
            os.system("python arduino_end.py")
            time.sleep(2)
            PWM_end_routine(pwm)
        break
