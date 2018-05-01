import cv2
import numpy as np
import math
import Queue
import time

ON_RPI = None

CAMERA_NO = 0
IMG_WIDTH = 360
IMG_HEIGHT = 270
SERVO_MID = 7
SERVO_OFFSET = 3
SERVO_PIN = 12

if ON_RPI:
    from pwm import *

HSV_LB = np.array([0,0,0])
HSV_UB = np.array([180,255,75])

class PID_controller():
    def __init__(self, KPID, QUEUE_SIZE = 2000):
        self.PID = list([0., 0., 0.])
        self.KPID = list([1.*KPID[0], 1.*KPID[1]/QUEUE_SIZE, 1.*KPID[2]])
        self.integral_ghb = Queue.Queue(QUEUE_SIZE)
        self.last_tmp = 0
        self.ctrl = 0
        self.queue_size = QUEUE_SIZE

    def step(self, cur_data):
        self.PID[0] = cur_data
        self.PID[2] = cur_data - self.last_tmp
        self.last_tmp = cur_data
        if not self.integral_ghb.full():
            self.integral_ghb.put(cur_data)
            self.PID[1] += cur_data
        else:
            self.PID[1] = self.PID[1] - self.integral_ghb.get() + cur_data
            self.integral_ghb.put(cur_data)
        self.ctrl =  sum(self.PID[i] * self.KPID[i] for i in range(3))/sum(self.KPID)

    def get_KPID(self):
        return [i for i in KPID]

    def get_PID(self):
        return [i for i in PID]

    def set_KPID(self,new_kpid):
        self.KPID[0] = new_kpid[0]
        self.KPID[1] = new_kpid[1] / float(QUEUE_SIZE)
        self.KPID[2] = new_kpid[2]
        
    def get_ctrl(self):
        return self.ctrl

    def dump(self):
        print "PID : " + str(self.PID)
        print "CONTROL : " + str(self.ctrl)

def extract_polygon(img, slice_num=16, LB=np.array([0,0,0]), UB=np.array([180,255,75])):
    IMG_HEIGHT, IMG_WIDTH,_ = img.shape
    X_DIV = int(IMG_HEIGHT/float(slice_num))
    kernelOpen = np.ones((5,5))
    kernelClose = np.ones((20,20))

    poly_points = [None] * slice_num
    detected_contours = [None] * slice_num
    
    for i in range(0, slice_num) :
        sliced_img = img[X_DIV*i + 1:X_DIV*(i+1), int(0):int(IMG_WIDTH)]
        imgHSV = cv2.cvtColor(sliced_img,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(imgHSV, LB, UB)
        maskOpen = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
        maskFinal = maskClose
        _,conts,_ = cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        if(conts):
            c = max(conts, key = cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            if w < IMG_WIDTH / 4.:
                seg_size = (w, h)
                poly_points[i] = (x + w/2, y + h/2 + X_DIV * i)
                detected_contours[i] = c

            if not ON_RPI:
                cv2.drawContours(sliced_img, c,-1,(255,0,0),3)

        valid_contours = [i for i in detected_contours if i is not None]
        valid_points = [i for i in poly_points if i is not None]

    return valid_points

def evaluate_function(angle_part, translate_part, x, y):
    x = abs(x)
    return math.exp(-1*y/IMG_HEIGHT/20.)*(
        (1 - 1.*x/IMG_WIDTH/2.0)*angle_part + 
         1.*x/IMG_WIDTH * translate_part)


cam = cv2.VideoCapture(CAMERA_NO)
controller = PID_controller([140, 10, 50])
pwm = []

if ON_RPI:
    pwm = PWM_init({"SERVO":12})

while True:
    try:
        resolution = 16
        _, img = cam.read()
        img = cv2.resize(img,(IMG_WIDTH,IMG_HEIGHT))
        path = extract_polygon(img, resolution)

        if not len(path) == 0:
            n = len(path)

            curve_cm = [0, 0]
            for i in range(0, len(path)):
                curve_cm[0] += 1.*path[i][0]/len(path)
                curve_cm[1] += 1.*path[i][1]/len(path)
            
            vec_sec = [path[0][0]-path[n-1][0], path[0][1]-path[n-1][1]]
            ang_sec = 90 - np.angle(vec_sec[0] - vec_sec[1] * 1J, deg=True)

            translate_part = 90 - np.interp(curve_cm[0], [0, IMG_WIDTH], [180, 0])
            angle_part = ang_sec

            ctrl_estimate = evaluate_function(angle_part, translate_part, curve_cm[0] - IMG_WIDTH/2.0, IMG_HEIGHT - curve_cm[1])
            controller.step(ctrl_estimate)
            servo_duty = np.interp(ctrl_estimate, [-90, 90], [12, 2.7])

            if ON_RPI:
                pwm["SERVO"].ChangeDutyCycle(servo_duty)

            if not ON_RPI:
                for i in range(len(path) - 1):
                    cv2.line(img, path[i] ,path[i + 1],(0, 255, 0),5)
                cv2.imshow("cam",img)
                cv2.waitKey(10)

            if __debug__:
                print "curve CM : " + str(curve_cm)
                print "secant vec" + str(vec_sec)
                print "Translate Part : " + str(translate_part)
                print "Angle Part : " + str(angle_part)
                print "ctrl estimate: " + str(ctrl_estimate)
                print "servo duty: " + str(servo_duty)
                controller.dump()
    except KeyboardInterrupt:
        break
        if ON_RPI:
            PWM_end_routine(pwm)
