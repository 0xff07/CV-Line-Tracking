import cv2
import numpy as np
import math
import Queue
import time
import RPi.GPIO as GPIO

CAMERA_NO = 0
SLICE_NUM = 2
IMG_WIDTH = 360
IMG_HEIGHT = 270
QUEUE_SIZE = 200
X_DIV = int(IMG_HEIGHT/float(SLICE_NUM))
LOOP_DELAY = 0.01
PWM_MIN = 2
PWM_MAX = 11.5
SERVO_PIN = 12

HSV_LB = np.array([0,0,0])
HSV_UB = np.array([180,255,75])

CM_x = range(0, SLICE_NUM)
CM_y = range(0, SLICE_NUM)
CM_w = range(0, SLICE_NUM)
CM_h = range(0, SLICE_NUM)
angles = range(0, SLICE_NUM - 1)

sliced_img = range(0, SLICE_NUM)

PID = [0, 0, 0]
KPID = [0.1, 0./QUEUE_SIZE , 0] # Kp, Ki, Kd
GHB = Queue.Queue(QUEUE_SIZE);
last = 0;


def black_contours(img, LB = HSV_LB, UB = HSV_UB):
    kernelOpen = np.ones((5,5))
    kernelClose = np.ones((20,20))

    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(imgHSV, LB, UB)
    maskOpen = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernelOpen
    )
    maskClose=cv2.morphologyEx(
        maskOpen,
        cv2.MORPH_CLOSE,
        kernelClose
    )
    maskFinal = maskClose

    _, conts, _ = cv2.findContours(
        maskFinal.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE)
    return conts


def contour_pos(contour):
    x, y, w, h = cv2.boundingRect(c)
    CM_x = x + w/2
    CM_y = y + h/2 + X_DIV * i
    CM_w = w
    CM_h = h
    return CM_x, CM_y, CM_w, CM_h

def pwm_end_routine():
    servo_pwm.ChangeDutyCycle(6.5)
    PWM_PIN.stop()
    GPIO.cleanup()

cam = cv2.VideoCapture(CAMERA_NO)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(6.5)
servo_pwm.ChangeDutyCycle(6.5)
time.sleep(1)

while True:
    try:
        _, img = cam.read()
        img = cv2.resize(
            img,
            (IMG_WIDTH,IMG_HEIGHT)
        )

        for i in range(0,SLICE_NUM) :
            sliced_img[i] = img[X_DIV*i + 1:X_DIV*(i+1), 
                            int(0):int(IMG_WIDTH)]

            conts = black_contours(sliced_img[i])

            if(conts):
                c = max(conts, key = cv2.contourArea)
                CM_x[i], CM_y[i], CM_w[i], CM_h[i] = contour_pos(c)

                if __debug__:
                    cv2.drawContours(sliced_img[i],conts,-1,(255,0,0),3)
                    cv2.rectangle(
                        sliced_img[i],
                        (CM_x[i] - CM_w[i]/2, CM_y[i] - CM_h[i]/2 - X_DIV*i),
                        (CM_x[i] + CM_w[i]/2, CM_y[i] + CM_h[i]/2 - X_DIV*i),
                        #(x+w,y+h),
                        (0,0,255),
                        2
                    )
                    if i > 0:
                        cv2.line(
                            img,
                            (CM_x[i],CM_y[i]),
                            (CM_x[i-1], CM_y[i-1]),
                            (0, 255, 0),
                            5
                        )

        # Calculate tangent angle
        for i in range(0, SLICE_NUM - 1):
            pos = [CM_x[i]/2 + CM_x[i + 1]/2 - IMG_WIDTH/2, 
                   CM_y[i]/2 + CM_y[i + 1]/2 - IMG_HEIGHT/2]
            vec_t = [CM_x[i] - CM_x[i + 1], CM_y[i] - CM_y[i + 1]]
            rad = math.acos(vec_t[0]/math.sqrt(vec_t[0]**2 + vec_t[1]**2))
            angles[i] = rad * 180 / math.pi

        # Calculate PID values
        PID[0] = pos[0]
        PID[2] = pos[0] - last
        last = pos[0]

        if not GHB.full():
            GHB.put(pos[0])
            PID[1] += pos[0]
        else:
            PID[1] = PID[1] - GHB.get() + pos[0]
            GHB.put(pos[0])


        ctrl_val = int(np.dot(
                [PID[0], PID[1], PID[2]], 
                KPID)/sum(KPID)
        )
        ctrl_val = np.interp(ctrl_val, [-120, 150], [4.6, 8.6])
        ctrl_val = np.interp(angles[0], [180, 0], [4.6, 8.6])
        
        if __debug__:
            cv2.imshow("cam",img)
            cv2.waitKey(10)
        print "ANGLE : " + str(angles[0])
        print "PID :" + str(PID)
        print "CONTROL :" + str(ctrl_val)
        servo_pwm.ChangeDutyCycle(ctrl_val)
        time.sleep(0.0001)
    except KeyboardInterrupt:
        servo_pwm.ChangeDutyCycle(6.5)
        servo_pwm.stop()
        GPIO.cleanup()
        if __debug__:
            cv2.destroyAllWindows()
        break
