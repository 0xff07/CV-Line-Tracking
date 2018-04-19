import cv2
import numpy as np
import math
import Queue
import time

SLICE_NUM = 2
IMG_WIDTH = 360
IMG_HEIGHT = 270
QUEUE_SIZE = 200
X_DIV = int(IMG_HEIGHT/float(SLICE_NUM))
LOOP_DELAY = 0.01

HSV_LB = np.array([0,0,0])
HSV_UB = np.array([180,255,100])

cam = cv2.VideoCapture(0)
kernelOpen = np.ones((5,5))
kernelClose = np.ones((20,20))

CM_x = range(0, SLICE_NUM)
CM_y = range(0, SLICE_NUM)
CM_w = range(0, SLICE_NUM)
CM_h = range(0, SLICE_NUM)

sliced_img = range(0, SLICE_NUM)

PID = [0, 0, 0]
KPID = [1, 1/200 , 1] # Kp, Ki, Kd
GHB = Queue.Queue(QUEUE_SIZE);
last = 0;

while True:

    ret, img = cam.read()
    img = cv2.resize(
        img,
        (IMG_WIDTH,IMG_HEIGHT)
    )

    for i in range(0,SLICE_NUM) :
        sliced_img[i] = img[X_DIV*i + 1:X_DIV*(i+1), 
                            int(0):int(IMG_WIDTH)]

        # convert BGR to HSV
        imgHSV = cv2.cvtColor(sliced_img[i],cv2.COLOR_BGR2HSV)

        # create the Mask
        mask = cv2.inRange(imgHSV,HSV_LB,HSV_UB)

        # morphology
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

        maskFinal=maskClose

        _, conts, h = cv2.findContours(
            maskFinal.copy(),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE)

        if __debug__:
            cv2.drawContours(sliced_img[i],conts,-1,(255,0,0),3)

        if(conts):
            c = max(conts, key = cv2.contourArea)
            x,y,w,h=cv2.boundingRect(c)
            CM_x[i] = x + w/2
            CM_y[i] = y + h/2 + X_DIV * i
            CM_w[i] = w
            CM_h[i] = h

            if __debug__:
                cv2.rectangle(
                    sliced_img[i],
                    (x,y),
                    (x+w,y+h),
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

    if __debug__:
        cv2.imshow("cam",img)
        cv2.waitKey(10)

    # Calculate tangent angle
    pos = [CM_x[0]/2 + CM_x[1]/2 - IMG_WIDTH/2, 
           CM_y[0]/2 + CM_y[1]/2 - IMG_HEIGHT/2]
    vec_t = [CM_x[0] - CM_x[1], CM_y[0] - CM_y[1]]
    angle = math.acos(vec_t[0]/math.sqrt(vec_t[0]**2 + vec_t[1]**2))
    angle = angle * 180 / math.pi

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
            [PID[0], PID[1]/QUEUE_SIZE, PID[2]], 
            KPID)/sum(KPID)/2
    )

    print "ANGLE : " + str(angle)
    print "PID :" + str(PID)
    print "CONTROL :" + str(ctrl_val)
    time.sleep(LOOP_DELAY)
