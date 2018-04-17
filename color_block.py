import cv2
import numpy as np
import math

SLICE_NUM = 2
IMG_WIDTH = 340
IMG_HEIGHT = 220
X_DIV = int(IMG_HEIGHT/float(SLICE_NUM))

lowerBound=np.array([0,0,0])
upperBound=np.array([180,255,100])

cam= cv2.VideoCapture(0)
kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))

CM_x = range(0, SLICE_NUM)
CM_y = range(0, SLICE_NUM)
CM_w = range(0, SLICE_NUM)
CM_h = range(0, SLICE_NUM)


sliced_img = range(0, SLICE_NUM)

while True:
    ret, img=cam.read()
    img=cv2.resize(img,(IMG_WIDTH,IMG_HEIGHT))

    for i in range(0,SLICE_NUM) :
        sliced_img[i] = img[X_DIV*i + 1:X_DIV*(i+1), int(0):int(IMG_WIDTH)]
        # convert BGR to HSV
        imgHSV= cv2.cvtColor(sliced_img[i],cv2.COLOR_BGR2HSV)
        # create the Mask
        mask=cv2.inRange(imgHSV,lowerBound,upperBound)
        # morphology
        maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
        maskFinal=maskClose
        _, conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(sliced_img[i],conts,-1,(255,0,0),3)
        if(conts):
            c = max(conts, key = cv2.contourArea)
            x,y,w,h=cv2.boundingRect(c)
            CM_x[i] = x + w/2
            CM_y[i] = y + h/2 + X_DIV * i
            CM_w[i] = w
            CM_h[i] = h
            cv2.rectangle(sliced_img[i],(x,y),(x+w,y+h),(0,0,255), 2)
    # Do control with segs

    for i in range(0, SLICE_NUM - 1):
        cv2.line(img, (CM_x[i], CM_y[i]), (CM_x[i+1], CM_y[i+1]), (0, 255, 0), 10)
    #cv2.imshow("cam",img)
    cv2.waitKey(10)
    vec_t_x = CM_x[0] - CM_x[1]
    vec_t_y = CM_y[0] - CM_y[1]
    pos = [CM_x[0]/2 + CM_x[1]/2 - IMG_WIDTH/2, CM_y[0]/2 + CM_y[1]/2 - IMG_HEIGHT/2]
    vec_t = [vec_t_x, vec_t_y]
    angle = math.acos(vec_t[0]/math.sqrt(vec_t[0]**2 + vec_t[1]**2))
    angle = angle * 180 / math.pi
    print "angle : " + str(angle) + " Deviation : " + str(pos[0])
