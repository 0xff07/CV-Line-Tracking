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

    #cropped_top = img[int(0):int(IMG_HEIGHT / SLICE_NUM), int(0):int(IMG_WIDTH)]
    #cropped_bot = img[int(IMG_HEIGHT / SLICE_NUM):int(IMG_HEIGHT), int(0):int(IMG_WIDTH)]
    #image = [cropped_top, cropped_bot]

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
            x,y,w,h=cv2.boundingRect(conts[0])
            CM_x[i] = x + w/2
            CM_y[i] = y + h/2 + X_DIV * i
            CM_w[i] = w
            CM_h[i] = h
            cv2.rectangle(sliced_img[i],(x,y),(x+w,y+h),(0,0,255), 2)
        elif i > 0:
            CM_x[i] = CM_x[i-1]
            CM_y[i] = CM_y[i-1]
            CM_w[i] = CM_w[i-1]
            CM_h[i] = CM_h[i-1]
            
    # Do control with segs
    for i in range(0, SLICE_NUM - 1):
        cv2.line(img, (CM_x[i], CM_y[i]), (CM_x[i+1], CM_y[i+1]), (0, 255, 0), 10)
    cv2.imshow("cam",img)
    
    cv2.waitKey(10)
