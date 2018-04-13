import cv2
import numpy as np
import math

SLICE_NUM = 2
lowerBound=np.array([0,0,0])
upperBound=np.array([180,255,100])

cam= cv2.VideoCapture(0)
kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))

seg_x = [0, 0]
seg_y = [0, 0]
seg_w = [0, 0]
seg_h = [0, 0]




while True:
    ret, img=cam.read()
    img=cv2.resize(img,(340,220))

    height, width = img.shape[:2]
    cropped_top = img[int(0):int(height * 0.5), int(0):int(width)]
    cropped_bot = img[int(height * 0.5):int(height), int(0):int(width)]
    image = [cropped_top, cropped_bot]
    #cv2.imshow("top",image[0])    
    #cv2.imshow("bot",image[1])    

    for i in range(0,SLICE_NUM) :

    #convert BGR to HSV
        imgHSV= cv2.cvtColor(image[i],cv2.COLOR_BGR2HSV)
    # create the Mask
        mask=cv2.inRange(imgHSV,lowerBound,upperBound)
    #morphology
        maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
        maskFinal=maskClose
        _, conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(image[i],conts,-1,(255,0,0),3)
        if(conts):
            x,y,w,h=cv2.boundingRect(conts[0])
            seg_x[i] = x + w/2
            seg_y[i] = y + h/2
            seg_w[i] = w
            seg_h[i] = h
            msg =  ("x" + str(i) + "=" + str(seg_x[i])) + ("y" + str(i) + "=" + str(seg_y[i]))
            print msg
            cv2.rectangle(image[i],(x,y),(x+w,y+h),(0,0,255), 2)
        cv2.imshow("maskClose" + str(i),maskFinal)
        cv2.imshow("cam" + str(i),image[i])

    # Do control with segs
    
    #cv2.imshow("maskOpen",maskOpen)
    #cv2.imshow("mask",mask)
    cv2.line(img, (seg_x[0], seg_y[0]), (seg_x[1], seg_y[1] + height/2), (0, 255, 0), 10)
    cv2.imshow("cam",img)
    
    cv2.waitKey(10)
