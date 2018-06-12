import cv2
import math
import numpy as np

def extract_polygon(img, slice_num=16, LB=np.array([0,0,0]), UB=np.array([180,255,60])):

    IMG_HEIGHT, IMG_WIDTH,_ = img.shape
    X_DIV = int(IMG_HEIGHT/float(slice_num))
    kernelOpen = np.ones((5,5))
    kernelClose = np.ones((20,20))
    poly_points = [None] * slice_num
    detected_contours = [None] * slice_num
    
    for i in range(0, slice_num) :
        sliced_img = img[X_DIV*i + 1:X_DIV*(i+1), int(0):int(IMG_WIDTH)]
        blur = cv2.GaussianBlur(sliced_img,(5,5),0)
        imgHSV = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(imgHSV, LB, UB)
        maskOpen = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
        maskFinal = maskClose
        _,conts,_ = cv2.findContours(maskFinal.copy(),\
                    cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        if(conts):
            c = max(conts, key = cv2.contourArea)
            
            M = cv2.moments(c)
            poly_points[i] = (int(M['m10']/M['m00']), int(M['m01']/M['m00']) + X_DIV * i)
            detected_contours[i] = c

        contours = [i for i in detected_contours if i is not None]
        points = [i for i in poly_points if i is not None]

    return points, contours

