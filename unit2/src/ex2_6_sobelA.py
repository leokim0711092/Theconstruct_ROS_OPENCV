#!/usr/bin/env python

import cv2
import numpy as np

img = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_2/Course_images/test_img_b.jpg')

#Convert the image to gray scale so the gradient is better visible
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img = cv2.resize(img,(450,350))

#Apply the horizontal sobel operator with a kernel size of 3
sobelx = cv2.Sobel(img,cv2.CV_64F,1,0,ksize=3)

#Apply the vertical sobel operator with a kernel size of 3
sobely = cv2.Sobel(img,cv2.CV_64F,0,1,ksize=3)

cv2.imshow('Original',img)
cv2.imshow('X',sobelx)
cv2.imshow('Y',sobely)

cv2.waitKey(0)
cv2.destroyAllWindows()