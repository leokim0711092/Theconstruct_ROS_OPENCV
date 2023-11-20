#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class ShowingImage(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #Define a kernel for the erosion 
        kernel_a = np.ones((5,5),np.uint8)
        erosion = cv2.erode(cv_image,kernel_a,iterations = 1)

        #Define a kernel for the dilation
        kernel_b = np.ones((3,3),np.uint8)
        dilation = cv2.dilate(cv_image,kernel_b,iterations = 1)

        #Define a kernel for the opening
        kernel_c = np.ones((7,7),np.uint8)
        opening = cv2.morphologyEx(cv_image, cv2.MORPH_OPEN, kernel_c)

        #Define a kernel for the closing
        kernel_d = np.ones((7,7),np.uint8)
        closing = cv2.morphologyEx(cv_image, cv2.MORPH_CLOSE, kernel_d)

        cv2.imshow('Original',cv_image)
        cv2.imshow('Erosion',erosion)
        cv2.imshow('Dilation',dilation)
        cv2.imshow('Opening',opening)
        cv2.imshow('Closing',closing)

        cv2.waitKey(0)



def main():
    showing_image_object = ShowingImage()
    rospy.init_node('line_following_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()