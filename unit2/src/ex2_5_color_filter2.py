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
            
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #Hsv limits are defined
        #here is where you define the range of the color you´re looking for
        #each value of the vector corresponds to the H,S & V values respectively
        min_green = np.array([40,0,220])
        max_green = np.array([70,220,255])

        min_red = np.array([0,0,220])
        max_red = np.array([10,255,0])

        min_blue = np.array([100,0,255])
        max_blue = np.array([130,220,255])


        #This is the actual color detection 
        #Here we will create a mask that contains only the colors defined in your limits
        #This mask has only one dimention, so its black and white }
        mask_g = cv2.inRange(hsv, min_green, max_green)
        mask_r = cv2.inRange(hsv, min_red, max_red)
        mask_b = cv2.inRange(hsv, min_blue, max_blue)

        #We use the mask with the original image to get the colored post-processed image
        res_b = cv2.bitwise_and(cv_image, cv_image, mask= mask_b)
        res_g = cv2.bitwise_and(cv_image, cv_image, mask= mask_g)
        res_r = cv2.bitwise_and(cv_image, cv_image, mask= mask_r)
        

        cv2.imshow('Green',res_g)
        cv2.imshow('Red',res_r)
        cv2.imshow('Blue',res_b)
        cv2.imshow('image',cv_image)
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