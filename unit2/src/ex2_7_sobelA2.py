#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


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
        #Apply the horizontal sobel operator with a kernel size of 3
        sobelx = cv2.Sobel(cv_image,cv2.CV_64F,1,0,ksize=3)

        #Apply the vertical sobel operator with a kernel size of 3
        sobely = cv2.Sobel(cv_image,cv2.CV_64F,0,1,ksize=3)

        cv2.imshow('Original',cv_image)
        cv2.imshow('X',sobelx)
        cv2.imshow('Y',sobely)
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