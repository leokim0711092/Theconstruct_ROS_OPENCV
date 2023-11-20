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
        face_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3/haar_cascades/frontalface.xml')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        ScaleFactor = 1.2

        minNeighbors = 3
        faces = face_cascade.detectMultiScale(gray, ScaleFactor, minNeighbors)
        for (x,y,w,h) in faces:
            
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,255,0),2)  
            roi = cv_image[y:y+h, x:x+w]
            
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