#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class LoadVideo(object):

    def __init__(self):
        self.ctrl_c = False 

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def video_detection (self,):
        cap = cv2.VideoCapture("/home/user/catkin_ws/src/unit3/Course_images/chris5-2.mp4")
        hog = cv2.HOGDescriptor()

        #We set the hog descriptor as a People detector
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


        
        while not self.ctrl_c:
            ret, frame = cap.read()

            img_original = cv2.resize(frame,(500,400))
            img = cv2.resize(frame,(500,400))     

            

            #We will define de 8x8 blocks in the winStride
            boxes, weights = hog.detectMultiScale(img, winStride=(8,8))
            boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

            for (xA, yA, xB, yB) in boxes:
                
                #Center in X 
                medX = xB - xA 
                xC = int(xA+(medX/2)) 

                #Center in Y
                medY = yB - yA 
                yC = int(yA+(medY/2)) 

                #Draw a circle in the center of the box 
                cv2.circle(img,(xC,yC), 1, (0,255,255), -1)

                # display the detected boxes in the original picture
                cv2.rectangle(img, (xA, yA), (xB, yB),
                                    (255, 255, 0), 2)    


            

            cv2.imshow('Face_original',img_original)

            cv2.imshow('Face',img)
            
                    

            cv2.waitKey(1)
        cap.release()
        





if __name__ == '__main__':
    rospy.init_node('load_video_node', anonymous=True)
    load_video_object = LoadVideo()
    try:
        load_video_object.video_detection()
        rospy.oncespin()
    except rospy.ROSInterruptException:
        pass
    
    cv2.destroyAllWindows()