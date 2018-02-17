#!/usr/bin/python


#This node will stream video for manual control of husky to the topic /husky_eyes


import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def CamNode():

    eyes= rospy.Publisher('/husky_eyes', Image,queue_size=30 )
    bridge= CvBridge()
    camera=cv2.VideoCapture(0)

    while True:

    	ret,frame=camera.read()
        cv2.imshow('Stream',frame)
        Image_to_be_Sent= bridge.cv2_to_imgmsg(frame,encoding="bgr8")
        eyes.publish(Image_to_be_Sent)
        if cv2.waitKey(1) == 27:
            break

    camera.release()
    cv2.destroyAllWindows()
    quit()

if __name__ == '__main__':

    rospy.init_node('Husky_eyes',anonymous=True)
    CamNode()
    rospy.spin()
