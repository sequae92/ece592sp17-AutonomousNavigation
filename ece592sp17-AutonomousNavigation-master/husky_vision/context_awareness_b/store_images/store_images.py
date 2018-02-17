#!/usr/bin/python
from subprocess import call
import os.path
import os
import rospy
import cv2
import numpy as np

from time import gmtime, strftime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *

global count
global pub
global cur_image,last_image,last_odom

def callback_image(data):
    global bridge
    global count
    global cur_image
    if count == 0:
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")[:320,:,:]
            print cv2_img.shape
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            filename='/home/robot/catkin_ws/src/ENet-training/visualize/workspace/input/cfl.jpg'
            cv2.imwrite(filename, cv2_img)
            print 'image saved '+filename+strftime(" %Y-%m-%d %H:%M:%S", gmtime())
            count=10
    else:
        count-=1
    cur_image=data

def odometryCb(msg):
    global cur_image,last_image,last_odom
    if not cur_image:
        last_odom=msg
        last_image=cur_image
	print msg.pose.pose


def main():
    global bridge
    global count
    global pub
    global cur_image,last_image,last_odom
    last_image=0
    cur_image=0
    last_odom=0
    count=10
    bridge = CvBridge()

    rospy.init_node('store_images')
    rospy.Subscriber('/camera/image_raw',Image,callback_image)
    rospy.Subscriber('/odom',Odometry,odometryCb)
    pub1 = rospy.Publisher('/enet/image',Image,queue_size=10)
    pub2 = rospy.Publisher('/enet/vector',Int32,queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            print "starting read"
            if os.path.isfile('/home/robot/catkin_ws/src/ENet-training/visualize/workspace/output/vector.csv'):
                print "file available"
                vector=np.loadtxt('/home/robot/catkin_ws/src/ENet-training/visualize/workspace/output/vector.csv')
                #cv2.imshow('frame',image)
                #cv2.waitKey(1)
               #pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
                print vector
                print vector.shape
                pub2.publish(vector)
               # os.remove('image.jpeg')
            print "vector published"
        except:
            print "error"
            pass



        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except:
        pass
