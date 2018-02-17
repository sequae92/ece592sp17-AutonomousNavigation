#!/usr/bin/env python

## husky_controller node will implement PID control or other control features for husky movement based on the path from navigation module. it  will publish to /motioncontrol topic with velocities.msg type

import rospy
from std_msgs.msg import String
from husky_controller.msg import velocities
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

from matplotlib import pyplot as plt
from PIL import Image


global xlin_vel,zang_vel
global header,mapWidth,mapHeight,mapTime,mapResolution,mapOrigin,mapData,mapDataResized
global mapDataSet

def occupancygrid_callback(data):
    global header,mapWidth,mapHeight,mapTime,mapResolution,mapOrigin,mapData,mapDataResized
    global mapDataSet
    mapDataSet=True
    header=data.header
    mapWidth=data.info.width
    mapHeight=data.info.height
    mapTime=data.info.map_load_time
    mapResolution=data.info.resolution
    mapOrigin=data.info.origin
    mapData=data.data
    mapDataResized=np.reshape(mapData,(-1,mapWidth))
    mapDataResized=mapDataResized[1000:3000,1000:3000]

    print "header :" + str(header)
    print "Width :" + str(mapWidth)
    print "Height :" + str(mapHeight)
    print "Time :" + str(mapTime)
    print "Resolution :" + str(mapResolution)
    print np.unique(mapData)
    print "-------------------------------------------------------"






def huskycontrolloop():
    global header,mapWidth,mapHeight,mapTime,mapResolution,mapOrigin,mapData,mapDataResized
    global xlin_vel,zang_vel
    global mapDataSet
    pub = rospy.Publisher('/husky_controller/velocitycontrol', velocities,queue_size=10)
    rospy.init_node('husky_controller', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid , occupancygrid_callback)
    rate=rospy.Rate(10)
    xlin_vel=0.25
    zang_vel=0.25
    #plt.ion()
    while not rospy.is_shutdown():
        pub.publish(xlin_vel,zang_vel)
        if mapDataSet == True:
            #cv2.imshow("a",mapDataResized)
            #cv2.waitKey(1)
            print "display"
            mapList=list(mapDataResized)

            plt.imshow(mapList,cmap='Spectral', vmin=-1, vmax=1)
            plt.show(block=False)
            #plt.draw()
        rate.sleep()

if __name__ == '__main__':
    global xlin_vel,zang_vel
    global mapDataSet
    mapDataSet=False
    xlin_vel=0
    zang_vel=0
    huskycontrolloop()


