#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

#run below in commond line
#rosparam set map lidar

"""
map{lidar, orbslam, zed}
odom{/odometry/filtered, /odom/orbslam, /odom/zed}
if map is None, odom is /odometry/filtered

"""
topic_list = rospy.get_published_topics()

flag_laser = ['/scan', 'sensor_msgs/LaserScan'] in topic_list

if flag_laser:
    pub = rospy.Publisher('send_fake_tracking', Int16, queue_size=1, latch=True)
    rospy.init_node('fake_node')
    r = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        pub.publish(4)
        r.sleep()
else:
    rospy.loginfo("NO Laser Scan")
