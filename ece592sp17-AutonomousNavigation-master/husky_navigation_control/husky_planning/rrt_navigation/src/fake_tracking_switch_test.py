#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16


topic_list = rospy.get_published_topics()
# get sensor info, if no parameter, set sensor_map to None

"""
"""

pub = rospy.Publisher('send_fake_tracking', Int16, queue_size=1, latch=True)
rospy.init_node('fake_node')

r = rospy.Rate(1) # 1hz
while not rospy.is_shutdown():
    now = rospy.get_rostime()
    sec = now.secs
    if sec % 50 != 0:
        pub.publish(4)
        r.sleep()
    else:
        pub.publish(3)
        r.sleep()
