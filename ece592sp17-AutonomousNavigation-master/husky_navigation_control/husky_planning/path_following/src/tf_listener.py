#!/usr/bin/env python

import rospy
import math
import tf
import tf2_ros

import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('tf_husky',  anonymous=True)
    tfBuffer = tf2_ros.Buffer()

    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    pub = rospy.Publisher('/husky_location_tf', geometry_msgs.msg.TransformStamped ,queue_size=1)
    br = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            #rospy.loginfo(trans)
            print trans
            #add information into tf topic
            br.sendTransform(trans)
            #create a new topic for publishing tf
            pub.publish(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
