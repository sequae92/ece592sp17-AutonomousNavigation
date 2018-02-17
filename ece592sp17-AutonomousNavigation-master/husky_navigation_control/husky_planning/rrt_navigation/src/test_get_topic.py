#!/usr/bin/env python

import rospy

if __name__=='__main__':
    topic_list = rospy.get_published_topics()
    print topic_list
    print  ['/gazebo/link_states', 'gazebo_msgs/LinkStates'] in topic_list
