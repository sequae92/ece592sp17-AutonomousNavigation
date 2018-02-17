#!/usr/bin/env python

import roslib
import rospy

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


class sending_points():
    def __init__(self):
        self.path_pub = rospy.Publisher("/Path_disp",Path,queue_size=10, latch = True)

    def path_sender(self, points_lst):
        #make sure points_lst has least 2 points
        if len(points_lst) <2:
            rospy.loginfo("No Path")
            return
        #initial the path class
        rrt_path = Path()
        rrt_path.header.frame_id = 'map'

        for location in points_lst:
            pose = PoseStamped()
            pose.pose = location
            rrt_path.poses.append(pose)

        self.path_pub.publish(rrt_path)


def pt_2_Pose(pt):
    location = Pose()
    location.position.x = pt[0]
    location.position.y = pt[1]
    location.position.z = 0
    return location
