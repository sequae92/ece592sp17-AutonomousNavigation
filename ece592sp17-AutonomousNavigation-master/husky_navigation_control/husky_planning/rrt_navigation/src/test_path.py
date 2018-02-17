#!/usr/bin/env python

#import roslib
import rospy

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
#import custom function
from supplement_fcn_path import sending_points, pt_2_Pose
#from sending_path import sending_points
#from pair_to_pose_msg import pt_2_Pose


if __name__=='__main__':

    rospy.init_node('sending_simple_goals', anonymous=True)
    ini_lst = [(0,0), (2,2), (2,4), (-2,4), (-3,-3), (1,-5)]

    poses_collection = []
    for item in ini_lst:
        temp = pt_2_Pose(item)
        poses_collection.append(temp)

    path = sending_points()

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
       path.path_sender(poses_collection)
       r.sleep()
