#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Pose,PoseStamped

global pub
global path
global poses
global poses,pose_stamped
global odom
def callback_odom(data):
    global path
    global pub
    global poses,odom,seq,prev_odom
    odom=data
    if (abs(odom.pose.pose.position.x-prev_odom.pose.pose.position.x) > 0.01) or (abs(odom.pose.pose.position.y-prev_odom.pose.pose.position.y) > 0.01):
        pose_stamped=PoseStamped()
        pose_stamped.header.stamp=rospy.Time.now()
        pose_stamped.header.frame_id='odom'
        pose_stamped.header.seq=seq
        seq+=1
        pose_stamped.pose=odom.pose.pose
        poses.append(pose_stamped)
        path.poses=poses
        pub.publish(path)
    prev_odom=odom

if __name__ == '__main__':
    global path
    global pub
    global poses,odom,seq,prev_odom
    seq=1
    poses=[]
    odom=Odometry()
    prev_odom=Odometry()
    path=Path()

    rospy.init_node('path_recorder')

    path.header.frame_id='odom'
    path.header.stamp=rospy.Time.now()

    rospy.Subscriber('/husky_velocity_controller/odom',Odometry,callback_odom)

    pub=rospy.Publisher('/odom/path',Path,queue_size=10)
    rospy.spin()


