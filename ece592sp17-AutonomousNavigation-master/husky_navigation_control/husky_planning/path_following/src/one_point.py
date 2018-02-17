#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import message_filters
import tf2_ros
from tf.transformations import euler_from_quaternion


# pub  gmetry_msgs/Twist




def e_angle_handler(theta_t, theta_c):
    if theta_t >= 0 and theta_c >= 0:
        e_init = theta_t - theta_c
        return e_init
    elif theta_t <= 0 and theta_c <= 0:
        e_init = theta_t - theta_c
        return e_init
    elif theta_t >= 0 and theta_c <= 0:
        if theta_c <= theta_t- math.pi:
            e_init = theta_t - (theta_c+2*math.pi)
            return e_init
        else:
            e_init = theta_t - theta_c
            return e_init
    else:
        if theta_c <= theta_t + math.pi:
            e_init = theta_t - theta_c
            return e_init
        else:
            e_init = theta_t - (theta_c-2*math.pi)
            return e_init




def callback(msg):
    global x_tgt, y_tgt, theta_tgt, P
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=1)
    rospy.loginfo(msg)

    rospy.loginfo('x_t: %f, y_t: %f, theta_t: %f' %(x_tgt,y_tgt, theta_tgt))
    x_current = msg.transform.translation.x
    y_current = msg.transform.translation.y
    
    q_1 = msg.transform.rotation.x
    q_2 = msg.transform.rotation.y
    q_3 = msg.transform.rotation.z
    q_0 = msg.transform.rotation.w

    quaternion =(q_1,q_2, q_3, q_0)
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)
    # postion not update at the same time??

    """

    """

    theta_current = yaw
    rospy.loginfo('x_c: %f, y_c: %f, theta_c: %f' %(x_current,y_current, theta_current) )

    e_x     = x_tgt - x_current
    e_y     = y_tgt - y_current
    #### the angular
    path_angle = math.atan2(e_y,e_x)
    #e_theta_path = path_angle-theta_current
    print path_angle, theta_current
    e_theta_path = e_angle_handler(path_angle, theta_current)

    #e_theta = e_angle_handler(theta_tgt, theta_current)
    e_theta = theta_tgt-theta_current
    #rospy.loginfo('path angle: %f' %path_angle)
    #rospy.loginfo('x_e: %f, y_e: %f, theta_e_path: %f' %(e_x,e_y, e_theta_path) )


    string = Twist()

    dist = math.sqrt((e_x)**2+(e_y)**2)

    if dist > 0.15:

        if abs(e_theta_path) > 0.05:
            w = e_theta_path
            w = min(w, 0.08)
            string.linear.x = 0
            string.angular.z = w
            print "turning"
        else:
            x = dist
            x = min(0.8,x)
            string.linear.x = x
            string.angular.z = 0
            print "heading"

    else:
        string.linear.x = 0
        string.angular.z = 0
        print "reach target"
        print "enter target point and orientation, 3 values seperated by space(unit:m):"

        s = raw_input().split(" ")
        target =  (float(s[0]), float(s[1]), float(s[2]))
        x_tgt, y_tgt, theta_tgt = target
    pub.publish(string)
"""
def" call_od_data(msg):
    x_current = msg.pose.pose.position.x
    y_current = msg.pose.pose.position.y
    q_1 = msg.pose.pose.orientation.x
    q_2 = msg.pose.pose.orientation.y
    q_3 = msg.pose.pose.orientation.z
    q_0 = msg.pose.pose.orientation.w
    print "it is odem"
    rospy.loginfo('x_c: %f, y_c: %f' %(x_current,y_current))
"""




def main():
    rospy.init_node('Location_info',  anonymous=True)
    #rospy.Subscriber("/odometry/filtered",Odometry, call_od_data )
    rospy.Subscriber('/husky_location_tf', TransformStamped, callback )
    #pose_sub = message_filters.Subscriber('/husky_location_tf', TransformStamped)
    #ts = message_filters.TimeSynchronizer([pose_sub], 10)
    #ts.registerCallback(callback)
    rospy.spin()

if __name__=='__main__':
    print "enter target point and orientation, 3 values seperated by space(unit:m):"
    s = raw_input().split(" ")
    target =  (float(s[0]), float(s[1]), float(s[2]))
    #target = [1, 2, 0]
    P = 1.5
    x_tgt, y_tgt, theta_tgt = target
    try:
        main()
    except:
        pass
