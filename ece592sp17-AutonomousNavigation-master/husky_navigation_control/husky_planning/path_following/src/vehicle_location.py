#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from path_following.msg import temp_target
# pub  gmetry_msgs/Twist

pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=10)




def set_target(msg):
    global x_tgt, y_tgt, theta_tgt, P
    target = [msg.x, msg.y, msg.rotation]
    P = 1
    print target
    x_tgt, y_tgt, theta_tgt = target


def e_angle_handler(theta_t, theta_c):
    e_init = theta_t - theta_c
    if abs(e_init) > math.pi:
        if theta_c > 0:
            theta_c_new = theta_c -2*math.pi
        elif theta_c < 0:
            theta_c_new = theta_c +2*math.pi
    else:
        theta_c_new = theta_c


    e_theta =  theta_t - theta_c_new

    return e_theta

def callback(msg):
    global x_tgt, y_tgt, theta_tgt, P
    #rospy.loginfo('   ')
    #rospy.loginfo('x_t: %f, y_t: %f, theta_t: %f' %(x_tgt,y_tgt, theta_tgt))
    x_current = msg.pose.pose.position.x
    y_current = msg.pose.pose.position.y
    q_1 = msg.pose.pose.orientation.x
    q_2 = msg.pose.pose.orientation.y
    q_3 = msg.pose.pose.orientation.z
    q_0 = msg.pose.pose.orientation.w
    theta_current = math.atan2(2*(q_0*q_3+q_1*q_2), 1-2*(q_2**2+q_3**2))
    #rospy.loginfo('x_c: %f, y_c: %f, theta_c: %f' %(x_current,y_current, theta_current) )

    e_x     = x_tgt - x_current
    e_y     = y_tgt - y_current
    #### the angular
    path_angle = math.atan2(e_y,e_x)
    e_theta_path = e_angle_handler(path_angle, theta_current)

    e_theta = e_angle_handler(theta_tgt, theta_current)
    #rospy.loginfo('path angle: %f' %path_angle)
    #rospy.loginfo('x_e: %f, y_e: %f, theta_e_path: %f' %(e_x,e_y, e_theta_path) )


    string = Twist()
    #string.linear.y = 0
    #string.linear.z = 0
    #string.angular.x = 0


    if abs(e_theta_path) > 0.1:
        w = P*e_theta_path/10
        string.linear.x = 0.02
        string.angular.z = w

    else:
        dist = math.sqrt((e_x)**2+(e_y)**2)
        rospy.loginfo('error distance: %f' %dist)
        if dist > 0.05:
            x = 2*P*dist
            string.linear.x = x
            string.angular.z = 0
            #pub.publish(string)
        else:
            if abs(e_theta) > 0.1:
                w = P*e_theta_path/2
                string.linear.x = 0.02
                string.angular.z = w
                #pub.publish(string)
    pub.publish(string)

def main():
    rospy.init_node('Location_info',  anonymous=True)
    rospy.Subscriber("/waypoint_target",temp_target, set_target )
    rospy.Subscriber("/husky_velocity_controller/odom",Odometry, callback )
    rospy.spin()

if __name__=='__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
