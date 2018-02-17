#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rrt_navigation.msg import waypoints

pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=10)
reach_flag = False

def call_path(msg):
    global points_meter_pair, target_pos_meter, init_node_meter
    global flag_lst
    rospy.loginfo(msg)
    #print msg
    points_cell = msg.mid_points_cell
    points_meter = msg.mid_points_meter
    #print points
    points_meter_pair = []
    for i in range(len(points_meter)/2):
        points_meter_pair.append((points_meter[2*i],points_meter[2*i+1]))

    #print points_meter_pair
    target_pos_meter = points_meter_pair[-1]
    init_node_meter =  points_meter_pair[0]
    #print init_node_meter, target_pos_meter
    flag_lst = [True]+[False]*(len(points_meter_pair)-1)

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def get_orientation(msg):
    q_1 = msg.pose.pose.orientation.x
    q_2 = msg.pose.pose.orientation.y
    q_3 = msg.pose.pose.orientation.z
    q_0 = msg.pose.pose.orientation.w
    return math.atan2(2*(q_0*q_3+q_1*q_2), 1-2*(q_2**2+q_3**2))

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

def get_current_pos(msg):
    #global current_pos_meter, robot_orientation
    global points_meter_pair, target_pos_meter, init_node_meter
    global flag_lst, reach_flag
    current_pos_meter = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    robot_orientation = get_orientation(msg)

    #print current_pos_meter

    index_false = flag_lst.index(False)
    temp_target = points_meter_pair[index_false]
    #print temp_target
    e_x  = temp_target[0] - current_pos_meter[0]
    e_y  = temp_target[1] - current_pos_meter[1]
    #print e_x, e_y
    path_angle = math.atan2(e_y,e_x)
    e_theta_path = e_angle_handler(path_angle, robot_orientation)

    #e_theta = e_angle_handler(path_angle, robot_orientation)
    #rospy.loginfo('path angle: %f' %path_angle)
    #rospy.loginfo('x_e: %f, y_e: %f, theta_e_path: %f' %(e_x,e_y, e_theta_path) )
    string = Twist()
    #string.linear.y = 0
    #string.linear.z = 0
    #string.angular.x = 0
    while flag_lst[-1] == False:
        print "start!"
        if abs(e_theta_path) > 0.1:
            w = 1*e_theta_path/10
            string.linear.x = 0.02
            string.angular.z = w
            print "turning"
        else:
            dist = math.sqrt((e_x)**2+(e_y)**2)
            print "forward"
            if dist > 0.3:
                x = 2*1*dist
                string.linear.x = x
                string.angular.z = 0
            #pub.publish(string)
            elif dist >= 0 and dist <= 0.3:
                x = 0
                string.linear.x = x
                string.angular.z = 0
                print "close temp"
                flag_lst[index_false] = True
                print index_false
                index_false = flag_lst.index(False)
                print index_false
                temp_target = points_meter_pair[index_false]
                #print temp_target
                e_x  = temp_target[0] - current_pos_meter[0]
                e_y  = temp_target[1] - current_pos_meter[1]
                path_angle = math.atan2(e_y,e_x)
                e_theta_path = e_angle_handler(path_angle, robot_orientation)

        pub.publish(string)
    reach_flag = True

def main():
    global reach_flag
    rospy.init_node('handle_path',  anonymous=True)
    #while not reach_flag:
    rospy.Subscriber("/way_point_info",waypoints, call_path ,queue_size=1)
    rospy.Subscriber("/odometry/filtered",Odometry, get_current_pos,queue_size=10)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
