#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rrt_navigation.msg import waypoints
from path_following.msg import temp_target

current_path_cell = []
current_path_meter = []
points_meter_pair = []
flag_lst = []
pub = rospy.Publisher('/waypoint_target', temp_target,queue_size=10)

def call_path(msg):
    #global points_meter_pair, target_pos_meter, init_node_meter
    global flag_lst
    global current_path_cell, current_path_meter,points_meter_pair
    global current_pos_meter, robot_orientation
    if current_path_cell==[] and  current_path_meter==[]:
        current_path_cell = msg.mid_points_cell
        current_path_meter =  msg.mid_points_meter
        for i in range(len(current_path_meter)/2):
            points_meter_pair.append((current_path_meter[2*i],current_path_meter[2*i+1]))
        flag_lst = flag_lst = [True]+[False]*(len(points_meter_pair)-1)
        print "initial!!"
        print flag_lst

    if len(current_path_cell) > len(msg.mid_points_cell):
        print "update"
        current_path_cell = msg.mid_points_cell
        current_path_meter =  msg.mid_points_meter
        for i in range(len(current_path_meter)/2):
            points_meter_pair.append((current_path_meter[2*i],current_path_meter[2*i+1]))
        flag_lst = flag_lst = [True]+[False]*(len(points_meter_pair)-1)

        print flag_lst
    else:
        print "no update"
    index_false = flag_lst.index(False)
    temp_waypoint = points_meter_pair[index_false]
    print index_false, temp_waypoint
    if distance(temp_waypoint, current_pos_meter) < 0.2:
        flag_lst[index_false] = True
        print index_false
        index_false = flag_lst.index(False)
        print index_false
        temp_waypoint = points_meter_pair[index_false]
        print index_false, temp_waypoint
    string = temp_target()
    string.x = temp_waypoint[0]
    string.y = temp_waypoint[1]
    string.rotation = 0
    pub.publish(string)



def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def get_orientation(msg):
    q_1 = msg.pose.pose.orientation.x
    q_2 = msg.pose.pose.orientation.y
    q_3 = msg.pose.pose.orientation.z
    q_0 = msg.pose.pose.orientation.w
    return math.atan2(2*(q_0*q_3+q_1*q_2), 1-2*(q_2**2+q_3**2))

def get_current_pos(msg):
    global current_pos_meter, robot_orientation
    current_pos_meter = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    robot_orientation = get_orientation(msg)


def main():
    rospy.init_node('handle_waypoints',  anonymous=True)
    rospy.Subscriber("/way_point_info",waypoints, call_path ,queue_size=1)
    rospy.Subscriber("/odometry/filtered",Odometry, get_current_pos,queue_size=10)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
