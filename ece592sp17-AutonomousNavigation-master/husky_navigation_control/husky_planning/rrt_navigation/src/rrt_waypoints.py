#!/usr/bin/env python

import rospy
import math
#from nav_msgs.msg import OccupancyGrid
import numpy as np
from random import randint
from math import sqrt
from rrt_navigation.msg import waypoints
from rrt_auxilary import *
from geometry_msgs.msg import Twist
import geometry_msgs.msg
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String

robot_pos = (0,0)
robot_orientation = 0
pub_rrt = rospy.Publisher('/way_point_info', waypoints,queue_size=10)
pub_init_move = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=10)

print "enter target point, 2 values seperated by space(unit:m):"
s = raw_input().split(" ")
target_pos =  (float(s[0]), float(s[1]))

current_path_cell = []
current_path_meter = []

"""
use numpy would be more efficient?

def reshape_array(width, height, array):
    re_array = []
    for i in range(1,height+1):
        re_array.append(array[0+(i-1)*width:(i)*width-1])
    return re_array
"""
def reshape_array(width, height, array):

    return np.reshape(array,(height, width))


def get_robot_pos(msg):
    global robot_pos, robot_orientation
    robot_pos = ( msg.transform.translation.x, msg.transform.translation.y)
    q_1 = msg.transform.rotation.x
    q_2 = msg.transform.rotation.y
    q_3 = msg.transform.rotation.z
    q_0 = msg.transform.rotation.w
    robot_orientation = math.atan2(2*(q_0*q_3+q_1*q_2), 1-2*(q_2**2+q_3**2))

def meter_to_cell(origin_meter, pos_meter, resolution):
    x_cell = int(round((pos_meter[0]-origin_meter[0])/resolution))
    y_cell = int(round((pos_meter[1]-origin_meter[1])/resolution))
    return (x_cell, y_cell)

def cell_to_meter(origin_meter, cell_pose, resolution):
    x_meter = cell_pose[0]*resolution+origin_meter[0]
    y_meter = cell_pose[1]*resolution+origin_meter[1]
    return (x_meter, y_meter)

def rrt_callback(msg):
    global robot_pos, robot_orientation, target_pos
    global current_path_cell, current_path_meter
    print msg.info

    str_init_move = Twist()

    resolution =  msg.info.resolution
    width =  msg.info.width
    height =  msg.info.height
    #unit in meter
    origin_cell_pos = (msg.info.origin.position.x, msg.info.origin.position.y)

    map_array =  msg.data
    # reshape the array to height*width
    map_array = reshape_array(width, height, map_array)

    #print robot_pos, robot_orientation

    robot_x_cell, robot_y_cell = meter_to_cell(origin_cell_pos, robot_pos, resolution)

    #print robot_x_cell, robot_y_cell
    threshold = 0.25
    robot_pos_cell = (robot_x_cell, robot_y_cell)
    target_pos_cell = meter_to_cell(origin_cell_pos, target_pos, resolution)
    if not check_feasible(robot_pos_cell, threshold,map_array):
        print "scan enviroment"
        for i in range(20):
            str_init_move.linear.x = 0.1
            str_init_move.angular.z = 0.1
            pub_init_move.publish(str_init_move)
        for i in range(20):
            str_init_move.linear.x = -0.1
            str_init_move.angular.z = -0.1
            pub_init_move.publish(str_init_move)
    step = 40
    hit_tgt = False




    rrt = Tree(robot_pos_cell, target_pos_cell ,step, hit_tgt, height, width, map_array, threshold)
    #print rrt.tgt
    while len(rrt.nodes) <= 100 and rrt.hit_tgt == False:
        rrt.generate_rand(width, height)
    #print "rand point",rrt.rand_pt
        c_pt = rrt.close_node()
    #print "close node:", c_pt
        rrt.get_next_node(c_pt)
    #print rrt.nodes
        temp = rrt.near_target()

    if len(rrt.nodes) <= 100:
        closest_node = temp[1]
        path = find_path(closest_node, rrt.tgt,rrt.lines)
    else:
        closest_node = connect_target(rrt)

        rrt.lines.append((closest_node, rrt.tgt))
        path = find_path(closest_node, rrt.tgt,rrt.lines)

    #print path
    point_path = []
    for i in range(len(path)):
        point_path.append(path[i][0][0])

        point_path.append(path[i][0][1])
        if i == len(path)-1:
            point_path.append(path[i][1][0])
            point_path.append(path[i][1][1])

    #point_path = tuple(point_path)
    print "target cell position: ", rrt.tgt
    print "target position in meter: ", target_pos
    print point_path
    points_lst = []
    for i in range(len(point_path)/2):
        (x,y)=cell_to_meter(origin_cell_pos, (point_path[2*i],point_path[2*i+1]), resolution)
        points_lst.append(x)
        points_lst.append(y)
        """
        points_lst.append(round(x,4))
        points_lst.append(round(y,4))
        """
    print points_lst

    if current_path_cell==[] and  current_path_meter==[]:
        current_path_cell = point_path
        current_path_meter =  points_lst
        print "initial!!"
        waypoints_msg = waypoints()
        waypoints_msg.mid_points_cell= point_path
        #why not working? decimal problem?
        waypoints_msg.list_in_meter = points_lst
        pub_rrt.publish(waypoints_msg)
    if len(current_path_cell) > len(point_path):
        print "update"
        current_path_cell = point_path
        current_path_meter =  points_lst
        waypoints_msg = waypoints()
        waypoints_msg.mid_points_cell = point_path
        waypoints_msg.list_in_meter = points_lst
    #print waypoints_msg
    #rospy.loginfo(waypoints_msg)
        pub_rrt.publish(waypoints_msg)


def main():
    rospy.init_node('handle_rrt',  anonymous=True)

    rospy.Subscriber("/husky_location_tf",geometry_msgs.msg.TransformStamped, get_robot_pos ,queue_size=1)
    rospy.Subscriber("/map",OccupancyGrid, rrt_callback ,queue_size=1)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
