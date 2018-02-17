#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from path_following.msg import check_path
from rrt_navigation.msg import path_flag


def reshape_array(width, height, array):

    return np.reshape(array,(height, width))

def array_to_pair(data):
    temp = []
    for i in range(len(data)/2):
        temp.append([data[2*i], data[2*i+1]])
    return temp

def check_obj_on_path(path_to_check, map_2D):

    pts_start = path_to_check[0:-1]
    pts_end = path_to_check[1:]
    for i in range(len(pts_end)):
        pt_start = pts_start[i]
        pt_end = pts_end[i]
        increment_x = pt_end[0] - pt_start[0]
        increment_y = pt_end[1] - pt_start[1]
        for j in range(1, 101):

            col = pt_start[0] + (j/100)*increment_x
            row = pt_start[1] + (j/100)*increment_y
            col = int(round(col))
            row = int(round(row))
            if map_2D[row][col] > 50:
                return False
    return True


global num, lst_1D

def callback_path(msg):
    global num, lst_1D
    num = msg.passed
    lst_1D = msg.mid_points_cell

def callback_map(msg):
    global num, lst_1D
    #map_resolution = msg.info.resolution
    map_width = msg.info.width
    map_height = msg.info.height
    #map_origin = msg.info.origin
    map_1D = msg.data
    map_2D = reshape_array(map_width, map_height, map_1D)

    points_pair = array_to_pair(lst_1D)
    path_to_check = points_pair[num:]

    no_obj_path = check_obj_on_path(path_to_check, map_2D)

    result = path_flag()
    result.keepgoing = no_obj_path
    pub = rospy.Publisher('/check_path', path_flag,queue_size=1, latch = True)
    pub.publish(result)


def detect_handle():
    rospy.init_node('path_obstacle_handler', anonymous=True)
    #pub = rospy.Publisher('/way_point_info', waypoints,queue_size=1)

    rospy.Subscriber("/send_current_path", check_path, callback_path)

    #map_name=rospy.get_param('map_topic_name','/map')

    #rospy.Subscriber(map_name, OccupancyGrid, callback_map)
    rospy.Subscriber('/map_obstacle_expand', OccupancyGrid, callback_map)

        


    rospy.spin()


if __name__ == '__main__':

    #obstacle_map = rospy.Publisher('/map_obstacle_expand', OccupancyGrid,queue_size=0, latch = True)
    #unknown_map = rospy.Publisher('/map_unknown2clear', OccupancyGrid,queue_size=5, latch= True)

    detect_handle()
