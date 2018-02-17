#!/usr/bin/env python

import rospy
#import math
import numpy as np
#from random import randint
#from math import sqrt
#from rrt_navigation.msg import waypoints
#from rrt_auxilary import *
#from geometry_msgs.msg import Twist,TransformStamped, PoseStamped
#import geometry_msgs.msg
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


def back_to_1d_array(array):
    height = len(array)
    width = len(array[0])
    temp = np.reshape(array,(height*width))
    temp.astype(int)

    return temp.tolist()

def reshape_array(width, height, array):

    return np.reshape(array,(height, width))

def check_near_by_obs(cell, pair):
    row_num = cell[0]
    col_num = cell[1]
    near_cell = [[row_num-1, col_num-1],[row_num-1, col_num],[row_num-1, col_num+1],
                 [row_num  , col_num-1],                     [row_num  , col_num+1],
                 [row_num+1, col_num-1],[row_num+1, col_num],[row_num+1, col_num+1]]
    for item in near_cell:
        if item in pair:
            return True
    return False

def check_one_unknown(cell, pair):
    row_num = cell[0]
    col_num = cell[1]
    near_cell = [[row_num-1, col_num-1],[row_num-1, col_num],[row_num-1, col_num+1],
                 [row_num  , col_num-1],                     [row_num  , col_num+1],
                 [row_num+1, col_num-1],[row_num+1, col_num],[row_num+1, col_num+1]]
    for item in near_cell:
        if item in pair:
            return False
    return True

#fail, time
def map_unknown_reduce(array):
    r,c = np.where(array[:]<0)
    pair = []
    for num in range(len(r)):
        pair.append([r[num], c[num]])
    temp_array = array.copy()

    num_row = len(array)
    num_col = len(array[0])
    for cell in pair:
        one_unknown = check_one_unknown(cell, pair)
        print one_unknown
        if one_unknown:
            temp_array[cell[0]][cell[1]] = 0
    return temp_array


def map_obstacle_expand(array, resolution, threshold):
    #create boolean array for obstacle, it obstacle, true
    #bool_array = (array>threshold)
    #index of cell of obstacle
    r,c = np.where(array[:]>threshold)
    #create pair r,c
    pair = []
    for num in range(len(r)):
        pair.append([r[num], c[num]])
    temp_array = array.copy()

    num_row = len(array)
    num_col = len(array[0])
    #print num_row, num_col
    #keep 0.25 meter distance
    expand_cell = int(0.35/resolution)
    #print expand_cell
    flag = rospy.get_param("filter_on")


    if flag is "true":
        for cell in pair:

            two_obstacle = check_near_by_obs(cell, pair)

            if two_obstacle:
            #print cell
                slice_r_center = cell[0]
                slice_c_center = cell[1]
                slice_r_start = slice_r_center-expand_cell
                slice_r_end   = slice_r_center+expand_cell+1
                slice_c_start = slice_c_center-expand_cell
                slice_c_end   = slice_c_center+expand_cell+1
                slice_r = range(slice_r_start,slice_r_end,1)
                slice_r_list = [x for x in slice_r if x>=0 and x<num_row]
                slice_c = range(slice_c_start,slice_c_end,1)
                slice_c_list = [x for x in slice_c if x>=0 and x<num_col]
        #print slice_c_list
                for i in slice_r_list:
                    try:
                        temp_array[i][slice_c_list[0]:slice_c_list[-1]] = 100
                    except:
                        print cell, slice_c_list
            else:
                temp_array[cell[0]][cell[1]] = 0

    else:
        for cell in pair:

            slice_r_center = cell[0]
            slice_c_center = cell[1]
            slice_r_start = slice_r_center-expand_cell
            slice_r_end   = slice_r_center+expand_cell+1
            slice_c_start = slice_c_center-expand_cell
            slice_c_end   = slice_c_center+expand_cell+1
            slice_r = range(slice_r_start,slice_r_end,1)
            slice_r_list = [x for x in slice_r if x>=0 and x<num_row]
            slice_c = range(slice_c_start,slice_c_end,1)
            slice_c_list = [x for x in slice_c if x>=0 and x<num_col]
        #print slice_c_list
            for i in slice_r_list:
                try:
                    temp_array[i][slice_c_list[0]:slice_c_list[-1]] = 100
                except:
                    print cell, slice_c_list


    return temp_array

    """
    #create boolean array for obstacle, it obstacle, true
    #bool_array = (array>threshold)
    #index of cell of obstacle
    r,c = np.where(array[:]>threshold)
    #create pair r,c
    pair = []
    for num in range(len(r)):
        pair.append((r[num], c[num]))
    temp_array = array
    num_row = len(array)
    num_col = len(array[0])
    #print num_row, num_col
    #keep 1.5 meter distance
    expand_cell = int(0.25/resolution)
    #print expand_cell
    for cell in pair:
        #print cell
        slice_r_center = cell[0]
        slice_c_center = cell[1]
        slice_r_start = slice_r_center-expand_cell
        slice_r_end   = slice_r_center+expand_cell+1
        slice_c_start = slice_c_center-expand_cell
        slice_c_end   = slice_c_center+expand_cell+1
        slice_r = range(slice_r_start,slice_r_end,1)
        slice_r_list = [x for x in slice_r if x>=0 and x<num_row]
        slice_c = range(slice_c_start,slice_c_end,1)
        slice_c_list = [x for x in slice_c if x>=0 and x<num_col]
        #print slice_c_list
        for i in slice_r_list:
            try:
                temp_array[i][slice_c_list[0]:slice_c_list[-1]] = 100
            except:
                print cell, slice_c_list

    return temp_array
    """
def map_unknown(array):
    temp = [0]*len(array)
    for i in range(len(array)):
        if array[i] != -1:
            temp[i] = array[i]

    """
    temp = array
    #index of cell of unknown
    r,c = np.where(array[:]== -1)
    #make a pair
    pair = []
    for num in range(len(r)):
        pair.append((r[num], c[num]))

    for cell in pair:
        temp[cell[0]][cell[1]] = 0
    """
    return temp


def callback(map_msg, pubs):
    obs_map = pubs[0]
    unk_map = pubs[1]
    array = map_msg.data
    resolution = map_msg.info.resolution
    width = map_msg.info.width
    height = map_msg.info.height
    threshold = 25
    # for unknown map
    unk_map_msg = OccupancyGrid()
    unk_map_msg.info = map_msg.info
    unk_data = map_unknown(array)
    unk_map_msg.data = unk_data
    unk_map.publish(unk_map_msg)


    obs_map_msg = OccupancyGrid()
    obs_map_msg.info = map_msg.info

    data = reshape_array(width, height, unk_data)

    obs_data = map_obstacle_expand(data, resolution, threshold)
    #obs_data = map_unknown_reduce(obs_data)
    obs_map_msg.data = back_to_1d_array(obs_data)

    obs_map.publish(obs_map_msg)


    #r = rospy.Rate(1)
    #r.sleep()


def map_handle(pub1, pub2):
    rospy.init_node('map_obstacle_handler', anonymous=True)

    """
    need to handle map topic?
    """
    #odom_name=rospy.get_param('odom_topic_name','/odom')
    map_name=rospy.get_param('map_topic_name','/map')

    rospy.Subscriber(map_name, OccupancyGrid, callback,(pub1, pub2))



    rospy.spin()


if __name__ == '__main__':
    obstacle_map = rospy.Publisher('/map_obstacle_expand', OccupancyGrid,queue_size=0, latch = True)
    unknown_map = rospy.Publisher('/map_unknown2clear', OccupancyGrid,queue_size=5, latch= True)
    map_handle(obstacle_map,unknown_map)
