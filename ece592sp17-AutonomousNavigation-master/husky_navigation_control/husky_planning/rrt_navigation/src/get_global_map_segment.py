#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from math import cos, sin

def meter_to_cell(origin_meter, pos_meter, resolution):
    x_cell = int(round((pos_meter[0]-origin_meter[0])/resolution))
    y_cell = int(round((pos_meter[1]-origin_meter[1])/resolution))
    return (x_cell, y_cell)

def cell_to_meter(origin_meter, cell_pose, resolution):
    x_meter = cell_pose[0]*resolution+origin_meter[0]
    y_meter = cell_pose[1]*resolution+origin_meter[1]
    return (x_meter, y_meter)

def reshape_array(width, height, array):

    return np.reshape(array,(height, width))

def back_to_1d_array(array):
    height = len(array)
    width = len(array[0])
    temp = np.reshape(array,(height*width))
    temp.astype(int)

    return temp.tolist()

def get_rect_corner_m(x, y, theta, w,h):
    w_half = w/2
    rotation_M = np.array([[cos(theta), -sin(theta)],
                           [sin(theta),  cos(theta)]])

    left_up = np.array([[-w_half],[h]])
    left_up = np.matmul(rotation_M, left_up) + np.array([[x],[y]])
    left_up = np.reshape(left_up, (2)).tolist()

    left_down = [x - w_half*cos(theta),
                 y - w_half*sin(theta)]
    right_down = [x + w_half*cos(theta),
                  y + w_half*sin(theta)]
    """
    left_up = [x - w_half*cos(theta) - h*sin(theta),
               y - w_half*sin(theta) + h*cos(theta)]
    """
    right_up =[x + w_half*cos(theta) - h*sin(theta),
               y + w_half*sin(theta) + h*cos(theta)]

    return [left_up, right_up, left_down, right_down]




def get_rec_corner_cell(lst, origin_meter, resolution):
    temp = []
    for corner in lst:
        cell = meter_to_cell(origin_meter, corner, resolution)
        temp.append(cell)
    return temp

"""
need revise
"""
def get_rela_ind_local(row, col):
    # since it is not asymmetry, so manully input
    center =14
    h_cell = 80
    col_rel = col - 14
    row_rel = h_cell - row
    return row_rel, col_rel

def update_global_map(global_map, local_map, robot_pose_m, height_local, width_local, resolution_global, origin_meter):
    robot_position_cell = meter_to_cell(origin_meter,
                                        (robot_pose_m[0], robot_pose_m[1]),
                                        resolution_global)
    theta = robot_pose_m[2]
    rotation_M = np.array([[cos(theta), -sin(theta)],
                           [sin(theta),  cos(theta)]])
    for i in range(height_local):
        for j in range(width_local):

            row_rel, col_rel = get_rela_ind_local(i, j)

            temp = np.array([[col_rel],[row_rel]])
            temp = np.matmul(rotation_M, temp) + np.array([[robot_position_cell[0]],[robot_position_cell[1]]])
            temp = np.reshape(temp, (2)).tolist()
            if local_map[i][j]> 50:

                global_map[int(round(temp[1]))][int(round(temp[0]))] = 100
            else:
                global_map[int(round(temp[1]))][int(round(temp[0]))] = 0
    return global_map

"""
def update_global_map(global_map, local_map, corner, width_local, height_local):

    for i in range(height_local):
        for j in range(width_local):

            if local_map[i][j] > 50:
                ratio_i = ((i+1)/height_local)
                i_start = (int(round(corner[0][0] + ratio_i*(corner[2][0]-corner[0][0]))),
                           int(round(corner[0][1] + ratio_i*(corner[2][1]-corner[0][1]))))
                i_end = (int(round(corner[1][0] + ratio_i*(corner[3][0]-corner[1][0]))),
                         int(round(corner[1][1] + ratio_i*(corner[3][1]-corner[1][1]))))
                ratio_j = (j+1)/width_local
                ind_r, ind_c = ( int(round(i_start[0] + ratio_j*(i_end[0]-i_start[0]))),
                                 int(round(i_start[1] + ratio_j*(i_end[1]-i_start[1]))))
                global_map[ind_r][ind_c] = 100
    return global_map
"""

rospy.init_node('map_global_handle', anonymous=True)


local_map_msg = rospy.wait_for_message("local_map",OccupancyGrid)


#initial global Map
width_global = 1200
height_global = 1200

data_global = np.ones((height_global,width_global), dtype = np.int8)
data_global = -1*data_global
print data_global
# global resolution should equal local resolution
"""
resolution = 0.05
width_local = 34
height_local = 80
lenth_width_local = width_local*resolution
lenth_height_local = height_local*resolution
print lenth_width_local , lenth_height_local
origin_global = [-width_global*resolution/2, -height_global*resolution/2]
"""
resolution_global = local_map_msg.info.resolution
width_local = local_map_msg.info.width
height_local = local_map_msg.info.height
lenth_width_local = width_local*resolution_global
lenth_height_local = height_local*resolution_global
origin_global = [-width_global*resolution_global/2,
                 -height_global*resolution_global/2]
local_map_data = local_map_msg.data
local_map_data = reshape_array(width_local, height_local, local_map_data)



# simulate pose for robot, you should get it from odom
# simple pose format:(x,y, theta)
fake_robot_pose1 = [1, 5, 0]
fake_robot_pose2 = [0, 0, 3.14/4]
fake_robot_pose3 = [0, 0, 3.14/4.1]
fake_robot_pose4 = [0, 0, 3.14/3.9]



data_global = update_global_map(data_global, local_map_data, fake_robot_pose1,
                                height_local, width_local, resolution_global,
                                origin_global)

data_global = update_global_map(data_global, local_map_data, fake_robot_pose2,
                                height_local, width_local, resolution_global,
                                origin_global)
"""
data_global = update_global_map(data_global, local_map_data, fake_robot_pose3,
                                height_local, width_local, resolution_global,
                                origin_global)
data_global = update_global_map(data_global, local_map_data, fake_robot_pose4,
                                height_local, width_local, resolution_global,
                                origin_global)
"""
data_global_1D = back_to_1d_array(data_global)

origin_global = Pose()
origin_global.position.x = -width_global*resolution_global/2
origin_global.position.y = -height_global*resolution_global/2
origin_global.position.z = 0

# create OccupancyGrid message
global_map = OccupancyGrid()
# header
now = rospy.get_rostime()
map_global_meta = MapMetaData()
map_global_meta.map_load_time = now

map_global_meta.resolution = resolution_global
map_global_meta.width = width_global
map_global_meta.height = height_global
map_global_meta.origin = origin_global

#set all attribute
global_map.header.seq = rospy.Time

global_map.header.stamp.secs = now.secs
global_map.header.stamp.nsecs = now.nsecs
global_map.header.frame_id = "map"
global_map.info = map_global_meta
global_map.data = data_global_1D
pub = rospy.Publisher('global_map', OccupancyGrid, queue_size=1, latch=True)
r = rospy.Rate(1) # 1hz
while not rospy.is_shutdown():
    pub.publish(global_map)
    r.sleep()
