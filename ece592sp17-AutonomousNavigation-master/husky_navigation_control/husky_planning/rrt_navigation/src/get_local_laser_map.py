#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


def callback(msg):
    print msg.angle_min

class LaserMapLocal():
    def __init__(self):
        laser_info = rospy.wait_for_message("/scan", LaserScan)
        self.angle_min = laser_info.angle_min
        self.angle_max = laser_info.angle_max
        self.angle_increment = laser_info.angle_increment
        self.range_min = laser_info.range_min
        self.range_max = laser_info.range_max






def main():
    rospy.init_node('laser_handler', anonymous=True)




    laser_msg = rospy.wait_for_message("/scan", LaserScan)


    print laser_msg.angle_increment
    print laser_msg.range_min, laser_msg.range_max

    print len(laser_msg.ranges)
    print (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment


    rospy.spin()

if __name__ == '__main__':
    main()

"""
header:
  seq: 243
  stamp:
    secs: 1493580758
    nsecs: 367045596
  frame_id: base_laser
angle_min: -2.35619449615
angle_max: 2.09234976768
angle_increment: 0.00613592332229
time_increment: 9.76562732831e-05
scan_time: 0.10000000149
range_min: 0.019999999553
range_max: 5.59999990463
ranges: <array type: float32, length: 726>
intensities: <array type: float32, length: 0>
"""







"""
def back_to_1d_array(array):
    height = len(array)
    width = len(array[0])
    temp = np.reshape(array,(height*width))
    temp.astype(int)

    return temp.tolist()

"""
"""
how to import marker index?
now copy directly
"""
"""
# remove one [] and dtype=float32
source_M_cm = np.array(
       [[ -68.03807831,  207.68032837],
        [ -67.38652802,  206.47531128],
        [ -66.7419281 ,  205.28315735],
        [ -66.10417938,  204.10366821],
        [ -65.15646362,  201.76982117],
        [ -64.84877014,  201.78187561],
        [ -64.23090363,  200.63916016],
        [ -63.61946869,  199.50834656],
        [ -63.01435089,  198.38922119],
        [ -62.71110535,  198.40061951],
        [ -62.40777588,  198.41203308],
        [ -62.10436249,  198.42344666],
        [ -61.80086899,  198.43487549],
        [ -61.49729538,  198.44628906],
        [ -61.19363785,  198.4577179 ],
        [ -60.88990021,  198.46914673],
        [ -60.58607864,  198.48057556],
        [ -60.28217316,  198.49201965],
        [ -59.97818756,  198.50344849],
        [ -59.9570961 ,  199.647995  ],
        [ -59.65135956,  199.65965271],
        [ -59.34553909,  199.67132568],
        [ -59.0396347 ,  199.6829834 ],
        [ -58.73365021,  199.69465637],
        [ -58.70557022,  200.85270691],
        [ -58.39780807,  200.86459351],
        [ -58.36771011,  202.03570557],
        [ -58.05815506,  202.0478363 ],
        [ -22.97936058,  234.90959167],
        [ -22.6134491 ,  234.92895508],
        [ -22.24741936,  234.94833374],
        [ -21.88127518,  234.96772766],
        [ -21.51501083,  234.98710632],
        [ -21.14863205,  235.0065155 ],
        [ -20.7821331 ,  235.02590942],
        [ -20.41551781,  235.0453186 ],
        [ -20.04878616,  235.06472778],
        [ -19.68193626,  235.08415222],
        [  83.87342834,  237.32377625],
        [  82.52826691,  232.54249573],
        [  81.23319244,  227.93928528],
        [  80.52158356,  224.97038269],
        [  70.40499115,  196.41648865],
        [  69.48990631,  193.05291748],
        [  68.60353088,  189.79486084],
        [  68.52574921,  188.73809814],
        [  68.84311676,  188.74809265],
        [  68.76449585,  187.70188904],
        [  69.08032227,  187.71170044],
        [  69.39622498,  187.72151184],
        [  69.71221161,  187.73132324],
        [  70.02828979,  187.74113464],
        [  70.34444427,  187.75094604],
        [  70.25856781,  186.71440125],
        [  70.57319641,  186.72402954],
        [  70.88790894,  186.7336731 ],
        [  71.20270538,  186.74330139],
        [  71.51758575,  186.75294495],
        [  71.83254242,  186.7625885 ],
        [  72.14759064,  186.77223206],
        [  72.46271515,  186.78187561],
        [  72.77792358,  186.79151917],
        [  73.09322357,  186.80117798],
        [  73.40859985,  186.81082153],
        [  73.72406006,  186.82048035],
        [  74.03960419,  186.83013916],
        [  74.35523224,  186.83979797],
        [  74.24939728,  185.81132507],
        [  74.56350708,  185.82080078],
        [  74.45713806,  184.80245972],
        [  74.76973724,  184.81176758],
        [  75.08241272,  184.82106018],
        [  75.39517212,  184.83036804],
        [  75.28514099,  183.82168579],
        [  75.59640503,  183.83081055],
        [  75.90774536,  183.83995056],
        [  76.21916962,  183.84907532],
        [  76.10554504,  182.84989929],
        [  76.41548157,  182.85887146],
        [  76.72550201,  182.86782837],
        [  77.03560638,  182.87680054],
        [  77.34578705,  182.88577271],
        [  77.65605164,  182.89474487],
        [  77.53578949,  181.90460205],
        [  77.8445816 ,  181.91339111],
        [  78.15345001,  181.92219543],
        [  78.46240234,  181.93099976],
        [  78.77143097,  181.93981934],
        [  79.08053589,  181.94862366],
        [  78.95376587,  180.96739197],
        [  79.26141357,  180.9760437 ],
        [  79.56913757,  180.98468018],
        [  79.8769455 ,  180.99333191],
        [  80.18482971,  181.00198364],
        [  80.49279785,  181.01063538],
        [  80.80084229,  181.01928711],
        [  81.10896301,  181.02793884],
        [  81.41716766,  181.03660583],
        [  81.72544861,  181.04525757],
        [  82.03381348,  181.05392456],
        [  82.34225464,  181.06259155],
        [  82.65077209,  181.07125854],
        [  82.95937347,  181.07992554],
        [  83.26805115,  181.08859253],
        [  83.57681274,  181.09727478],
        [  83.88565063,  181.10594177],
        [  84.19457245,  181.11462402],
        [  84.50357056,  181.12330627],
        [  84.81265259,  181.13198853],
        [  85.12181091,  181.14067078],
        [  85.43104553,  181.14936829],
        [  85.74036407,  181.15805054],
        [  86.04976654,  181.16674805],
        [  86.3592453 ,  181.1754303 ],
        [  86.66880035,  181.18412781],
        [  86.97843933,  181.19282532],
        [  87.28816223,  181.20153809],
        [  87.59796143,  181.2102356 ],
        [  88.39118195,  182.21398926],
        [  88.70278931,  182.22288513],
        [  89.01447296,  182.23176575],
        [  89.32624054,  182.24064636],
        [  89.63809204,  182.24954224],
        [  89.95001984,  182.25842285],
        [  90.26203156,  182.26731873],
        [  90.5741272 ,  182.2762146 ],
        [  90.88629913,  182.28511047],
        [  91.19855499,  182.29400635],
        [  91.51088715,  182.30291748],
        [  91.82330322,  182.31181335],
        [  92.13580322,  182.32072449],
        [  92.44837952,  182.32963562],
        [  92.76103973,  182.33854675],
        [  93.07378387,  182.34745789],
        [  93.38660431,  182.35638428],
        [  93.6995163 ,  182.36529541],
        [  94.01249695,  182.3742218 ],
        [  94.32556915,  182.38313293],
        [  94.12228394,  181.39352417],
        [  94.43385315,  181.40228271],
        [  94.74549866,  181.411026  ]])


#convert to meter
source_M_m = source_M_cm/100

rospy.init_node('map_local_handle', anonymous=True)


local_map_x_left = source_M_m[0][0]
local_map_x_right = source_M_m[-1][0]
local_map_y = 4 # 4 meter user defined

resolution_local = 0.05 # a cell is 0.05m x 0.05m
width_local = int(round((local_map_x_right - local_map_x_left)/resolution_local))
height_local = int(round(local_map_y/resolution_local))
print width_local, height_local

# initial the local map set all cells to zeros
data_local = np.zeros((height_local,width_local), dtype = np.int8)


#convert index from meter to cell
index_cell = []
ind_x_cell = []
ind_y_cell = []
for pair in source_M_m:

    ind_x = int(round((pair[0]-local_map_x_left)/resolution_local))-1
    ind_y = int(round((local_map_y-pair[1])/resolution_local))-1
    index_cell.append([ind_x, ind_y])

print index_cell

# update the local map
for pair in index_cell:
    ind_row = pair[1]
    ind_col = pair[0]
    data_local[ind_row][ind_col]= 100

data_local_1D =  back_to_1d_array(data_local)



origin_local = Pose()
origin_local.position.x = local_map_x_left
origin_local.position.y = 0
origin_local.position.z = 0

# create OccupancyGrid message
local_map = OccupancyGrid()
# header
now = rospy.get_rostime()
map_local_meta = MapMetaData()
map_local_meta.map_load_time = now

map_local_meta.resolution = resolution_local
map_local_meta.width = width_local
map_local_meta.height = height_local
map_local_meta.origin = origin_local

#set all attribute
local_map.header.seq = rospy.Time

local_map.header.stamp.secs = now.secs
local_map.header.stamp.nsecs = now.nsecs
local_map.header.frame_id = "map"
local_map.info = map_local_meta
local_map.data = data_local_1D
pub = rospy.Publisher('local_map', OccupancyGrid, queue_size=1, latch=True)
r = rospy.Rate(1) # 1hz
while not rospy.is_shutdown():
    pub.publish(local_map)
    r.sleep()
"""




"""
map_info(map_metadata)
######################
map_load_time:
  secs: 0
  nsecs:         0
resolution: 0.0500000007451
width: 4000
height: 4000
origin:
  position:
    x: -100.0
    y: -100.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0

"""
