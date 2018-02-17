#!/bin/bash
source /opt/ros/kinetic/setup.sh
rosparam set use_sim_time false
rosparam set odom_topic odom
rosparam set map_update_interval 5
rosparam set maxUrange 5
rosparam set maxRange 5.6
#rosparam set xmin -25
#rosparam set xmax 25
#rosparam set ymin -25
#rosparam set ymax 25
