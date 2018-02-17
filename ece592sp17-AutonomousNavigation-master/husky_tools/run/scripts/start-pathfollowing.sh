#!/bin/bash
source /opt/ros/kinetic/setup.sh
source ~/catkin_ws/devel/setup.bash 
rosrun path_following list_points_from_rrt.py
