#!/bin/bash
source /opt/ros/kinetic/setup.sh
source ~/catkin_ws/devel/setup.bash 
rosrun rrt_navigation expand_obstacle_and_unknown_map.py
