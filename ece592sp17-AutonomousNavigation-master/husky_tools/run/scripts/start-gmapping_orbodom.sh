#!/bin/bash
source /opt/ros/kinetic/setup.sh
source ~/catkin_ws/devel/setup.bash 
source ~/.bashrc
#rosrun gmapping slam_gmapping scan:=/scan
roslaunch gmapping gmapping_orbodom.launch
