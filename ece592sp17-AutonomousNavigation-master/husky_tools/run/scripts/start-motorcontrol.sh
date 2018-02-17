#!/bin/bash
source /opt/ros/kinetic/setup.sh
source ~/catkin_ws/devel/setup.bash 
rosrun husky_motorcontrol hardware.py
