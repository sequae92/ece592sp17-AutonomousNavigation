the run.sh has to be executed to run all of the following nodes in that order:
1) roscore
2) rosparam set use_sim_time false
3) rosrun husky_description description.launch
4) rosrun husky_modeswitch husky_modeswitch_node.py
5) rosrun husky_motorcontrol hardware.py
6) rosrun husky_python_control python_control.py
7) roslaunch urg_node urg_node.launch
8) rosrun gmapping slam_gmapping scan:=scan
9) roslaunch husky_navigation move_base.launch


Requirements:
install screen using
sudo apt-get install screen


Run joy node on laptop to control
rosrun joy joy_node
