run following command in different terminals

rosrun gmapping slam_gmapping scan:=/scan
rosrun path_following tf_listener.py
rosrun rrt_navigation rrt_waypoints_new.py
rosrun path_following list_points_from_rrt.py 
rviz

#####################################################
you could send goal command from rviz
