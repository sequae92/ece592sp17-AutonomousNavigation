HUSKY NAVIGATION AND CONTROL MODULE
----------------------------------

*Authors
- Aamod Velangi <avelang@ncsu.edu>
- Chun Chao Lin <clin14@ncsu.edu>
- Meghana Hada <mhada@ncsu.edu>
- Suraj Manjunath Shanbhag <smshanbh@ncsu.edu>

*Advisor
- Dr. Edgar Lobaton <ejlobato@ncsu.edu> 

*WiKi link
- https://github.ncsu.edu/smshanbh/ece592sp17-AutonomousNavigation/wiki/Control-and-Planning


*Commond:

0.  roslaunch husky_gazebo husky_modified_playpen.launch
1.  rosrun gmapping slam_gmapping scan:=/scan
2.  rosrun path_following tf_listener.py        (option, for simulation that using lidar)
3.  rosrun path_following list_points_from_rrt.py 
4.  rosrun rrt_navigation expand_obstacle_and_unknown_map.py
5.  rosrun rrt_navigation rrt_waypoints_obstacle_expand.py 
6.  rviz


