Requirements:  
1) install screen using   :sudo apt-get install screen   
2) connect microsoft xbox 360 controller to the laptop/jetson  
3) Setup ROS_IP and ROS_MASTER_URI  

For control in real world  
1) On jetson navigate to husky_tools/ and execute run.sh  
2) on Laptop run   
    rosrun joy joy_node  

For simulation   
1) execute husky_tools/setup/setup_gazebo.sh  
2) execute husky_tools/run_simulation.sh  

