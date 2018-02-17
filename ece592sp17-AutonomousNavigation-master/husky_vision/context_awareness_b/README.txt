Module Name: CONTEXT AWARENESS TEAM-B

Authors: Spondon Kundu(skundu@ncsu.edu)
         Xilai Li (xli47@ncsu.edu)
	 Shamim Samadi(ssamadi@ncsu.edu)
	 Harish Pullagurla(hpullag@ncsu.edu)

Advisor: Dr. Tianfu Wu

Steps to follow while running our module on the Jetson

Step 1: cd ~/catkin_ws/src/ENet-Training/visualize

Step 2:./demo_sim.sh (This runs the ENet model and waits for input image feed)

Step 3: In another terminal - rosrun store_images store_images.py (This runs the python script which synchronizes received odom with the image and then it is passed to the model for ground plane segmentation.

Step 4: rostopic echo /enet/vector - To check the Groundplane segmentation

Step 5: rostopic echo /enet/odom - To check if odom is being sent to the next module for generating Occupancy Grid

Any topic changes have to be done in store_images.py
