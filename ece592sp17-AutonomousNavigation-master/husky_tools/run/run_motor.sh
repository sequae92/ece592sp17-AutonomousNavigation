path=""
set -e
echo "starting roscore"
screen -d -m -S roscore ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-roscore.sh
sleep 5
# setting parameters

echo "setting rosparam" 
rosparam set use_sim_time false
rosparam set map_topic map
rosparam set odom_topic odom
rosparam set goal_topic move_base_simple/goal
rosparam set filter_on True

echo "uploading robot description"
screen -d -m -S huskydescription ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-description.sh
sleep 5
echo "starting modeswitch"
screen -d -m -S modeswitch ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-modeswitch.sh
sleep 5
echo "starting python control"
screen -d -m -S pythoncontrol ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-pythoncontrol.sh
sleep 5

echo "ALL set to go!"
echo "Check if the comm light is green: else run :  pkill screen   and execute this script again"