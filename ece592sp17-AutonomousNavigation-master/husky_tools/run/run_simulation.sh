path=""
set -e
echo "start simulation"
screen -d -m -S gazebo ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-gazebo.sh
sleep 5
echo "starting modeswitch"
screen -d -m -S modeswitch ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-modeswitch.sh
sleep 5
echo "starting motorcontrol"
screen -d -m -S motorcontrol ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-motorcontrol.sh
sleep 5
echo "starting joystick"
screen -d -m -S joy ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-joy.sh
sleep 5

echo "ALL set to go!"
echo "Check if the comm light is green: else run :  pkill screen   and execute this script again"
