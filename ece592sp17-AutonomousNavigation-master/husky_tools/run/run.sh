path=""
set -e
echo "starting roscore"
screen -d -m -S roscore ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-roscore.sh
sleep 5
echo "setting rosparam" 
screen -d -m -S rosparameters ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/set-param.sh
sleep 5
echo "uploading robot description"
screen -d -m -S huskydescription ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-description.sh
sleep 5
echo "starting modeswitch"
screen -d -m -S modeswitch ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-modeswitch.sh
sleep 5
#echo "starting motorcontrol"
#screen -d -m -S motorcontrol ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-motorcontrol.sh
#sleep 5
echo "starting python control"
screen -d -m -S pythoncontrol ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-pythoncontrol.sh
sleep 5

echo "start lidar"
screen -d -m -S urgnode ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-urgnode.sh
sleep 5

echo "start gmapping"
screen -d -m -S gmapping ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-gmapping.sh
sleep 5

if [ "$1" == "-movebase" ];then
    echo "start movebase"
    screen -d -m -S movebase ~/catkin_ws/src/ece592sp17-AutonomousNavigation/husky_tools/run/scripts/start-movebase.sh
    sleep 5
fi

echo "ALL set to go!"
echo "Check if the comm light is green: else run :  pkill screen   and execute this script again"
