echo -e "\033[32m ----------Creating Workspace----------\033[0m"
mkdir ~/Documents/husky_kinetic
cd ~/Documents/husky_kinetic
wget https://raw.githubusercontent.com/CSUN-SERL/ISLURPDevguide/master/kinetic-husky-wet.rosinstall

echo -e "\033[32m ----------Downloading Source Code----------\033[0m"
# Get the source code
wstool init src kinetic-husky-wet.rosinstall

echo -e "\033[32m ----------Installing Dependencies----------\033[0m"
# Install Dependencies (ignore errors)
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r

# Setup the catkin workspace
cd src
catkin_init_workspace
cd ..
catkin_make

rm -rf ~/Documents/husky_kinetic
