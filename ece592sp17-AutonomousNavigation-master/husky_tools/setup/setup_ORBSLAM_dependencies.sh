#!/bin/bash

#script to install all dependencies for ORBSLAM

#install dependencies for Pangolin
sudo apt-get install libglew-dev -y
sudo apt-get install cmake -y

#install Pangolin
cd ~/
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make -j5

#install opencv separately
