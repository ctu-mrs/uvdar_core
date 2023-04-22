#!/bin/sh

cd ~/git
git clone https://github.com/caspososs/uvdar_core.git uvdar_core_tim
cd uvdar_core_tim
git checkout camp
cd ~/
mkdir -p ~/tim_l_ws/src && cd ~/tim_l_ws/src
ln -s ~/git/uvdar_core_tim
cd ~/tim_l_ws 
catkin init
catkin config --extend ~/modules_workspace/devel
catkin build
cd ~/tim_l_ws/src/uvdar_core_tim/scripts/demo
