#!/bin/bash

cleanup(){
  pkill -f rosbag
  pkill -f roslaunch
}

trap cleanup INT

devnum=`rosrun bluefox2 bluefox2_list_cameras | tail -n1 | cut -d" " -f 3 | cut -d"," -f1`
mkdir ~/bag_files
mkdir ~/bag_files/uvdar_calib/
mkdir ~/bag_files/uvdar_calib/${devnum}
roslaunch bluefox2 single_nodelet.launch aec:=false agc:=false expose_us:=50 camera:=cam device:=${devnum} & \
  rosbag record -a -x '(.*)compressed(.*)' -O ~/bag_files/uvdar_calib/${devnum}/C_${devnum}.bag

