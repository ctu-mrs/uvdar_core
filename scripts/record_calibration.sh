#!/bin/bash

cleanup(){
  pkill -f rosbag
  pkill -f roslaunch
}

trap cleanup INT

devnum=`rosrun bluefox2 bluefox2_list_cameras | tail -n1 | cut -d" " -f 3 | cut -d"," -f1`
roslaunch bluefox2 single_nodelet.launch aec:=false agc:=false expose_us:=100 camera:=cam device:=${devnum} & \
  rosbag record -a -x '(.*)compressed(.*)' -O ~/bag_files/uvdar_calib_tii/C_${devnum}.bag

