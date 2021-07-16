#!/bin/bash

cleanup(){
  pkill -f rosbag
  pkill -f roslaunch
}

trap cleanup INT

devnum=`rosrun bluefox2 bluefox2_list_cameras | tail -n1 | cut -d" " -f 3 | cut -d"," -f1`
roslaunch bluefox2 single_nodelet.launch fps:=60 aec:=false agc:=false expose_us:=500 camera:=cam device:=${devnum}

