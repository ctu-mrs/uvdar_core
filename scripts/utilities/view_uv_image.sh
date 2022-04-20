#!/bin/bash

cleanup(){
  pkill -f roslaunch
  pkill -f rqt_image_view
}

trap cleanup INT

devnum=`rosrun bluefox2 bluefox2_list_cameras | tail -n1 | cut -d" " -f 3 | cut -d"," -f1`
roslaunch bluefox2 single_nodelet.launch aec:=false agc:=false expose_us:=1000 camera:=cam device:=${devnum} & \
  rqt_image_view /cam/image_raw
