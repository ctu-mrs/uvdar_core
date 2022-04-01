#!/bin/bash

cleanup(){
  pkill -f rqt_image_view
  pkill -f roslaunch
}

trap cleanup INT

devnum=`rosrun bluefox2 bluefox2_list_cameras | tail -n1 | cut -d" " -f 3 | cut -d"," -f1`
roslaunch bluefox2 single_nodelet.launch fps:=60 aec:=true agc:=true camera:=cam device:=${devnum} & \
  rqt_image_view /cam/image_raw
