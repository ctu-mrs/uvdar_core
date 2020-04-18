#!/usr/bin/zsh
rosbag record -a -x '/uav42/bluefox2/image_raw(.*)|/uav42/reprojection/image_raw(.*)|/uav42/reprojection/image_raw/h264(.*)|/uav42/uvdar/reprojection/image_raw/compressedDepth(.*)|/uav42/uvdar/blink_visualization/image_raw/compressedDepth(.*)' -O with_rois_visual_new.bag  

