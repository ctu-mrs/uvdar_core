#!/usr/bin/zsh
rosbag record -a -x '/uav3/bluefox2/image_raw(.*)|/uav3/uvdar/reprojection/image_raw/compressedDepth(.*)|/uav3/uvdar/blink_visualization/image_raw/compressedDepth(.*)' -O with_rois_visual_new.bag  

