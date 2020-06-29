#!/usr/bin/zsh
rosbag record -a -x '/uav3/bluefox2/image_raw(.*)|/uav3/uvdar/reprojection/image_raw/compressedDepth(.*)' -O with_rois_visual.bag  

