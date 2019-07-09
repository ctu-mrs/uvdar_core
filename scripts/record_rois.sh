#!/usr/bin/zsh
rosbag record -a -x '/uav3/bluefox2/image_raw(.*)' -O with_rois.bag  
