#!/bin/bash

printout=`timeout 10s roslaunch uvdar_core cam_only_basler.launch`
echo "$printout" | grep Found | grep -oP -m1 'Serial\sNumber\s+\K\S+(?=:)'
