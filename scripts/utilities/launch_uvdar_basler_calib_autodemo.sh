#!/bin/bash
DIR=$(dirname "$(readlink -f "$0")")
SERIAL=$("$DIR/basler_get_cam.sh" | tail -n 1)
mkdir -p ~/bag_files/uvdar_calib/${SERIAL}
roslaunch uvdar_core uvdar_basler_calib_setup_demo.launch camera_serial:=${SERIAL} "$@"
