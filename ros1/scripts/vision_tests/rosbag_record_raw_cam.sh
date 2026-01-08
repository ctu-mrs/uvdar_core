#!/bin/bash

path="/home/\$(optenv USER mrs)/bag_files/latest/"

# By default, we record everything.
# Except for this list of EXCLUDED topics:
exclude=(

# IN GENERAL, DON'T RECORD CAMERAS
#
# If you want to record cameras, create a copy of this script
# and place it at your tmux session.
#
# Please, seek an advice of a senior researcher of MRS about
# what can be recorded. Recording too much data can lead to
# ROS communication hiccups, which can lead to eland, failsafe
# or just a CRASH.

# Every topic containint "compressed"
'(.*)compressed(.*)'
# Every topic containint "theora"
'(.*)theora(.*)'
# Every topic containint "h264"
'(.*)h264(.*)'

'(.*)/bluefox_front/camera_info'
'(.*)/bluefox_front/image_raw'
'(.*)/bluefox_front/parameter_descriptions'
'(.*)/bluefox_front/parameter_updates'

'(.*)/blink_processor/uvdar_blink_visualization/compressed'
'(.*)/blink_processor/uvdar_blink_visualization/compressed/parameter_descriptions'
'(.*)/blink_processor/uvdar_blink_visualization/compressed/parameter_updates'
'(.*)/blink_processor/uvdar_blink_visualization/compressedDepth'
'(.*)/blink_processor/uvdar_blink_visualization/compressedDepth/parameter_descriptions'
'(.*)/blink_processor/uvdar_blink_visualization/compressedDepth/parameter_updates'
'(.*)/blink_processor/uvdar_blink_visualization/theora'
'(.*)/blink_processor/uvdar_blink_visualization/theora/parameter_descriptions'
'(.*)/blink_processor/uvdar_blink_visualization/theora/parameter_updates'
# '(.*)/uvdar_bluefox/left/camera_info'
# '(.*)/uvdar_bluefox/left/image_raw'
'(.*)/uvdar_bluefox/left/image_raw/compressed'
'(.*)/uvdar_bluefox/left/image_raw/compressed/parameter_descriptions'
'(.*)/uvdar_bluefox/left/image_raw/compressed/parameter_updates'
'(.*)/uvdar_bluefox/left/image_raw/compressedDepth'
'(.*)/uvdar_bluefox/left/image_raw/compressedDepth/parameter_descriptions'
'(.*)/uvdar_bluefox/left/image_raw/compressedDepth/parameter_updates'
'(.*)/uvdar_bluefox/left/image_raw/theora'
'(.*)/uvdar_bluefox/left/image_raw/theora/parameter_descriptions'
'(.*)/uvdar_bluefox/left/image_raw/theora/parameter_updates'
'(.*)/uvdar_bluefox/right/camera_info'
'(.*)/uvdar_bluefox/right/image_raw'
'(.*)/uvdar_bluefox/right/image_raw/compressed'
'(.*)/uvdar_bluefox/right/image_raw/compressed/parameter_descriptions'
'(.*)/uvdar_bluefox/right/image_raw/compressed/parameter_updates'
'(.*)/uvdar_bluefox/right/image_raw/compressedDepth'
'(.*)/uvdar_bluefox/right/image_raw/compressedDepth/parameter_descriptions'
'(.*)/uvdar_bluefox/right/image_raw/compressedDepth/parameter_updates'
'(.*)/uvdar_bluefox/right/image_raw/theora'
'(.*)/uvdar_bluefox/right/image_raw/theora/parameter_descriptions'
'(.*)/uvdar_bluefox/right/image_raw/theora/parameter_updates'
'(.*)/uvdar_bluefox/back/camera_info'
'(.*)/uvdar_bluefox/back/image_raw'
'(.*)/uvdar_bluefox/back/image_raw/compressed'
'(.*)/uvdar_bluefox/back/image_raw/compressed/parameter_descriptions'
'(.*)/uvdar_bluefox/back/image_raw/compressed/parameter_updates'
'(.*)/uvdar_bluefox/back/image_raw/compressedDepth'
'(.*)/uvdar_bluefox/back/image_raw/compressedDepth/parameter_descriptions'
'(.*)/uvdar_bluefox/back/image_raw/compressedDepth/parameter_updates'
'(.*)/uvdar_bluefox/back/image_raw/theora'
'(.*)/uvdar_bluefox/back/image_raw/theora/parameter_descriptions'
'(.*)/uvdar_bluefox/back/image_raw/theora/parameter_updates'
)

# file's header
filename=`mktemp`
echo "<launch>" > "$filename"
echo "<arg name=\"UAV_NAME\" default=\"\$(env UAV_NAME)\" />" >> "$filename"
echo "<group ns=\"\$(arg UAV_NAME)\">" >> "$filename"

echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" args=\"-o $path -a" >> "$filename"

# if there is anything to exclude
if [ "${#exclude[*]}" -gt 0 ]; then

  echo -n " -x " >> "$filename"

  # list all the string and separate the with |
  for ((i=0; i < ${#exclude[*]}; i++));
  do
    echo -n "${exclude[$i]}" >> "$filename"
    if [ "$i" -lt "$( expr ${#exclude[*]} - 1)" ]; then
      echo -n "|" >> "$filename"
    fi
  done

fi

echo "\" />" >> "$filename"

# file's footer
echo "</group>" >> "$filename"
echo "</launch>" >> "$filename"

cat $filename
roslaunch $filename
