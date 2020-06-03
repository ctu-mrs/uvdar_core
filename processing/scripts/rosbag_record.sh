#!/bin/bash

path="/home/\$(optenv USER mrs)/bag_files/latest/"

exclude=(
# # Every topic containint "compressed"
# '(.*)compressed(.*)'
# # Every topic containint "image_raw"
# '(.*)image_raw(.*)'
# # Every topic containint "theora"
# '(.*)theora(.*)'
# # Every topic containint "h264"
# '(.*)h264(.*)'

"(.*)mobius/image_raw/compressed/(.*)"
"(.*)mobius/image_raw/compressedDepth(.*)"
"(.*)mobius/image_raw/theora(.*)"

"(.*)bluefox(.*)left/image_raw"
"(.*)bluefox(.*)left/image_raw/h264/(.*)"
"(.*)bluefox(.*)left/image_raw/h264"
"(.*)bluefox(.*)left/image_raw/compressed/(.*)"
"(.*)bluefox(.*)left/image_raw/compressedDepth(.*)"
"(.*)bluefox(.*)left/image_raw/theora(.*)"

"(.*)bluefox(.*)right/image_raw"
"(.*)bluefox(.*)right/image_raw/h264/(.*)"
"(.*)bluefox(.*)right/image_raw/h264"
"(.*)bluefox(.*)right/image_raw/compressed/(.*)"
"(.*)bluefox(.*)right/image_raw/compressedDepth(.*)"
"(.*)bluefox(.*)right/image_raw/theora(.*)"

"(.*)bluefox_of/image_raw"
"(.*)bluefox_of/image_raw/compressed/(.*)"
"(.*)bluefox_of/image_raw/compressed"
"(.*)bluefox_of/image_raw/compressedDepth(.*)"
"(.*)bluefox_of/image_raw/theora(.*)"

"(.*)uvdar_bluefox(.*)left/image_raw"
"(.*)uvdar_bluefox(.*)left/image_raw/h264/(.*)"
"(.*)uvdar_bluefox(.*)left/image_raw/h264"
"(.*)uvdar_bluefox(.*)left/image_raw/compressed"
"(.*)uvdar_bluefox(.*)left/image_raw/compressed/(.*)"
"(.*)uvdar_bluefox(.*)left/image_raw/compressedDepth(.*)"
"(.*)uvdar_bluefox(.*)left/image_raw/theora(.*)"

"(.*)uvdar_bluefox(.*)right/image_raw"
"(.*)uvdar_bluefox(.*)right/image_raw/h264/(.*)"
"(.*)uvdar_bluefox(.*)right/image_raw/h264"
"(.*)uvdar_bluefox(.*)right/image_raw/compressed"
"(.*)uvdar_bluefox(.*)right/image_raw/compressed/(.*)"
"(.*)uvdar_bluefox(.*)right/image_raw/compressedDepth(.*)"
"(.*)uvdar_bluefox(.*)right/image_raw/theora(.*)"

"(.*)uvdar/blink_visualization/image_raw"
"(.*)uvdar/blink_visualization/image_raw/compressed/(.*)"
"(.*)uvdar/blink_visualization/image_raw/compressedDepth(.*)"
"(.*)uvdar/blink_visualization/image_raw/theora(.*)"
"(.*)uvdar/blink_visualization/image_raw/h264/(.*)"
"(.*)uvdar/blink_visualization/image_raw/h264"
"(.*)uvdar/reprojection/image_raw"
"(.*)uvdar/reprojection/image_raw/compressed/(.*)"
"(.*)uvdar/reprojection/image_raw/compressedDepth(.*)"
"(.*)uvdar/reprojection/image_raw/theora(.*)"

"(.*)rs_t265/fisheye(.*)"
"(.*)rs_d435(.*)depth_to_infra(.*)"
"(.*)rs_d435(.*)depth_to_color/image_raw"
"(.*)rs_d435(.*)depth_to_color(.*)compressed"
"(.*)rs_d435(.*)depth_to_color(.*)compressed/(.*)"
"(.*)rs_d435(.*)color/image_raw"
"(.*)rs_d435(.*)/depth/(.*)"
"(.*)rs_d435(.*)/infra(.*)"
"(.*)rs_d435(.*)/color/image_rect_color"

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
