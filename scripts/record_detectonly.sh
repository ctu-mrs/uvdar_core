
#!/usr/bin/zsh
# rosbag record -a -x '/uav3/uvdar/filtered(.*)|/uav3/uvdar/measured(.*)' -O filtered_nocovar.bag
rosbag filter $1 filtered_detectonly.bag 'not ("/uav42/uvdar/blink_visualization/image_raw/compressed" in topic) and not ("/uav42/uvdar/blinkersSeen" in topic) and not ("/uav42/uvdar/filtered" in topic) and not ("/uav42/uvdar/measured" in topic) and (topic != "/tf" or topic == "/tf"  and m.transforms[0].child_frame_id != "/color_camera")'
