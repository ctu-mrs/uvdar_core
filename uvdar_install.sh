#!/bin/bash

set -e

####################### USER Parameter ########################
# default workspace location
workspace=/home/$USER/workspace
# default git folder
GIT_PATH="/home/$USER/git"
# default exposure for bluefox cams
EXPOSURE=1000
############################################################### 

clean(){
  rm -f $tmp_file_LED_launch 
  kill -9 $pid_led_manager
  rm -f $tmp_file_cam_launch 
  kill -9 $pid_cam_launch 
}

#error handling 
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR
trap 'clean exit 1' SIGINT

# camera IDs
id_left_cam=-1
id_right_cam=-1
id_back_cam=-1

# temporary file names
tmp_file_LED_launch="/tmp/led_manager.txt"
tmp_file_cam_launch="/tmp/cam_launch.txt"

#################### Some helper funcitons ####################
workspace_not_existent(){
    echo $'\e[0;31m\nFolder does not exist!\e[0m'
    echo "Please ensure you have the right workspace selected and the folder:" $'\e[0;33m'$workspace''$'\e[0;33m/src\e[0m' "exists."
    exit 1
}

# extract ID for two bluefox cameras
extract_id_two_cams(){
  echo $'\e[1;32mPlease connect the left cam to NUC and UNPLUG the other cam(s)! Wait approximately 5 sec. Then hit any key.\e[0m'
    read -n 1 key
    id_left_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
    echo -e $'\e[1;32m\nNow please connect the right cam to NUC and UNPLUG the other cam(s)! Wait approximately 5 sec. Then hit any key.\e[0m'
    read -n 1 key
    id_right_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
}

print_cam_ids_and_write_to_bash(){

    #remove previous environment variables 
    sed -i '/export BLUEFOX_UV_*/d' ~/.bashrc
    
    echo -e "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
    if [ $id_left_cam != -1 ]; then
        echo "Left  cam ID: $id_left_cam"
        echo "export BLUEFOX_UV_LEFT=$id_left_cam" >> ~/.bashrc
        echo "export BLUEFOX_UV_LEFT_EXPOSE_US=$EXPOSURE" >> ~/.bashrc
    else
        echo $'\e[0;33mCouldn\'t extract left cam ID! Please restart the camera configuration\e[0m'
    fi
    if [ $id_right_cam != -1 ]; then
        echo "Right cam ID: $id_right_cam"
        echo "export BLUEFOX_UV_RIGHT=$id_right_cam" >> ~/.bashrc
        echo "export BLUEFOX_UV_RIGHT_EXPOSE_US=$EXPOSURE" >> ~/.bashrc
    else 
        echo $'\e[0;33mCouldn\'t extract right cam ID! Please restart the camera configuration\e[0m'
    fi
    if [ $id_back_cam != -1 ] && [ $n_cams == 3 ]; then
        echo "Back  cam ID: $id_back_cam" 
        echo "export BLUEFOX_UV_BACK=$id_back_cam" >> ~/.bashrc
        echo "export BLUEFOX_UV_BACK_EXPOSE_US=$EXPOSURE" >> ~/.bashrc
    elif [ $id_back_cam == -1 ] && [ $n_cams == 3 ]; then
        echo $'\e[0;33mCouldn\'t extract back cam ID! Please restart the camera configuration\e[0m'
    fi
    echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
    echo $'\e[1;32mNow please connect all cameras. Wait approximately 5 seconds. Then press any key..\e[0m'
    read -n 1 key
}

test_cam(){
    sleep 2 
    echo $'\e[1;34m\n'$1'' $'\e[1;34,mcamera output:\e[0m\n'
    rostopic hz "/$UAV_NAME/uvdar_bluefox/$1/image_raw" & 
    pid_hz_topic=$!
    sleep 15; 
    kill $pid_hz_topic
}
###############################################################

build_workspace(){
    echo "Cloning UVDAR repository into $GIT_PATH folder:"
    cd $GIT_PATH
    if [ -d "uvdar_core" ]; then
        echo "uvdar_core already cloned"
    else
        git clone https://github.com/TimLakemann/uvdar_core.git
        sudo apt install libgbm-dev
    fi                                

    if [[ -d "$workspace/src" ]]
    then
        if [ -d "$workspace/src/uvdar_core" ]; then echo "uvdar_core package already exists in workspace. Skipping..."
        else 
            ln -s $GIT_PATH/uvdar_core $workspace/src/uvdar_core
        fi
    else 
        workspace_not_existent
    fi

    echo "Installing Bluefox drivers:"
    cd $GIT_PATH
    if [ -d "camera_base" ]; then echo "camera_base already cloned"
    else
        git clone https://github.com/ctu-mrs/camera_base.git
    fi
    if [ -d "bluefox2" ]; then echo "bluefox2 already cloned"
    else 
        git clone https://github.com/ctu-mrs/bluefox2.git
        cd bluefox2/install
        sudo ./install.sh
    fi

    if [ -d "$workspace/src" ]; then
        if [ -d "$workspace/src/camera_base" ]; then echo "camera_base package already exists in workspace"
        else 
            ln -s $GIT_PATH/camera_base $workspace/src/camera_base 
        fi
        if [ -d "$workspace/src/bluefox2" ]; then echo "bluefox2 package already exists in workspace"
        else 
            ln -s $GIT_PATH/bluefox2 $workspace/src/bluefox2 
        fi
        cd $workspace
        catkin clean -y
        catkin build
    else 
        workspace_not_existent
    fi

    source $workspace/devel/setup.bash
}

echo ""
echo $'\e[0;35m#######################################################################'
echo "##################### UVDAR Configuration script ######################"
echo $'#######################################################################\n\e[0m'


read -n 2 -p $'\e[1;32mAre you calling this script on a real drone? [y/n]\n\e[0m'  resp_uav
response_uav=`echo $resp_uav | sed -r 's/(.*)$/\1=/'`
if [[ $response_uav =~ ^(n|N)=$ ]]
then
    echo  $'\e[0;33mPlease call this script only on a real UAV\n\e[0m'
    exit 1
fi

###################### Build Workspace ########################
read -n 2 -p $'\e[1;32mDo you want to (re)-build the workspace? [y/n]\n\e[0;33m[Hint:] When calling this script for the first time, please enter y!\n\e[0m'  resp_ws
response_ws=`echo $resp_ws | sed -r 's/(.*)$/\1=/'`
if [[ $response_ws =~ ^(y|Y)=$ ]]
then
    build_workspace
fi
###############################################################


read -n 4 -p $'\e[1;32mPlease enter the MRS-ID [e.g. X01, X02, X13..]. Then hit enter.\n\e[0m'  resp_MRS_ID
echo "Adding MRS-ID to bashrc..."
#remove previous environment variables
sed -i '/export MRS_ID=*/d' ~/.bashrc
echo "export MRS_ID=$resp_MRS_ID" >> ~/.bashrc


#################### Camera Configuration #####################
read -n 2 -p $'\e[1;32mDo you want to configure/test the cameras? [y/n]\e[0m\n' resp_cam
response_cam=`echo $resp_cam | sed -r 's/(.*)$/\1=/'`
if [[ $response_cam =~ ^(y|Y)=$ ]]
then 
    echo "####################### Camera Configuration #######################"
    read -p $'\e[1;32mHow many cameras are on the drone? [2/3] \e[0m\n' n_cams
    #read n_cams
    if [[ $n_cams -eq 2 ]]; then
        echo "Two cams selected!"
        extract_id_two_cams
        print_cam_ids_and_write_to_bash
        echo "Testing cameras. One moment please..."
        roslaunch uvdar_core camera_only_two_sided.launch left:=$id_left_cam right:=$id_right_cam expose_us_left:=$EXPOSURE expose_us_right:=$EXPOSURE &> $tmp_file_cam_launch &
        sleep 10 
        pid_cam_launch=$! 
        test_cam left
        test_cam right
        
        kill $pid_cam_launch
        rm -f $tmp_file_cam_launch
        
        sleep 2
    elif [[ $n_cams -eq 3 ]]; then
        echo "Three cams selected!"
        extract_id_two_cams
        # extract id for back camera 
        echo -e $'\e[1;32m\nFinally please connect the back cam to NUC and UNPLUG the other two cams! Wait approximately 5 sec. Then hit any key.\e[0m'
        read -n 1 key
        id_back_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
        print_cam_ids_and_write_to_bash
        echo "Testing cameras. One moment please..."
        roslaunch uvdar_core camera_only_three_sided.launch &> $tmp_file_cam_launch &
        roslaunch uvdar_core camera_only_three_sided.launch left:=$id_left_cam right:=$id_right_cam back:=$id_back_cam expose_us_left:=$EXPOSURE expose_us_right:=$EXPOSURE expose_us_back:=$EXPOSURE &> $tmp_file_cam_launch &
        sleep 10 
        pid_cam_launch=$! 
        test_cam left
        test_cam right
        test_cam back
    
        kill $pid_cam_launch
        rm -f $tmp_file_cam_launch
        
        sleep 2
    else
        echo "Only valid options: 2 or 3. return.."; exit 1
    fi
    echo -e "##################### Camera Configuration done! ####################\n"
fi 
###############################################################

##################### LED Configuration #######################
read -n 2 -p $'\e[1;32mDo you want to test the LEDs? [y/n]\e[0m\n' resp_led
response_led=`echo $resp_led | sed -r 's/(.*)$/\1=/'`
if [[ $response_led =~ ^(y|Y)=$ ]]; then 
    source $workspace/devel/setup.bash
    
    echo "####################### LED Configuration #######################"
    echo $'\e[1;32mWhich module is the UVDAR board connected to?\e[0m'
    echo "Enter:"
    echo "1 = /dev/MRS_MODULE1"
    echo "2 = /dev/MRS_MODULE2"
    echo "3 = /dev/MRS_MODULE3" 
    echo "4 = /dev/MRS_MODULE4"
    read -n 2 resp_module 
    echo "Starting with LED initialization on:/dev/MRS_MODULE$resp_module... This will take about 20 seconds."
    roslaunch uvdar_core led_manager.launch sequence_file:=$GIT_PATH/uvdar_core/config/blinking_sequences/test_assignment.txt portname:=/dev/MRS_MODULE$resp_module &> $tmp_file_LED_launch & 
    pid_led_manager=$! 
    sleep 5; rosservice call /$UAV_NAME/uvdar_led_manager_node/quick_start 0
    sleep 2; rosservice call /$UAV_NAME/uvdar_led_manager_node/load_sequences
    sleep 2; rosservice call /$UAV_NAME/uvdar_led_manager_node/select_sequences [0,1,2,3]
    sleep 2; rosservice call /$UAV_NAME/uvdar_led_manager_node/set_frequency 1
    sleep 5
   
    # kill the LED manager and remove temporary file
    kill -9 "$pid_led_manager"
    rm $tmp_file_LED_launch
    echo "##################### LED Configuration done! ###################"
    echo $'\e[1;32mPlease verify that the LEDs are correctly wired!\e[0m'
    echo "Blinking Pattern: Clockwise blinking circle starting at the left front arm!"
else
    echo "OK. Exiting script..."
fi 
###############################################################

exit 0
