#!/bin/bash

set -e


################### USER PARAMETER ################### 
# default workspace location
workspace=/home/$USER/workspace
# default git folder
GIT_PATH="/home/$USER/git"

# temporary file names
tmp_file_LED_launch="/tmp/led_manager.txt"

#error handling 
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR
trap 'rm -f $tmp_file_LED_launch exit 1' SIGINT

###### Some helper funcitons ######
workspace_not_existent(){
    echo $'\e[0;31m\nFolder does not exist!\e[0m'
    echo "Please ensure you have the right workspace selected and the folder:" $'\e[0;33m'$workspace''$'\e[0;33m/src\e[0m' "exists."
    exit 1
}

# extract ID for two bluefox cameras
extract_id_two_cams(){
    echo $'\e[1;32mPlease connect the left cam to NUC and UNPLUG the right cam! Wait approximately 5 sec. Then hit any key.\e[0m'
    read -n 1 key
    rosrun bluefox2 bluefox2_list_cameras 
    id_left_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
    echo -e $'\e[1;32m\nNow please connect the right cam to NUC and UNPLUG the left cam! Wait approximately 5 sec. Then hit any key.\e[0m'
    read -n 1 key
    id_right_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
}


echo "Cloning UVDAR repository into $GIT_PATH folder:"
cd $GIT_PATH
if [ -d "uvdar_core" ]; then
    echo "uvdar_core already cloned"
else
    git clone https://github.com/TimLakemann/uvdar_core.git
    sudo apt install libgbm-dev
fi                                

# clone uvdar_core + symlink to workspace
if [[ -d "$workspace/src" ]]
then
    if [ -d "$workspace/src/uvdar_core" ]; then echo "uvdar_core package already exists in workspace. Skipping..."
    else 
        ln -s $GIT_PATH/uvdar_core $workspace/src/uvdar_core
    fi
else 
    workspace_not_existent
fi

default=y
while true; do
    [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mInstall UVDAR on real UAV? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
    response=`echo $resp | sed -r 's/(.*)$/\1=/'`

    if [[ $response =~ ^(y|Y)=$ ]]
    then 
        # Bluefox driver
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
        
        cd $workspace && cd ..
        # build workspace
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
#            catkin build
            source $workspace/devel/setup.bash

        else 
            workspace_not_existent
        fi

        read -n 2 -p $'\e[1;32mDo you want to configure/test the cameras? [y/n]\e[0m\n' resp_cam
        response_cam=`echo $resp_cam | sed -r 's/(.*)$/\1=/'`

        if [[ $response_cam =~ ^(y|Y)=$ ]]
        then 
            echo "####################### Camera Configuration #######################"
            read -p $'\e[1;32mHow many cameras are on the drone? [2/3] \e[0m\n' n_cams
            #read n_cams
            if [[ $n_cams -eq 2 ]]; then
                echo "Two cams selected!"
                test_two_cams
                echo -e "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
                echo "Left  cam ID: $id_left_cam"
                echo "Right cam ID: $id_right_cam"
                echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
                echo $'\e[1;32mNow please connect all cameras. Then press any key..\e[0m'
                read -n 1 key
                echo "Left Cam:"
                roslaunch bluefox2 single_nodelet.launch expose_us:=1000 aec:=false fps:=200 device:=$id_left_cam > /dev/null  & pid_roslaunch=$! & rostopic hz /mv_$id_left_cam/image_raw & pid_topic=$!
                sleep 15
                kill $pid_roslaunch && kill $pid_topic
            elif [[ $n_cams -eq 3 ]]; then
                echo "Three cams selected!"
                test_two_cams
                echo -e $'\e[1;32m\nFinally please connect the back cam to NUC and UNPLUG the other two cams! Wait approximately 5 sec. Then hit any key.\e[0m'
                read -n 1 key
                id_back_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
                echo -e "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
                echo "Left  cam ID: $id_left_cam"
                echo "Right cam ID: $id_right_cam"
                echo "Back  cam ID: $id_back_cam" 
                echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
                # write to .bashrc and check camera functionlaity
                echo "####################### Camera Configuration done! #######################"
            else
                echo "Only valid options: 2 or 3. retun.."; exit 1
            fi
        fi 
        echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
        read -n 2 -p $'\e[1;32mDo you want to test the LEDs? [y/n]\e[0m\n' resp_led
        response_led=`echo $resp_led | sed -r 's/(.*)$/\1=/'`

        if [[ $response_led =~ ^(y|Y)=$ ]]
        then 
            source $workspace/devel/setup.bash
            
            echo "####################### LED Configuration #######################"
            echo $'\e[1;32mWhich module is the UVDAR board connected to?\e[0m'
            echo "Enter:"
            echo "1 = /dev/MRS_MODULE1"
            echo "2 = /dev/MRS_MODULE2"
            echo "3 = /dev/MRS_MODULE3"
            read -n 2 resp_module 
            echo "Starting with LED initialization on:/dev/MRS_MODULE$resp_module... This will take about 20 seconds."
            roslaunch uvdar_core led_manager.launch portname:=/dev/MRS_MODULE$resp_module &> $tmp_file_LED_launch & 
            pid_led_manager=$! 
            sleep 5; rosservice call /$UAV_NAME/uvdar_led_manager_node/quick_start 0
            sleep 2; rosservice call /$UAV_NAME/uvdar_led_manager_node/load_sequences
            sleep 2; rosservice call /$UAV_NAME/uvdar_led_manager_node/select_sequences [0,1,2,3]
            sleep 2; rosservice call /$UAV_NAME/uvdar_led_manager_node/set_frequency 1
            sleep 5
           
            # kill the LED manager and the remove temporary file
            kill -9 "$pid_led_manager"
            rm $tmp_file_LED_launch
            echo "####################### LED Configuration done! #######################"
            echo "Please verify that the LEDs are correctly wired!"
            echo "Blinking frequency alignment:"
            echo "Left front  = Constant on"
            echo "Right front = slow blinking pattern"
            echo "Left back   = moderate blinking pattern"
            echo "Right back  = fastest blinking patter"
        else
            echo "Exiting script..."
        fi 
       
        exit 0

    elif [[ $response =~ ^(n|N)=$ ]]
    then
        while true; do
            simulator=y
            [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mInstall the UVDAR-Simulator? [y/n] (default: '"$simulator"$')\e[0m\n' resp1 || resp1=$simulator; }
            response1=`echo $resp1 | sed -r 's/(.*)$/\1=/'`
            if [[ $response1 =~ ^(y|Y)=$ ]]
            then
                echo "...to be done soon"
                exit 0 
            elif [[ $response1 =~ ^(n|N)=$ ]]
            then 
                echo "Exiting  script."
                exit 0
            else
                echo " What? \"$resp\" is not a correct answer. Try y+Enter."
            fi
        done
    else
    echo " What? \"$resp\" is not a correct answer. Try y+Enter."
    fi

done
