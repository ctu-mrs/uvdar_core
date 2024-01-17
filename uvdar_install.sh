#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# default workspace location
workspace=/home/$USER/workspace
# default git folder
GIT_PATH="/home/$USER/git"

default=y
while true; do
    [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mInstall UVDAR on real UAV? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
    response=`echo $resp | sed -r 's/(.*)$/\1=/'`

    if [[ $response =~ ^(y|Y)=$ ]]
    then
        
        # Bluefox driver
        echo "Installing Bluefox drivers:"
        echo $GIT_PATH
        cd $GIT_PATH
        if [ -d "camera_base" ]; then
            echo "camera_base already cloned"
        else
            git clone https://github.com/ctu-mrs/camera_base.git
        fi
        if [ -d "bluefox2" ]; then
            echo "bluefox2 already cloned"
        else 
            git clone https://github.com/ctu-mrs/bluefox2.git
            cd bluefox2/install
            sudo ./install.sh
        fi
        
        cd /home/$USER

        # build workspace
        if [ -d "$workspace" ]; then
            if [ -d "$workspace/src/camera_base" ]; then echo "camera_base package already exists in workspace"
            else 
                ln -s $GIT_PATH/camera_base $workspace/src/camera_base 
            fi
            if [ -d "$workspace/src/bluefox2" ]; then echo "bluefox2 package already exists in workspace"
            else 
                ln -s $GIT_PATH/bluefox2 $workspace/src/bluefox2 
            fi
            cd $workspace
            catkin build
            source $workspace/devel/setup.bash

        else 
            echo "Workspace at location: $workspace does not exist. Please ensure you select the right workspace or setup one at your wanted location"
            exit 1
        fi

        echo "+++++++ Camera Configuration +++++++"
        echo "How many cameras are on the drone? Enter 2 or 3:"
        read n_cams
        if [[ $n_cams -eq 2 ]]; then
            echo "Two cams selected!"
            echo "Please connect the left cam to NUC and UNPLUG the right cam! Then hit any key."
            read -n 1 key
            rosrun bluefox2 bluefox2_list_cameras 
            id_left_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
            echo -e "\nNow please connect the right cam to NUC and UNPLUG the left cam! Then hit any key."
            read -n 1 key
            id_right_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
            echo -e "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
            echo "Left cam ID: $id_left_cam"
            echo "Right cam ID: $id_right_cam"
            echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
            echo "Now please connect all cameras in."
        elif [[ $n_cams -eq 3 ]]; then
            echo "Three cams selected!"
            echo "Please connect the left cam to NUC and UNPLUG the other two cams! Then hit any key."
            read -n 1 key
            rosrun bluefox2 bluefox2_list_cameras
            id_left_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
            echo -e "\nNow please connect the right cam to NUC and UNPLUG the other two cams! Then hit any key."
            read -n 1 key
            id_right_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
            echo -e "\nFinally please connect the back cam to NUC and UNPLUG the other two cams! Then hit any key."
            read -n 1 key
            id_back_cam=$(rosrun bluefox2 bluefox2_list_cameras | sed -n -E -e 's/.*Serial: ([0-9]+).*/\1/p')
            echo -e "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
            echo "Left cam ID: $id_left_cam"
            echo "Right cam ID: $id_right_cam"
            echo "Back cam ID: $id_back_cam" 
            echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
            # write to .bashrc and check camera functionlaity
        else
            echo "Only valid options: 2 or 3. retun.."; exit 1
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