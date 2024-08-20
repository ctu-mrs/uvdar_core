#!/bin/bash
tmp_file_LED_launch="/tmp/led_manager.txt"
##################### LED Configuration #######################
read -n 2 -p $'\e[1;32mDo you want to test the LEDs? [y/n]\e[0m\n' resp_led
response_led=`echo $resp_led | sed -r 's/(.*)$/\1=/'`
if [[ $response_led =~ ^(y|Y)=$ ]]; then 
    source $workspace/devel/setup.bash
    
    echo "####################### LED Configuration #######################"
    echo $'\e[0;33mLED testing works only with a battery as the power source!\e[0m'
    echo $'\e[1;32mWhich module is the UVDAR board connected to?\e[0m'
    echo "Enter:"
    echo "1 = /dev/MRS_MODULE1"
    echo "2 = /dev/MRS_MODULE2"
    echo "3 = /dev/MRS_MODULE3" 
    echo "4 = /dev/MRS_MODULE4"
    read -n 2 resp_module 
    echo "Starting with LED initialization on:/dev/MRS_MODULE$resp_module... This will take about 20 seconds."
    path_to_led_config=$(rospack find uvdar_core)/config/blinking_sequences/test_assignment.txt
    roslaunch uvdar_core led_manager.launch sequence_file:=$path_to_led_config portname:=/dev/MRS_MODULE$resp_module &> $tmp_file_LED_launch & 
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
    echo $'\e[0;33mIf the blinking pattern didn\'t change: Please shutdown the NUC, detach the battery, attach it again and call this script again!\e[0m'
else
    echo "OK. Exiting script..."
fi 
