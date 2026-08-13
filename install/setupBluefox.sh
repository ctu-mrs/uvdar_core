#!/bin/bash
#
# configure_bluefox_cameras.sh
#
# ROS 2 port of the original ROS1 uvdar camera-configuration script.
# Detects Bluefox camera serials one at a time (left/right), writes them
# to ~/.bashrc as environment variables, then launches and tests both
# cameras together via the two_bluefox.launch.py file.
#
# Assumes:
#   - ros2 run bluefox2 bluefox2_list_cameras is on PATH and working
#     (i.e. udev permissions already fixed - see fix_mvbluefox_permissions.sh)
#   - uvdar_core package provides two_bluefox.launch.py accepting
#     device_left / device_right / expose_us_left / expose_us_right
#     (adjust ARG NAMES section below to match your actual launch args)
#   - Image topics follow the /<uav_name>/<camera_name>/bluefox/image_raw
#     convention already used in your launch files

set -e

####################### USER Parameters ########################
EXPOSURE=1000
UAV_NAME="${UAV_NAME:-uav1}"
LAUNCH_PACKAGE="uvdar_core"
LAUNCH_FILE="two_bluefox.launch.py"
TEST_DURATION_SEC=10
# Workspace containing src/uvdar_core (typically symlinked back to your git
# checkout). Using the workspace src path ties this script to whatever is
# actually built/run, rather than assuming a separate ~/git layout.
WORKSPACE="/home/$USER/ws_tim"
CALIB_DIR="$WORKSPACE/src/uvdar_core/config/camera/bluefox_ocam_calib"
##################################################################

id_left_cam=-1
id_right_cam=-1

tmp_file_cam_launch="/tmp/cam_launch_$$.txt"
pid_cam_launch=""

#################### Helper functions ####################

clean() {
    if [[ -n "$pid_cam_launch" ]]; then
        kill -9 "$pid_cam_launch" 2>/dev/null || true
    fi
    rm -f "$tmp_file_cam_launch"
}
trap clean EXIT
trap 'echo "Interrupted."; clean; exit 1' SIGINT

list_camera_serials() {
    # Adjust this parser if your installed bluefox2 build's output format differs.
    # Known-good for the ctu-mrs bluefox2 ros2_release build (space-separated serials,
    # trailing newline stripped) — matches what get_available_cameras() in your
    # existing launch file already assumes.
    ros2 run bluefox2 bluefox2_list_cameras 2>/dev/null | tr -s ' \n' ' '
}

extract_id_two_cams() {
    echo -e "\033[1;32mConnect the LEFT camera and unplug all others. Wait ~5s, then press any key.\033[0m"
    read -n 1 -r -s
    echo
    sleep 3
    id_left_cam=$(list_camera_serials | awk '{print $1}')
    if [[ -z "$id_left_cam" ]]; then
        echo -e "\033[0;31mNo camera detected for LEFT. Aborting.\033[0m"
        exit 1
    fi

    echo -e "\033[1;32mNow connect the RIGHT camera and unplug all others. Wait ~5s, then press any key.\033[0m"
    read -n 1 -r -s
    echo
    sleep 3
    id_right_cam=$(list_camera_serials | awk '{print $1}')
    if [[ -z "$id_right_cam" ]]; then
        echo -e "\033[0;31mNo camera detected for RIGHT. Aborting.\033[0m"
        exit 1
    fi

    if [[ "$id_left_cam" == "$id_right_cam" ]]; then
        echo -e "\033[0;31mLeft and right resolved to the same serial ($id_left_cam). Something went wrong.\033[0m"
        exit 1
    fi
}

update_calib_symlinks() {
    # Points bf_left.yaml / bf_right.yaml (the repo's existing fixed filenames,
    # already referenced by the detector config) at bf_uv_<serial>.yaml for
    # whichever camera is currently in each position.
    if [[ ! -d "$CALIB_DIR" ]]; then
        echo -e "\033[0;33mCalib directory not found ($CALIB_DIR) - skipping symlink update.\033[0m"
        return
    fi

    local left_target="$CALIB_DIR/bf_uv_${id_left_cam}.yaml"
    local right_target="$CALIB_DIR/bf_uv_${id_right_cam}.yaml"

    if [[ -f "$left_target" ]]; then
        ln -sf "$left_target" "$CALIB_DIR/bf_left.yaml"
        echo "bf_left.yaml -> $(basename "$left_target")"
    else
        echo -e "\033[0;33mNo calib file found for LEFT serial $id_left_cam ($left_target). Symlink not updated - camera may be uncalibrated.\033[0m"
    fi

    if [[ -f "$right_target" ]]; then
        ln -sf "$right_target" "$CALIB_DIR/bf_right.yaml"
        echo "bf_right.yaml -> $(basename "$right_target")"
    else
        echo -e "\033[0;33mNo calib file found for RIGHT serial $id_right_cam ($right_target). Symlink not updated - camera may be uncalibrated.\033[0m"
    fi

    echo -e "\033[0;33mNote: if this workspace is NOT built with --symlink-install, rebuild\033[0m"
    echo -e "\033[0;33m(colcon build --packages-select uvdar_core) so these symlink changes\033[0m"
    echo -e "\033[0;33mpropagate to the installed package used at launch time.\033[0m"
}

write_ids_to_bashrc() {
    # Remove any previous exports for these vars before rewriting
    sed -i '/export BLUEFOX_LEFT_ID=/d'      ~/.bashrc
    sed -i '/export BLUEFOX_RIGHT_ID=/d'     ~/.bashrc
    sed -i '/export EXPOSE_US_LEFT=/d'       ~/.bashrc
    sed -i '/export EXPOSE_US_RIGHT=/d'      ~/.bashrc

    {
        echo "export BLUEFOX_LEFT_ID=$id_left_cam"
        echo "export BLUEFOX_RIGHT_ID=$id_right_cam"
        echo "export EXPOSE_US_LEFT=$EXPOSURE"
        echo "export EXPOSE_US_RIGHT=$EXPOSURE"
    } >> ~/.bashrc

    echo -e "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
    echo "Left  cam ID: $id_left_cam  (exported as BLUEFOX_LEFT_ID)"
    echo "Right cam ID: $id_right_cam  (exported as BLUEFOX_RIGHT_ID)"
    echo "Exposure: $EXPOSURE us (both cams, exported as EXPOSE_US_LEFT / EXPOSE_US_RIGHT)"
    echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

    # Make available to this script's own process without requiring a new shell
    export BLUEFOX_LEFT_ID="$id_left_cam"
    export BLUEFOX_RIGHT_ID="$id_right_cam"
    export EXPOSE_US_LEFT="$EXPOSURE"
    export EXPOSE_US_RIGHT="$EXPOSURE"

    echo -e "\033[1;32mNow reconnect BOTH cameras. Wait ~5s, then press any key.\033[0m"
    read -n 1 -r -s
    echo
    sleep 3
}

test_cam() {
    local camera_name=$1
    local topic="/${UAV_NAME}/${camera_name}/bluefox/image_raw"
    echo -e "\033[1;34m\n${camera_name} camera output (topic: ${topic}):\033[0m"
    timeout "${TEST_DURATION_SEC}" ros2 topic hz "$topic" || \
        echo -e "\033[0;33mNo messages received on ${topic} within ${TEST_DURATION_SEC}s.\033[0m"
}

###############################################################

echo ""
echo -e "\033[0;35m#######################################################################"
echo "##################### UVDAR ROS 2 Camera Config Script ###############"
echo -e "#######################################################################\n\033[0m"

read -n 2 -p $'\033[1;32mAre you calling this script on a real drone? [y/n]\n\033[0m' resp_uav
echo
if [[ ! "$resp_uav" =~ ^[yY] ]]; then
    echo -e "\033[0;33mPlease call this script only on a real UAV.\033[0m"
    exit 1
fi

echo "####################### Camera Configuration #######################"
extract_id_two_cams
update_calib_symlinks
write_ids_to_bashrc

echo "Testing cameras. One moment please..."
# two_bluefox.launch.py reads BLUEFOX_LEFT_ID / BLUEFOX_RIGHT_ID / EXPOSE_US_LEFT /
# EXPOSE_US_RIGHT directly from the environment (not as launch arguments) - the
# only declared launch argument on this file is uav_name. write_ids_to_bashrc()
# already exported these vars into this script's own environment above.
ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" \
    uav_name:="$UAV_NAME" \
    &> "$tmp_file_cam_launch" &
pid_cam_launch=$!

sleep 10

test_cam left
test_cam right

kill "$pid_cam_launch" 2>/dev/null || true
wait "$pid_cam_launch" 2>/dev/null || true
rm -f "$tmp_file_cam_launch"

echo -e "##################### Camera Configuration done! ####################\n"
echo "Environment variables written to ~/.bashrc — run 'source ~/.bashrc' or open a new shell to use them elsewhere."

exit 0
