#!/bin/bash
### BEGIN INIT INFO
# Provides: tmux
# Required-Start:    $local_fs $network dbus
# Required-Stop:     $local_fs $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start the uav
### END INIT INFO
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi

source $HOME/.bashrc

# location for storing the bag files
# * do not change unless you know what you are doing
MAIN_DIR=~/"bag_files"

# the project name
# * is used to define folder name in ~/$MAIN_DIR
PROJECT_NAME=just_flying

# the name of the TMUX session
# * can be used for attaching as 'tmux a -t <session name>'
SESSION_NAME=mav

# following commands will be executed first in each window
# * do NOT put ; at the end
pre_input=""

# define commands
# 'name' 'command'
# * DO NOT PUT SPACES IN THE NAMES
# * "new line" after the command    => the command will be called after start
# * NO "new line" after the command => the command will wait for user's <enter>
input=(
  'Rosbag' 'waitForOffboard; ./record.sh
'
  'Sensors' 'waitForTime; roslaunch mrs_uav_deployment sensors.launch
'
  'Nimbro' 'waitForTime; rosrun mrs_uav_deployment run_nimbro.py ./config/network_config.yaml `rospack find mrs_uav_deployment`/config/communication_config.yaml
'
  'HwApi' 'waitForTime; roslaunch mrs_uav_px4_api api.launch
'
  'Status' 'waitForHw; roslaunch mrs_uav_status status.launch
'
  'Core' 'waitForTime; roslaunch mrs_uav_core core.launch platform_config:=`rospack find mrs_uav_deployment`/config/mrs_uav_system/$UAV_TYPE.yaml world_config:=`rospack find mrs_uav_deployment`/config/worlds/world_$WORLD_NAME.yaml custom_config:=./config/custom_config.yaml network_config:=./config/network_config.yaml
'
  'RTK' 'waitForHw; roslaunch mrs_serial rtk.launch
'
  'AutoStart' 'waitForHw; roslaunch mrs_uav_autostart automatic_start.launch
'
  'led_manager' 'waitForRos; roslaunch uvdar_core led_manager.launch
'
  'Trajectory' 'history -s rosservice call /'"$UAV_NAME"'/control_manager/start_trajectory_tracking; history -s rosservice call /'"$UAV_NAME"'/control_manager/goto_trajectory_start; history -s roslaunch uvdar_core load_trajectory.launch file:="tx1/line.txt" loop:=true
  '
  'Stop Trajectory' 'history -s rosservice call /'"$UAV_NAME"'/control_manager/stop_trajectory_tracking;
  '
# do NOT modify the command list below
  'EstimDiag' 'waitForHw; rostopic echo /'"$UAV_NAME"'/estimation_manager/diagnostics
'
  'kernel_log' 'tail -f /var/log/kern.log -n 100
'
  'roscore' 'roscore
'
  'simtime' 'waitForRos; rosparam set use_sim_time false
'
)

# the name of the window to focus after start
init_window="Status"

# automatically attach to the new session?
# {true, false}, default true
attach=true

###########################
### DO NOT MODIFY BELOW ###
###########################

export TMUX_BIN="/usr/bin/tmux -L mrs -f /etc/ctu-mrs/tmux.conf"

# find the session
FOUND=$( $TMUX_BIN ls | grep $SESSION_NAME )

if [ $? == "0" ]; then
  echo "The session already exists"
  $TMUX_BIN -2 attach-session -t $SESSION_NAME
  exit
fi

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

TMUX= $TMUX_BIN new-session -s "$SESSION_NAME" -d
echo "Starting new session."

# get the iterator
ITERATOR_FILE="$MAIN_DIR/$PROJECT_NAME"/iterator.txt
if [ -e "$ITERATOR_FILE" ]
then
  ITERATOR=`cat "$ITERATOR_FILE"`
  ITERATOR=$(($ITERATOR+1))
else
  echo "iterator.txt does not exist, creating it"
  mkdir -p "$MAIN_DIR/$PROJECT_NAME"
  touch "$ITERATOR_FILE"
  ITERATOR="1"
fi
echo "$ITERATOR" > "$ITERATOR_FILE"

# create file for logging terminals' output
LOG_DIR="$MAIN_DIR/$PROJECT_NAME/"
SUFFIX=$(date +"%Y_%m_%d_%H_%M_%S")
SUBLOG_DIR="$LOG_DIR/"$ITERATOR"_"$SUFFIX""
TMUX_DIR="$SUBLOG_DIR/tmux"
mkdir -p "$SUBLOG_DIR"
mkdir -p "$TMUX_DIR"

# link the "latest" folder to the recently created one
rm "$LOG_DIR/latest" > /dev/null 2>&1
rm "$MAIN_DIR/latest" > /dev/null 2>&1
ln -sf "$SUBLOG_DIR" "$LOG_DIR/latest"
ln -sf "$SUBLOG_DIR" "$MAIN_DIR/latest"

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}"
  ((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 3

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  $TMUX_BIN send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;
${pre_input};
${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

$TMUX_BIN select-window -t $SESSION_NAME:$init_index

if $attach; then

  if [ -z ${TMUX} ];
  then
    $TMUX_BIN -2 attach-session -t $SESSION_NAME
  else
    tmux detach-client -E "tmux -L mrs a -t $SESSION_NAME"
  fi
else
  echo "The session was started"
  echo "You can later attach by calling:"
  echo "  tmux -L mrs a -t $SESSION_NAME"
fi
