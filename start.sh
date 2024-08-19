#!/bin/bash

FILEDIR=$(readlink -f ${BASH_SOURCE})
BASEDIR=$(dirname ${FILEDIR})

echo "File directory: ${BASEDIR}"

source ${BASEDIR}/../../devel/setup.bash

gnome-terminal -t "Rviz" -x bash -c "roslaunch plan_manage rviz.launch;exec bash;"

#shell脚本里变量名和等号之间不能有空格
ENABLE_VINS=true

sleep 0.5s
gnome-terminal -t "Simulation" -x bash -c "$SIMULATION_PATH/Blocks/LinuxNoEditor/Blocks.sh -ResX=640 -ResY=480 -windowed;exec bash;"

sleep 4s
gnome-terminal -t "Control" -x bash -c "roslaunch px4ctrl airsim_ctrl.launch enable_vins:=${ENABLE_VINS};exec bash;"

sleep 0.5s
gnome-terminal -t "Perception" -x bash -c "roslaunch vins vins_airsim.launch;exec bash;"

sleep 0.5s
gnome-terminal -t "Planner" -x bash -c "roslaunch plan_manage fast_planner_airsim.launch enable_vins:=${ENABLE_VINS};exec bash;"
