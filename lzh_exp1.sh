#!/bin/bash

FILEDIR=$(readlink -f ${BASH_SOURCE})
BASEDIR=$(dirname ${FILEDIR})

source ${BASEDIR}/../../devel/setup.bash

sleep 2s
gnome-terminal -t "Control" -x bash -c "roslaunch airsim_ctrl ctrl_md_exploration.launch;exec bash;"

sleep 1s
gnome-terminal -t "Perception" -x bash -c "roslaunch vins vins_airsim.launch;exec bash;"

sleep 1s
gnome-terminal -t "Planner" -x bash -c "roslaunch plan_manage vins_tra_pub_rviz.launch;exec bash;"