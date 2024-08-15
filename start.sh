#!/bin/bash

FILEDIR=$(readlink -f ${BASH_SOURCE})
BASEDIR=$(dirname ${FILEDIR})

echo "File directory: ${BASEDIR}"

source ${BASEDIR}/../../devel/setup.bash

gnome-terminal -t "Rviz" -- bash -c "roslaunch plan_manage rviz.launch;exec bash;"

# sleep 0.5s
# gnome-terminal -t "Simulation" -- bash -c "../CenterFeature/LinuxNoEditor/AirSimStreetView.sh -windowed;exec bash;"

# sleep 0.5s
# gnome-terminal -t "Simulation" -- bash -c "../Block/LinuxNoEditor/Blocks.sh -windowed;exec bash;"

# sleep 0.5s
# gnome-terminal -t "Simulation" -- bash -c "./simulation/Block3/LinuxNoEditor/Blocks.sh -ResX=640 -ResY=480 -windowed;exec bash;"

# sleep 0.5s
# gnome-terminal -t "Simulation" -- bash -c "./simulation/CityPark/LinuxNoEditor/CityPark.sh -ResX=640 -ResY=480 -windowed;exec bash;"

ENABLE_VINS=true

# sleep 0.5s
# gnome-terminal -t "Simulation" -- bash -c "./simulation/Blocks/LinuxNoEditor/Blocks.sh -ResX=640 -ResY=480 -windowed;exec bash;"

sleep 4s
gnome-terminal -t "Control" -- bash -c "roslaunch airsim_ctrl ctrl_md_exploration.launch enable_vins:=${ENABLE_VINS};exec bash;"

sleep 0.5s
gnome-terminal -t "Perception" -- bash -c "roslaunch vins vins_airsim.launch;exec bash;"
#gnome-terminal -t "Perception" -- bash -c "roslaunch vins vins_airsim_new.launch enable_vins:=${ENABLE_VINS};exec bash;"

# sleep 0.5s
# gnome-terminal -t "DenseMapping" -- bash -c "roslaunch surfel_fusion vins_airsim.launch enable_rviz:=false;exec bash;"

sleep 0.5s
gnome-terminal -t "Planner" -- bash -c "roslaunch plan_manage fast_planner_airsim.launch enable_vins:=${ENABLE_VINS};exec bash;"
