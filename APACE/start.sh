#!/bin/bash

FILEDIR=$(readlink -f ${BASH_SOURCE})
BASEDIR=$(dirname ${FILEDIR})

echo "File directory: ${BASEDIR}"

source ${BASEDIR}/../../../devel/setup.bash

gnome-terminal -t "Rviz" -x bash -c "roslaunch plan_manage rviz.launch;exec bash;"

# sleep 0.5s
# gnome-terminal -t "Simulation" -x bash -c "../../../CenterFeature/LinuxNoEditor/AirSimStreetView.sh -windowed;exec bash;"

# sleep 0.5s
# gnome-terminal -t "Simulation" -x bash -c "../../../wall_with_some_apriltag/LinuxNoEditor/Blocks.sh -ResX=640 -ResY=480 -windowed;exec bash;"

sleep 0.5s
gnome-terminal -t "Simulation" -x bash -c "./simulation/Blocks/LinuxNoEditor/Blocks.sh -ResX=640 -ResY=480 -windowed;exec bash;"

sleep 2s
gnome-terminal -t "Control" -x bash -c "roslaunch airsim_ctrl ctrl_md_exploration.launch;exec bash;"

sleep 0.5s
gnome-terminal -t "Planner" -x bash -c "roslaunch plan_manage agile_planner_airsim.launch;exec bash;"

sleep 8s
gnome-terminal -t "Perception" -x bash -c "roslaunch vins vins_airsim.launch;exec bash;"

# sleep 0.5s
# gnome-terminal -t "Checker" -x bash -c "rosrun map_publisher collision_checker.py;exec bash;"
