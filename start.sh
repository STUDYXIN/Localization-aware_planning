#!/bin/bash
PASSWD="'"
#source ${CURRENT_DIR}/devel/setup.bash

echo ${PASSWD} | sudo -S chmod +777 /dev/ttyACM0
cd /home/nzl-star/vins_mapping
source ./devel/setup.bash
gnome-terminal -t "Px4Control" -x bash -c "roslaunch px4ctrl run_ctrl_real_world.launch;exec bash;"

sleep 3s
gnome-terminal -t "realsense" -x bash -c "roslaunch realsense2_camera rs_camera_aligned_depth.launch;exec bash;"

sleep 7s
gnome-terminal -t "vins" -x bash -c "roslaunch vins fast_drone_250.launch;exec bash;"

sleep 1s
#roslaunch vins rviz.launch & sleep 5;
gnome-terminal -t "surfel_mapping" -x bash -c "roslaunch surfel_fusion vins_realsense.launch;exec bash;"

wait;
