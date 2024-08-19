#!/bin/zsh

FILEDIR=$(readlink -f "${(%):-%N}")
BASEDIR=$(dirname "${FILEDIR}")

echo "File directory: ${BASEDIR}"


# Source the setup file
source ${BASEDIR}/../../devel/setup.zsh

# Launch Rviz in a new terminal tab
gnome-terminal --tab -- zsh -c "\
echo Launching Rviz; \
roslaunch plan_manage rviz.launch; \
exec zsh"

ENABLE_VINS=true

sleep 1s
gnome-terminal --tab -- zsh -c "\
echo Launching simulator; \
./../../../simulator/LinuxNoEditor/Blocks.sh -ResX=640 -ResY=480 -windowed; \
exec zsh"

# Wait for 4 seconds before launching the next node
sleep 2s
gnome-terminal --tab -- zsh -c "\
echo Launching Control with VINS enabled: ${ENABLE_VINS}; \
roslaunch airsim_ctrl ctrl_md_exploration.launch enable_vins:=${ENABLE_VINS}; \
exec zsh"

# Wait for 0.5 seconds before launching the next node
sleep 0.5s
gnome-terminal --tab -- zsh -c "\
echo Launching Perception; \
roslaunch vins vins_airsim.launch; \
exec zsh"

# Wait for 3 seconds before launching the next node
sleep 3s
gnome-terminal --tab -- zsh -c "\
echo Launching Planner with VINS enabled: ${ENABLE_VINS}; \
roslaunch plan_manage fast_planner_airsim.launch enable_vins:=${ENABLE_VINS}; \
exec zsh"