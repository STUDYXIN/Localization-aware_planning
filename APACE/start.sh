#!/bin/bash

# 获取当前脚本的绝对路径
FILEDIR=$(readlink -f "${BASH_SOURCE[0]}")
BASEDIR=$(dirname "${FILEDIR}")
SETUP_FILE="${BASEDIR}/../../../devel/setup.bash"
SETUP_DIR=$(dirname "${SETUP_FILE}")

ENABLEVINS=false
USEKEBOARD_SPEED=0

echo "setup.bash is located in: ${SETUP_DIR}"
# 加载 ROS 环境
source "${SETUP_FILE}"

sleep 0.5s
gnome-terminal --tab -- bash -c "\
echo Rviz; \
roslaunch plan_manage rviz.launch; exec bash"

# 启动 Simulation
sleep 0.5s
gnome-terminal --tab -- bash -c "\
echo Simulation; \
./../../../../simulator/LinuxNoEditor/Blocks.sh -ResX=640 -ResY=480 -windowed; \
exec bash"


# 启动 Control
sleep 2s
gnome-terminal --tab -- bash -c "\
echo Control; \
roslaunch airsim_ctrl ctrl_md_exploration.launch enable_vins:=${ENABLEVINS}; \
exec bash"

# 启动 Perception
sleep 1s
gnome-terminal --tab -- bash -c "\
echo Perception; \
roslaunch vins vins_airsim.launch; \
exec bash"

# 启动 Planner
sleep 3s
gnome-terminal --tab -- bash -c "\
echo Planner; \
roslaunch plan_manage agile_planner_airsim.launch enable_vins:=${ENABLEVINS} keyboard_vector:=${USEKEBOARD_SPEED}; \
exec bash"
