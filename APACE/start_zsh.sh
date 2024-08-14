#!/bin/zsh

# 获取当前脚本的绝对路径
FILEDIR=$(readlink -f "${(%):-%x}")
BASEDIR=$(dirname "${FILEDIR}")
SETUP_FILE="${BASEDIR}/../../../devel/setup.zsh"
SETUP_DIR=$(dirname "${SETUP_FILE}")

echo "setup.zsh is located in: ${SETUP_DIR}"
# 加载 ROS 环境
source "${SETUP_FILE}"

# 启动 Rviz
gnome-terminal --title="Rviz" -- bash -c "roslaunch plan_manage rviz.launch; exec bash"

# 启动 Simulation
sleep 0.5s
gnome-terminal --title="Simulation" -- bash -c "./../../../../simulator/LinuxNoEditor/Blocks.sh -ResX=640 -ResY=480 -windowed; exec bash"

# 启动 Control
sleep 2s
gnome-terminal --title="Control" -- bash -c "roslaunch airsim_ctrl ctrl_md_exploration.launch; exec bash"


# 启动 Perception
sleep 1s
gnome-terminal --title="Perception" -- bash -c "roslaunch vins vins_airsim.launch; exec bash"

# 启动 Planner
sleep 5s
gnome-terminal --title="Planner" -- bash -c "roslaunch plan_manage agile_planner_airsim.launch; exec bash"


