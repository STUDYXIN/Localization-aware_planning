#!/bin/zsh

# 获取当前脚本的绝对路径
FILEDIR=$(readlink -f "${(%):-%x}")
BASEDIR=$(dirname "${FILEDIR}")
SETUP_FILE="${BASEDIR}/../../../devel/setup.zsh"
SETUP_DIR=$(dirname "${SETUP_FILE}")

echo "setup.zsh is located in: ${SETUP_DIR}"
# 加载 ROS 环境
source "${SETUP_FILE}"


sleep 0.5s
gnome-terminal --tab -- zsh -c "\
echo Rviz; \
roslaunch plan_manage rviz.launch; exec bash; \
exec zsh"

# 启动 Simulation
# sleep 0.5s
# gnome-terminal --tab -- zsh -c "\
# echo Simulation; \
# ./../../../../simulator/LinuxNoEditor/Blocks.sh -ResX=640 -ResY=480 -windowed; \
# exec zsh"
sleep 0.5s
gnome-terminal --tab -- zsh -c "\
echo Simulation; \
./ActiveSlam/simulator/LinuxNoEditor/Blocks.sh -ResX=640 -ResY=480 -windowed; \
exec zsh"

# 启动 Control
sleep 2s
gnome-terminal --tab -- zsh -c "\
echo Control; \
roslaunch airsim_ctrl ctrl_md_exploration.launch; \
exec zsh"

# 启动 Perception
sleep 1s
gnome-terminal --tab -- zsh -c "\
echo Perception; \
roslaunch vins vins_airsim.launch; \
exec zsh"

# 启动 Planner
sleep 5s
gnome-terminal --tab -- zsh -c "\
echo Planner; \
roslaunch plan_manage agile_planner_airsim.launch; \
exec zsh"

