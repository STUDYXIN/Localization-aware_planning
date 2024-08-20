#!/bin/bash

# 获取当前脚本的绝对路径
FILEDIR=$(readlink -f "${BASH_SOURCE[0]}")
BASEDIR=$(dirname "${FILEDIR}")
SETUP_FILE="${BASEDIR}/../../devel/setup.bash"
SETUP_DIR=$(dirname "${SETUP_FILE}")

echo "setup.bash is located in: ${SETUP_DIR}"
# 加载 ROS 环境
source "${SETUP_FILE}"

sleep 0.5s
gnome-terminal --tab -- bash -c "\
echo Rviz; \
roslaunch exploration_manager rviz.launch; exec bash"

# 启动 Planner
sleep 1s
gnome-terminal --tab -- bash -c "\
echo Planner; \
roslaunch exploration_manager exploration.launch; \
exec bash"
