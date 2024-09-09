#!/bin/bash

# 获取当前脚本的绝对路径
FILEDIR=$(readlink -f "${BASH_SOURCE[0]}")
BASEDIR=$(dirname "${FILEDIR}")
SETUP_FILE="${BASEDIR}/../../devel/setup.bash"
SETUP_DIR=$(dirname "${SETUP_FILE}")
# 第一个地图的参数
PLY_NAME=right_corner_sample25.ply
FEATURE_NAME=feature_map_sample1.ply
INIT_Y=0.0
# 第二个地图的参数
# PLY_NAME=forest_sample5.ply
# FEATURE_NAME=forest_feature_703.ply
# INIT_Y=13.0

echo "setup.bash is located in: ${SETUP_DIR}"
# 加载 ROS 环境
source "${SETUP_FILE}"
# 启动 Rviz
sleep 0.5s
gnome-terminal --tab -- bash -c "\
echo Rviz; \
roslaunch exploration_manager rviz.launch; exec bash"

# 加载点云文件
sleep 1s
gnome-terminal --tab -- bash -c "\
echo PCL; \
roslaunch map_publisher map_publisher.launch file_name:=${PLY_NAME}; exec bash"
# 启动 Planner
sleep 1s
gnome-terminal --tab -- bash -c "\
echo Planner; \
roslaunch exploration_manager exploration.launch feature_filename_:=${FEATURE_NAME} init_y:=${INIT_Y}; \
exec bash"


