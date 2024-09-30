#!/bin/bash

# 获取当前脚本的绝对路径
FILEDIR=$(readlink -f "${BASH_SOURCE[0]}")
BASEDIR=$(dirname "${FILEDIR}")
SETUP_FILE="${BASEDIR}/../../devel/setup.bash"
SETUP_DIR=$(dirname "${SETUP_FILE}")
# 第一个地图的参数
# PLY_NAME=right_corner_sample25.ply
# FEATURE_NAME=feature_map_sample1.ply
# INIT_X=-13.0
# INIT_Y=0.0
# INIT_Z=1.0
# GOAL_X=0.0
# GOAL_Y=3.0
# GOAL_Z=2.0
# MAPX=50.0
# MAPY=50.0
# MAPZ=5.1
# BOXMINX=-15.0
# BOXMINY=-15.0
# BOXMINZ=0.5
# BOXMAXX=15.0
# BOXMAXY=15.0
# BOXMAXZ=4.0


# 第二个地图的参数
PLY_NAME=forest_sample5.ply
FEATURE_NAME=forest_feature_703.ply
INIT_X=-13.0
INIT_Y=13.0
INIT_Z=1.0
GOAL_X=10.0
GOAL_Y=-10.0
GOAL_Z=2.0
MAPX=50.0
MAPY=50.0
MAPZ=5.1
BOXMINX=-15.0
BOXMINY=-15.0
BOXMINZ=0.5
BOXMAXX=15.0
BOXMAXY=15.0
BOXMAXZ=4.0


# 第三个地图的参数
# PLY_NAME=simple_scene1.ply
# FEATURE_NAME=simple_scene1_feature.ply
# INIT_X=-5.0
# INIT_Y=2.0
# INIT_Z=1.0
# GOAL_X=10.0
# GOAL_Y=-10.0
# GOAL_Z=2.0
# MAPX=50.0
# MAPY=50.0
# MAPZ=5.1
# BOXMINX=-15.0
# BOXMINY=-15.0
# BOXMINZ=0.5
# BOXMAXX=15.0
# BOXMAXY=15.0
# BOXMAXZ=4.0


# 第四个地图的参数
# PLY_NAME=simple_scene2.ply
# FEATURE_NAME=simple_scene2_feature.ply
# INIT_X=-6.0
# INIT_Y=1.0
# INIT_Z=1.0
# GOAL_X=4.5
# GOAL_Y=3.5
# GOAL_Z=3.0
# MAPX=50.0
# MAPY=50.0
# MAPZ=5.1
# BOXMINX=-15.0
# BOXMINY=-15.0
# BOXMINZ=0.5
# BOXMAXX=15.0
# BOXMAXY=15.0
# BOXMAXZ=4.0


# 第五个地图的参数
# PLY_NAME=long_corridor.ply
# FEATURE_NAME=long_corridor_feature.ply
# INIT_X=0.0
# INIT_Y=0.0
# INIT_Z=2.0
# GOAL_X=51.1
# GOAL_Y=14.0
# GOAL_Z=3.0
# MAPX=130.0
# MAPY=41.0
# MAPZ=15.0
# BOXMINX=-3.0
# BOXMINY=-15.0
# BOXMINZ=-0.5
# BOXMAXX=63.0
# BOXMAXY=20.0
# BOXMAXZ=7.0


echo "setup.bash is located in: ${SETUP_DIR}"
# 加载 ROS 环境
source "${SETUP_FILE}"
# 启动 Rviz
sleep 0.5s
gnome-terminal --tab -- bash -c "\
echo Rviz; \
roslaunch exploration_manager rviz.launch; exec bash"

# 加载点云文件
sleep 1.5s
gnome-terminal --tab -- bash -c "\
echo PCL; \
roslaunch map_publisher map_publisher.launch file_name:=${PLY_NAME}; exec bash"

# 启动 Planner
sleep 1s
gnome-terminal --tab -- bash -c "\
echo Planner; \
roslaunch exploration_manager exploration.launch feature_filename_:=${FEATURE_NAME} init_x:=${INIT_X} init_y:=${INIT_Y} init_z:=${INIT_Z} goal_x:=${GOAL_X} goal_y:=${GOAL_Y} goal_z:=${GOAL_Z} import_map_size_x:=${MAPX} import_map_size_y:=${MAPY} import_map_size_z:=${MAPZ} import_box_min_x:=${BOXMINX} import_box_min_y:=${BOXMINY} import_box_min_z:=${BOXMINZ}  import_box_max_x:=${BOXMAXX} import_box_max_y:=${BOXMAXY} import_box_max_z:=${BOXMAXZ}; \
exec bash"


