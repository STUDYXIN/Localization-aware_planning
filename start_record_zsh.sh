#!/bin/zsh

# BASEDIR=$(cd $(dirname $0) && pwd )
# 该启动脚本运行前需要编译程序，并把devel/setuo.zsh目录放到~/.zshrc中
source ~/.zshrc

gnome-terminal --tab --title="Airsim_ctrl" -- zsh -c "roslaunch airsim_ctrl ctrl_md_exploration.launch;exec zsh;"
sleep 1.5s
gnome-terminal --tab --title="Vins" -- zsh -c "roslaunch vins vins_airsim.launch;exec zsh;"

sleep 3.0s
gnome-terminal --tab --title="trajectory_publisher" -- zsh -c " roslaunch plan_manage vins_tra_pub_rviz.launch;exec zsh;"

# Test for real
# gnome-terminal --tab --title="Planner" -- zsh -c "roslaunch ego_planner rmua_test4_real.launch;exec zsh;"