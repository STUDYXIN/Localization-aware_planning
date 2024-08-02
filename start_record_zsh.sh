#!/bin/zsh

# 确保编译完成并且环境变量设置正确
source ~/.zshrc

# 获取 vins 功能包的路径
VINS_PACKAGE_PATH=$(rospack find vins)
SIGNAL_FILE="${VINS_PACKAGE_PATH}/../result/start_vins_signal.txt"

# 清空信号文件
if [ -f "$SIGNAL_FILE" ]; then
    : > "$SIGNAL_FILE" # 清空文件内容
fi

# 清屏
clear

# 定义清理函数
cleanup() {
    echo "清理中..."

    # 终止所有相关进程
    pkill -f "roslaunch airsim_ctrl ctrl_md_exploration.launch"
    pkill -f "roslaunch plan_manage vins_tra_pub_rviz.launch"
    pkill -f "roslaunch vins vins_airsim.launch"

    pkill -f rosmaster
    pkill -f roslaunch
    rosparam delete /
    
    echo "清理完毕！"
    exit
}

# 捕获 SIGINT 信号 (Ctrl+C) 并调用 cleanup 函数
trap cleanup SIGINT

# 启动 AirSim 控制
echo "启动 AirSim 控制..."
gnome-terminal --tab --title="Airsim_ctrl" -- zsh -c "roslaunch airsim_ctrl ctrl_md_exploration.launch; exec zsh;" &
sleep 1
# 启动轨迹发布器
echo "启动轨迹发布器..."
gnome-terminal --tab --title="trajectory_publisher" -- zsh -c "roslaunch plan_manage vins_tra_pub_rviz.launch; exec zsh;" &

# 等待无人机到达开始地点
echo "等待无人机到达开始地点..."
while true; do
    if [ -f "$SIGNAL_FILE" ]; then
        signal=$(cat "$SIGNAL_FILE")
        echo "$signal ..."
        if [ "$signal" = "start" ]; then
            echo "到达指定位置！启动 VINS..."
            gnome-terminal --tab --title="Vins" -- zsh -c "roslaunch vins vins_airsim.launch; exec zsh;"
            echo "等待5s VINS初始化......"
            sleep 5
            : > "$SIGNAL_FILE" # 清空文件内容
            echo "finish" > "$SIGNAL_FILE"
            echo "已写入 'finish'，轨迹开始运行！！！！！"
        fi
        if [ "$signal" = "reach" ]; then
            echo "到达指定位置!"
            break
        fi
    else
        echo "无法打开信号文件或文件不存在: $SIGNAL_FILE"
    fi
    sleep 1
done

# Test for real
# gnome-terminal --tab --title="Planner" -- zsh -c "roslaunch ego_planner rmua_test4_real.launch; exec zsh;"
cleanup
echo "脚本执行完毕。"
