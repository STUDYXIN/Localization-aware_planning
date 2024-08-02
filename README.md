# Localization-aware_planning
Localization-aware planning  

## 更改说明：
## APACE 
```
修改plan_manage/script 增加数个脚本，用于发布无人机运行轨迹，记录数据等

修改plan_manage/launch 增加数个启动文件：
bag_record_vins_orb_airsim.launch 录包
circle_fly_msg.launch 不使用
orb_tra_pub_rviz.launch 需要同时开启 vins节点和orb3节点， 该launch功能为发布轨迹让无人机动起来，并可视化
vins_tra_pub_rviz.launch 需要提前开启 airsim vins ，该launch功能同上
```
## VINS-Fusion
```
各文件都有所修改，主要做到适配airsim，发布特定点云等
```

## 启动
以直线行驶150m为例，在UE4搭建好后（或者运行包）
```
roslaunch airsim_ctrl ctrl_md_exploration.launch #运行rosbag不用启动
roslaunch vins vins_airsim.launch
roslaunch plan_manage vins_tra_pub_rviz.launch
```

## start_record_zsh.sh
在运行airsim工程后启动，需要检查result目录是否有features_data.csv和start_vins_signal.txt。