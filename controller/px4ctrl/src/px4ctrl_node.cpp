#include <signal.h>
#include <ros/ros.h>

#include "PX4CtrlFSM.h"

// @brief 安全退出 ROS 节点
void safeExitSigintHandler(int sig) {
  ROS_WARN("[px4ctrl] Exit!!!");
  ros::shutdown();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "px4ctrl");
  ros::NodeHandle nh("~");
  // 检测终止信号, 当接收到中断信号(如 Ctrl+C)时, 调用 safeExitSigintHandler 函数安全关闭节点
  signal(SIGINT, safeExitSigintHandler);
  ros::Duration(1.0).sleep();

  // 从 ROS 参数服务器读取参数
  Parameter_t param;
  param.config_from_ros_handle(nh);
  // 控制器
  Controller controller(param);
  // 状态机
  PX4CtrlFSM fsm(param, controller);

  // 无人机状态
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",
                                                               10,
                                                               boost::bind(&State_Data_t::feed, &fsm.state_data, _1));
  // 无人机扩展状态 (垂直起降和降落相关状态)
  ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                               10,
                                                               boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));
  // 里程计数据
  ros::Subscriber odom_sub =
     nh.subscribe<nav_msgs::Odometry>("odom",
                                       100,
                                       boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
  // 无人机期望状态指令
  ros::Subscriber cmd_sub =
      nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                    100,
                                                    boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                    ros::VoidConstPtr(),
                                                    ros::TransportHints().tcpNoDelay());
  // IMU 数据
  ros::Subscriber imu_sub =
      nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",
                                     100,
                                     boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                     ros::VoidConstPtr(),
                                     ros::TransportHints().tcpNoDelay());
  // 遥控器数据
  ros::Subscriber rc_sub;
  // mavros will still publish wrong rc messages although no RC is connected
  if ( !param.takeoff_land.no_RC ) {
    rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                            10,
                                            boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
  }
  // 电池状态
  ros::Subscriber bat_sub =
      nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                              100,
                                              boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
                                              ros::VoidConstPtr(),
                                              ros::TransportHints().tcpNoDelay());
  // 起飞降落命令
  ros::Subscriber takeoff_land_sub =
      nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                              100,
                                              boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                              ros::VoidConstPtr(),
                                              ros::TransportHints().tcpNoDelay());

  // 控制指令发布
  fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
  // 运动规划器触发信号发布
  fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
  // Debug 日志发布
  fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10);

  fsm.takeoff_state_pub=nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/is_hovering",10);

  // 设置飞控模式
  fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  // 飞控解锁与上锁
  fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  // 重启飞控
  fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

  // 暂停 0.5 秒确保所有初始化任务完成
  ros::Duration(0.5).sleep();

  // 检查遥控器连接状态
  if (param.takeoff_land.no_RC) {
    ROS_WARN("[px4ctrl] Remote controller disabled, be careful!");
  } 
  else {
    ROS_INFO("[px4ctrl] Waiting for RC...");
    while (ros::ok()) {
      ros::spinOnce();
      if (fsm.rc_is_received(ros::Time::now())) {
        ROS_INFO("[px4ctrl] RC received.");
        break;
      }
      ros::Duration(0.1).sleep();
    }
  }

  // 检查 PX4 飞控连接状态
  int trials = 0;
  while (ros::ok() && !fsm.state_data.current_state.connected) {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    if (trials++ > 5) ROS_ERROR("[px4ctrl] Unable to connnect to PX4!!!");
  }

  // 进入状态机执行函数循环
  ros::Rate r(param.ctrl_freq_max);
  while (ros::ok()) {
    r.sleep();
    ros::spinOnce();
    fsm.process();
  }

  return 0;
}
