#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"

class RC_Data_t {
 public:
  double mode;
  double gear;
  double reboot_cmd;
  double last_mode;
  double last_gear;
  double last_reboot_cmd;
  double takeoff_trigger;
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4];  // 遥控器的四个主要通道数值

  mavros_msgs::RCIn msg;
  ros::Time rcv_stamp;

  bool is_take_off;         // 自动起飞
  bool is_command_mode;     // 是否处于命令模式
  bool enter_command_mode;  // 是否进入命令模式
  bool is_hover_mode;       // 是否处于悬停模式
  bool enter_hover_mode;    // 是否进入悬停模式
  bool toggle_reboot;       // 是否重启飞控

  static constexpr double GEAR_SHIFT_VALUE = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;     // 重启命令阈值
  static constexpr double DEAD_ZONE = 0.25;                 // 死区范围

  RC_Data_t();
  void check_validity();
  bool check_centered();
  void feed(mavros_msgs::RCInConstPtr pMsg);
};

class Odom_Data_t {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;     // 位置
  Eigen::Vector3d v;     // 速度
  Eigen::Quaterniond q;  // 姿态
  Eigen::Vector3d w;     // 角速度

  nav_msgs::Odometry msg;  // 最近接收的里程计数据
  ros::Time rcv_stamp;     // 最近接收到里程计的时间戳
  bool recv_new_msg;       // 是否收到新的里程计数据

  Odom_Data_t();
  void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Imu_Data_t {
 public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d a;

  sensor_msgs::Imu msg;
  ros::Time rcv_stamp;

  Imu_Data_t();
  void feed(sensor_msgs::ImuConstPtr pMsg);
};

class State_Data_t {
 public:
  mavros_msgs::State current_state;
  mavros_msgs::State state_before_offboard;

  State_Data_t();
  void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t {
 public:
  mavros_msgs::ExtendedState current_extended_state;

  ExtendedState_Data_t();
  void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

class Command_Data_t {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;  // 位置
  Eigen::Vector3d v;  // 速度
  Eigen::Vector3d a;  // 加速度
  Eigen::Vector3d j;  // 加加速度
  double yaw;         // 偏航角
  double yaw_rate;    // 偏航角速度

  quadrotor_msgs::PositionCommand msg;  // 最近接收的期望状态指令
  ros::Time rcv_stamp;                  // 最近接收到指令的时间戳

  Command_Data_t();
  void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Battery_Data_t {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};       // 电池电压
  double percentage{0.0}; // 电池电量百分比

  sensor_msgs::BatteryState msg;
  ros::Time rcv_stamp;

  Battery_Data_t();
  void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool triggered{false};
  uint8_t takeoff_land_cmd;  // see TakeoffLand.msg for its defination

  quadrotor_msgs::TakeoffLand msg;
  ros::Time rcv_stamp;

  Takeoff_Land_Data_t();
  void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};

#endif