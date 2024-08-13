#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>

class Parameter_t {
 public:
  // 控制器增益参数
  struct Gain {
    double Kp0, Kp1, Kp2;
    double Kv0, Kv1, Kv2;
    double Kvi0, Kvi1, Kvi2;
    double Kvd0, Kvd1, Kvd2;
    double KAngR, KAngP, KAngY;
  };
  
  // 旋翼阻力
  struct RotorDrag {
    double x, y, z;
    double k_thrust_horz;
  };

  // 消息超时时间
  struct MsgTimeout {
    double odom;
    double rc;
    double cmd;
    double imu;
    double bat;
  };

  // 无人机油门映射曲线参数
  struct ThrustMapping {
    bool print_val;  // 是否打印无人机悬停时的油门估计值
    double K1;
    double K2;
    double K3;
    bool accurate_thrust_model;
    double hover_percentage;  // 悬停油门百分比
  };

  // 遥控器通道反转标志位
  struct RCReverse {
    bool roll;
    bool pitch;
    bool yaw;
    bool throttle;
  };

  // 自动起飞降落标志位
  struct AutoTakeoffLand {
    bool enable;           // 是否允许自动起降
    bool enable_auto_arm;  // 是否允许自动解锁
    bool no_RC;            // 是否无遥控器
    double height;         // 自动起飞高度
    double speed;          // 自动起飞速度
  };

  Gain gain;
  RotorDrag rt_drag;
  MsgTimeout msg_timeout;
  RCReverse rc_reverse;
  ThrustMapping thr_map;
  AutoTakeoffLand takeoff_land;

  int pose_solver;        // 姿态解算器选择标志位
  double mass;            // 无人机质量
  double gra;             // 重力加速度
  double max_angle;       // 最大姿态角(弧度)
  double ctrl_freq_max;   // 控制频率
  double max_manual_vel;  // 手动飞行最大速度
  double low_voltage;     // 低电压值(低于该值,自动降落)

  bool use_bodyrate_ctrl;  // 是否使用角速度控制
  bool use_simulation_;    // 是否使用仿真

  Parameter_t();
  void config_from_ros_handle(const ros::NodeHandle &nh);

 private:
	/*
	* @brief 从 ROS 参数服务器中读取必要的参数值.
	* @param nh: ROS 句柄
	* @param name: 参数在 yaml 文件中的名称
	* @param val: 参数在 C++ 文件中的变量名称
	*/
  template <typename TName, typename TVal>
  void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val) {
    if (nh.getParam(name, val)) {
      // pass
    } else {
      ROS_ERROR_STREAM("Read param: " << name << " failed.");
      ROS_BREAK();
    }
  };
};

#endif
