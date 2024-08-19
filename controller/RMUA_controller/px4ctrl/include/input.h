#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h> // 在../utils/quadrotor_msgs中
#include <quadrotor_msgs/TakeoffLand.h>
#include <uav_utils/utils.h>
#include <geometry_msgs/Pose.h>
#include "PX4CtrlParam.h"
// #include "controller.h"
#include <Eigen/Dense>
#include <queue>

class Odom_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;    // 世界系下的坐标
    Eigen::Vector3d v;    // 世界系下的速度
    Eigen::Quaterniond q; // 世界系下的姿态四元数
    Eigen::Vector3d w;    // 世界系下的三轴角速度

    nav_msgs::Odometry msg; // 里程计信息

    ros::Time rcv_stamp; // 时间戳
    bool recv_new_msg;   // 接收到新消息的标志位

    Odom_Data_t();
    void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Imu_Data_t
{
public:
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;

    Imu_Data_t();
    void feed(sensor_msgs::ImuConstPtr pMsg);
};

// class State_Data_t

class Command_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;

    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;

    Command_Data_t();
    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

struct Desired_State_t // 期望状态
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Quaterniond q;
    double yaw;
    double yaw_rate;

    Desired_State_t() {};

    Desired_State_t(Odom_Data_t &odom) // 输入里程计值获得构造函数
        : p(odom.p),
          v(Eigen::Vector3d::Zero()),
          a(Eigen::Vector3d::Zero()),
          j(Eigen::Vector3d::Zero()),
          q(odom.q),
          yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
          yaw_rate(0) {};
};

/*********使用队列实现接收参考轨迹点***************/
struct state_ref
{
    /* data */
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    double yaw;
    Eigen::Vector3d a;
    ros::Time rcv_stamp;
};
class ref_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::queue<state_ref> data_ref;
    bool data_ready = false; // 判断队列是否是满的，只有队列是满的，才可以开始进行轨迹跟踪
    int n_mpc = 15;          // 队列存储的最大长度

    // Desired_State_t msg;
    ref_Data_t();
    void feed(const Desired_State_t &des, const Odom_Data_t &odom_data);
};
/**********************************************/

class Takeoff_Land_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool triggered{false};
    uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination
    bool takeoff_flag = false;
    bool land_flag = true;

    quadrotor_msgs::TakeoffLand msg;
    ros::Time rcv_stamp;

    Takeoff_Land_Data_t();
    void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};

#endif