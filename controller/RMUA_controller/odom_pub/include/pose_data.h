#ifndef __POSE_DATA_H
#define __POSE_DATA_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

class Pose_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    Eigen::Vector3d p_last;
    Eigen::Quaterniond q_last;

    geometry_msgs::PoseStamped msg;
    ros::Time rcv_stamp;
    ros::Time last_rcv_stamp;

    Pose_Data_t();
    void feed(geometry_msgs::PoseStampedConstPtr pMsg);
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

#endif