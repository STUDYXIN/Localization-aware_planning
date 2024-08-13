#ifndef FSTO_H
#define FSTO_H

#include "fsto/config.h"
#include <quadrotor_msgs/PolynomialTraj.h>
#include "decomp_util/ellipsoid_decomp.h"
#include "decomp_ros_utils/data_ros_utils.h"
#include "gcopter/gcopter.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class Visualization
{
public:
    Visualization(Config &conf, ros::NodeHandle &nh_);

    Config config;
    ros::NodeHandle nh;

    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher appliedTrajectoryPub;
    ros::Publisher hPolyPub;
    ros::Publisher textPub;
    ros::Publisher ellipsoidPub;

    void visualize(const Trajectory<7> &appliedTraj, ros::Time timeStamp, double compT);
    void visualizePolyH(const vec_E<Polyhedron3D> &polyhedra, ros::Time timeStamp);
    void visualize(const vec_E<Polyhedron3D> &polyhedra, ros::Time timeStamp);
    void visualizeDoubleball(const Trajectory<7> &traj, int samples,double L,double gAcc, double PayloadSize,double QuadrotorSize);
    
};

class MavGlobalPlanner
{
public:
    MavGlobalPlanner(Config &conf, ros::NodeHandle &nh_);

    Config config;
    ros::NodeHandle nh;

    ros::Subscriber triggerSub;
    void triggerCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    ros::Subscriber odomSub;
    Eigen::Vector3d curOdom;
    ros::Time odomStamp;
    bool odomInitialized;
    void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    ros::Publisher trajPub;

    static void polynomialTrajConverter(const Trajectory<7> &traj,
                                               quadrotor_msgs::PolynomialTraj &msg,
                                               ros::Time &iniStamp);

    EllipsoidDecomp3D cvxDecomp;
    Visualization visualization;
};
#endif