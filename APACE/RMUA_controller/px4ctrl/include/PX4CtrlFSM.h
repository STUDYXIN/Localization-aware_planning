#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <input.h>
#include <controller.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/Land.h>
#include <quadrotor_msgs/TakeoffLand.h>

struct AutoTakeoffLand_t
{
    bool landed{false};
    ros::Time toggle_takeoff_land_time;
    Eigen::Vector4d start_pose;
    // motor idle running for 3 seconds before takeoff
    static constexpr double MOTORS_SPEEDUP_TIME = 3.0;
};

// TODO: transplant PX4CtrlFSM to AirSim

class PX4CtrlFSM
{
public:
    Parameter_t &param;

    Odom_Data_t odom_data;
    Imu_Data_t imu_data;
    Command_Data_t cmd_data;
    Takeoff_Land_Data_t takeoff_land_data;

    quadrotor_msgs::TakeoffLand takeoff_land_state_msg;

    airsim_ros_pkgs::Takeoff takeoff;
    airsim_ros_pkgs::Land land;

    Controller &controller;

    ros::Publisher traj_start_trigger_pub;
    ros::Publisher ctrl_FCU_pub;
    ros::Publisher debug_pub; // debug
    ros::Publisher takeoff_state_pub;

    ros::ServiceClient takeoff_client;
    ros::ServiceClient reset_client;
    ros::ServiceClient land_client;

    quadrotor_msgs::Px4ctrlDebug debug_msg; // debug

    Eigen::Vector4d hover_pose;
    ros::Time last_set_hover_pose_time;

    enum State_t
    {
        ON_GROUND,
        AUTO_HOVER,
        CMD_CTRL,
        AUTO_TAKEOFF,
        AUTO_LAND
    };

    PX4CtrlFSM(Parameter_t &, Controller &);
    void process();
    bool cmd_is_received(const ros::Time &now_time);
    bool odom_is_received(const ros::Time &now_time);
    bool imu_is_received(const ros::Time &now_time);
    bool recv_new_odom();
    State_t get_state() { return state; }
    bool get_landed() { return takeoff_land.landed; }

private:
    State_t state;
    AutoTakeoffLand_t takeoff_land;

    /* control related */
    Desired_State_t get_hover_des();
    Desired_State_t get_cmd_des();

    /* auto takeoff/land */
    void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
    void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing
    void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
    Desired_State_t get_rotor_speed_up_des(const ros::Time now);
    Desired_State_t get_takeoff_land_des(const double speed);

    /* tools */
    void set_hov_with_odom();

    void publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
    void publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
    void publish_trigger(const nav_msgs::Odometry &odom_msg);
};

#endif