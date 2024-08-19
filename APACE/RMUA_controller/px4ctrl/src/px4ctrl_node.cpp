#include <ros/ros.h>
#include "PX4CtrlFSM.h"
#include <signal.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
// #include <airsim_ros/AngleRateThrottle.h>
// #include <airsim_ros/Takeoff.h>

void mySigintHandler(int sig)
{
  ROS_INFO("[PX4Ctrl] exit...");
  ros::shutdown();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "px4ctrl");
  ros::NodeHandle nh("~");
  // check stop signal
  signal(SIGINT, mySigintHandler);
  ros::Duration(1.0).sleep();
  // initialize the param and read parameters from server
  Parameter_t param;
  param.config_from_ros_handle(nh);
  // initialize the controller
  Controller controller(param);
  PX4CtrlFSM fsm(param, controller);

  // initialize the subscribers
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
      // "/vins_estimator/imu_propagate",
      "odom",
      100,
      boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
      ros::VoidConstPtr(),
      ros::TransportHints().tcpNoDelay());
  ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
      "cmd",
      100,
      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
      ros::VoidConstPtr(),
      ros::TransportHints().tcpNoDelay());

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
      "/airsim_node/drone_1/imu/imu",
      100,
      boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
      ros::VoidConstPtr(),
      ros::TransportHints().tcpNoDelay());
  ros::Subscriber takeoff_land_sub = nh.subscribe<quadrotor_msgs::TakeoffLand>(
      "takeoff_land",
      100,
      boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
      ros::VoidConstPtr(),
      ros::TransportHints().tcpNoDelay());

  // initialize the publishers
  // fsm.ctrl_FCU_pub = nh.advertise<airsim_ros_pkgs::AngleRateThrottle>("/airsim_node/drone_1/angle_rate_throttle_frame", 10);
  fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/setpoint_raw/attitude", 10);
  fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("debugPx4ctrl", 10); // debug
  fsm.takeoff_state_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/is_hovering", 10);
  fsm.takeoff_client = nh.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
  fsm.land_client = nh.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/land");

  ros::Duration(0.5).sleep();

  if (param.takeoff_land.no_RC)
    ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");

  // enter fsm.process() with a fixed frequency
  ros::Rate r(param.ctrl_freq_max);
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
  }

  return 0;
}
