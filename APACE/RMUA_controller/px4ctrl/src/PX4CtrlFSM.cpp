#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>
// #include <airsim_ros/AngleRateThrottle.h>
// #include <airsim_ros/PoseCmd.h>
#include <uav_utils/geometry_utils.h>

using namespace std;
using namespace uav_utils;

const Eigen::Matrix3d ned_to_enu_rot_mat = []
{
  Eigen::Matrix3d mat;
  mat << 1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0;
  return mat;
}();

const Eigen::Matrix3d airsim_body_to_px4_body_mat = []
{
  Eigen::Matrix3d mat;
  mat << 1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0;
  return mat;
}();

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, Controller &controller_) : param(param_), controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
  state = ON_GROUND;
  hover_pose.setZero();
  takeoff.request.waitOnLastTask = 1;
  land.request.waitOnLastTask = 1;
  takeoff_land_state_msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
}

void PX4CtrlFSM::process()
{
  ros::Time now_time = ros::Time::now();
  Controller_Output_t u;
  Desired_State_t des(odom_data);
  bool rotor_low_speed_during_land = false;

  // STEP1: state machine runs
  switch (state)
  {
  case ON_GROUND:
  {
    if (!odom_is_received(now_time))
    {
      ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
      break;
    }
    if (cmd_is_received(now_time))
    {
      ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. You are sending commands before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
      break;
    }
    if (odom_data.v.norm() > 0.3)
    {
      ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
      break;
    }
    // if (!get_landed())
    // {
    //     // 如果无人机不在地面上，自动跳转到 AUTO_LAND
    //     ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
    //     ROS_INFO("\033[32m[px4ctrl] ON_GROUND(L1) --> AUTO_LAND\033[32m");
    //     state = AUTO_LAND;
    //     break;
    // }
    state = AUTO_TAKEOFF;
    ros::Duration(1.0).sleep();
    controller.resetThrustMapping();
    if (param.pose_solver == 3)
    {
      // while (!controller.ref_mpc.data_ref.empty()) {
      // controller.ref_mpc.data_ref.pop();
      // std::cout<<"队列长度：" << controller.ref_mpc.data_ref.size() << std::endl;
      // }
      std::queue<state_ref> emptyQueue;
      std::swap(controller.ref_mpc.data_ref, emptyQueue);
      std::cout << "清空队列" << std::endl;
    }
    set_start_pose_for_takeoff_land(odom_data);
    ROS_INFO("\033[32m[px4ctrl] ON_GROUND(L1) --> AUTO_TAKEOFF\033[32m");
    break;
  }
  case AUTO_HOVER:
  {
    if (!odom_is_received(now_time)) // 检查odom
    {
      ROS_ERROR("[px4ctrl] Reject CMD_CTRL, no odom");
    }
    else if (cmd_is_received(now_time))
    { // 跳转到CMD_CTRL
      state = CMD_CTRL;
      des = get_cmd_des();
      if (param.pose_solver == 3)
      {

        // while (!controller.ref_mpc.data_ref.empty()) {
        // controller.ref_mpc.data_ref.pop();
        // std::cout<<"队列长度：" << controller.ref_mpc.data_ref.size() << std::endl;
        // }
        std::queue<state_ref> emptyQueue;
        std::swap(controller.ref_mpc.data_ref, emptyQueue);
        std::cout << "清空队列" << std::endl;
      }
      ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
    }
    // TODO: 重新写跳转到AUTO_LAND的条件
    else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
    { // 跳转到AUTO_LAND
      state = AUTO_LAND;
      set_start_pose_for_takeoff_land(odom_data);
      ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
    }
    else
    {
      des = get_hover_des();
      ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
    }
    break;
  }
  case AUTO_TAKEOFF:
  {
    ROS_INFO("Taking off, height%f", odom_data.p(2));
    if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) // Wait for several seconds to warn prople.
    {
      des = get_rotor_speed_up_des(now_time);
    }
    else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) // reach the desired height
    {
      state = AUTO_HOVER;
      set_hov_with_odom();
      ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");
      ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
      ros::Duration(5).sleep();

      takeoff_land_state_msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::HOVERING;
      // 发布是否进入盘旋
    }
    else
    {
      // 如果没有到达目标高度
      des = get_takeoff_land_des(param.takeoff_land.speed);
      ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF: Taking off, height: %f.\033[32m", odom_data.p(2));
    }

    break;
  }
  case AUTO_LAND:
  {
    if (!odom_is_received(now_time))
    {
      ROS_ERROR("[px4ctrl] no odom, current state: AUTO_LAND");
    }
    else if (!get_landed())
    {
      des = get_takeoff_land_des(-param.takeoff_land.speed);
      if (!takeoff_land_data.land_flag)
      {
        land_client.call(land);
        takeoff_land_data.land_flag = true;
      }
    }
    else
    {
      takeoff_land_data.takeoff_flag = false;
      rotor_low_speed_during_land = true;
      static bool print_once_flag = true;
      state = ON_GROUND;
    }

    break;
  }
  case CMD_CTRL:
  {
    // TODO: 重新考虑odom丢失的情况
    if (!odom_is_received(now_time))
    {
      ROS_ERROR("[px4ctrl] no odom, current state: CMD_CTRL");
    }
    else if (!cmd_is_received(now_time))
    {
      state = AUTO_HOVER;
      set_hov_with_odom();
      des = get_hover_des();
      ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
    }
    else
      des = get_cmd_des();

    if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
    {
      ROS_ERROR("[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
					Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
                param.msg_timeout.cmd);
    }
  }
  default:
    break;
  }
  std::cout << "switch" << rotor_low_speed_during_land << std::endl;
  // STEP2: estimate thrust model
  // no need in rotor-drag algorithm

  // STEP3: solve and update new control commands
  if (rotor_low_speed_during_land)
  {
    std::cout << "motors_idling-front" << std::endl;
    motors_idling(imu_data, u);
    // std::cout << "motors_idling" << std::endl;
  }
  else
  {
    std::cout << "param.pose_solver" << param.pose_solver << std::endl;
    switch (param.pose_solver)
    {
    case 1:
      ROS_INFO_ONCE("Using algorithm from ZHEPEI WANG");
      break;

    case 2:
      ROS_INFO_ONCE("Using algorithm from rotor-drag");
      controller.update_alg2(des, odom_data, imu_data, u);
      break;
    case 3:
      ROS_INFO_ONCE("Using algorithm from INDI-MPC");
      controller.update_alg_nmpc(des, odom_data, imu_data, u);
      break;

    default:
      ROS_ERROR("Illegal pose_solver selection!");
      break;
    }
  }

  // STEP4: publish control commands to airsim
  if (param.use_bodyrate_ctrl)
  {
    Controller_Output_t u_out = u;
    // u_out.bodyrates = airsim_body_to_px4_body_mat * u_out.bodyrates;
    publish_bodyrate_ctrl(u_out, now_time);
  }
  else
  {
    Controller_Output_t u_out = u;
    // ENU四元数转回 NED
    Eigen::Matrix3d rot = (airsim_body_to_px4_body_mat * u_out.q.toRotationMatrix() * ned_to_enu_rot_mat);
    u_out.q = Eigen::Quaterniond(rot);
    publish_attitude_ctrl(u_out, now_time);
  }

  // STEP5: Detect if the drone has landed
  land_detector(state, des, odom_data);
  if (state == AUTO_HOVER)
    takeoff_state_pub.publish(takeoff_land_state_msg);

  // STEP6: Clear flags beyond their lifetime
  takeoff_land_data.triggered = false;
}

void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
  u.q = imu.q;
  u.bodyrates = Eigen::Vector3d::Zero();
  u.thrust = 0.04;
}

void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
  static State_t last_state = State_t::ON_GROUND;
  if (last_state == State_t::ON_GROUND && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
  {
    takeoff_land.landed = false; // Always holds
  }
  last_state = state;

  if (state == State_t::ON_GROUND)
  {
    takeoff_land.landed = true;
    return; // No need of other decisions
  }

  // land_detector parameters
  constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
  constexpr double VELOCITY_THR_C = 0.1;        // Constraint 2: velocity below VELOCITY_MIN_C m/s.
  constexpr double TIME_KEEP_C = 3.0;           // Constraint 3: Time(s) the Constraint 1&2 need to keep.

  static ros::Time time_C12_reached; // time_Constraints12_reached
  static bool is_last_C12_satisfy;
  if (takeoff_land.landed)
  {
    time_C12_reached = ros::Time::now();
    is_last_C12_satisfy = false;
  }
  else
  {
    bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
    if (C12_satisfy && !is_last_C12_satisfy)
    {
      time_C12_reached = ros::Time::now();
    }
    else if (C12_satisfy && is_last_C12_satisfy)
    {
      if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) // Constraint 3 reached
      {
        takeoff_land.landed = true;
      }
    }

    is_last_C12_satisfy = C12_satisfy;
  }
}

Desired_State_t PX4CtrlFSM::get_hover_des()
{
  Desired_State_t des;
  des.p = hover_pose.head<3>();
  des.v = Eigen::Vector3d::Zero();
  des.a = Eigen::Vector3d::Zero();
  des.j = Eigen::Vector3d::Zero();
  des.yaw = hover_pose(3);
  des.yaw_rate = 0.0;

  return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
  Desired_State_t des;
  des.p = cmd_data.p;
  des.v = cmd_data.v;
  des.a = cmd_data.a;
  des.j = cmd_data.j;
  des.yaw = cmd_data.yaw;
  des.yaw_rate = cmd_data.yaw_rate;

  return des;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const ros::Time now)
{
  double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
  double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
  if (des_a_z > 0.1)
  {
    ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
    des_a_z = 0.0;
  }

  Desired_State_t des;
  des.p = takeoff_land.start_pose.head<3>();
  des.v = Eigen::Vector3d::Zero();
  des.a = Eigen::Vector3d(0, 0, des_a_z);
  des.j = Eigen::Vector3d::Zero();
  des.yaw = takeoff_land.start_pose(3);
  des.yaw_rate = 0.0;

  return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed)
{
  ros::Time now = ros::Time::now();
  double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff
  double a_ss = 1;
  // takeoff_land.last_set_cmd_time = now;

  // takeoff_land.start_pose(2) += speed * delta_t;

  Desired_State_t des;
  if (param.take_off == 1)
  {
    // 垂直起降
    des.j = Eigen::Vector3d(0, 0, 0);
    des.a = Eigen::Vector3d(0, 0, 0);
    des.v = Eigen::Vector3d(0, 0, speed);
    des.p = Eigen::Vector3d(0, 0, speed * delta_t + takeoff_land.start_pose(2));
    des.yaw = takeoff_land.start_pose(3);
    des.yaw_rate = 0.0;
  }
  else if (param.take_off == 2)
  {
    // 梯形加速度规划
    double T_ss = param.T_ss;
    double j_ss = param.J_ss;
    if (delta_t < T_ss)
    {
      des.j = Eigen::Vector3d(1, 0, 1) * j_ss;
      des.a = Eigen::Vector3d(delta_t, 0, delta_t) * j_ss;
      double v_ss = 0.5 * delta_t * delta_t * j_ss;
      des.v = Eigen::Vector3d(v_ss, 0, v_ss);
      double p_ss = 1.0 / 6.0 * delta_t * delta_t * delta_t * j_ss;
      // ROS_WARN("delta_t: %f p_ss, 0-2 %f", delta_t, p_ss);
      des.p = Eigen::Vector3d(p_ss, 0, p_ss);
      des.yaw = takeoff_land.start_pose(3);
      des.yaw_rate = 0.0;
    }
    else if (delta_t >= T_ss && delta_t <= 2 * T_ss)
    {
      des.j = Eigen::Vector3d(-1, 0, -1);
      des.a = Eigen::Vector3d(2 * T_ss - delta_t, 0, 2 * T_ss - delta_t);
      double v_ss = -0.5 * delta_t * delta_t + 2 * delta_t * T_ss - T_ss * T_ss;
      des.v = Eigen::Vector3d(v_ss, 0, v_ss);
      double p_ss = -1.0 / 6.0 * delta_t * delta_t * delta_t + T_ss * delta_t * delta_t - T_ss * T_ss * delta_t + 1.0 / 3.0 * T_ss * T_ss * T_ss;
      des.p = Eigen::Vector3d(p_ss, 0, p_ss);
      des.yaw = takeoff_land.start_pose(3);
      des.yaw_rate = 0.0;
    }
    else
    {
      des.j = Eigen::Vector3d(0, 0, 0);
      des.a = Eigen::Vector3d(0, 0, 0);
      des.v = Eigen::Vector3d(T_ss * T_ss, 0, T_ss * T_ss);
      double p_ss = T_ss * T_ss * delta_t + 4.0 / 3.0 * T_ss * T_ss * T_ss;
      des.p = Eigen::Vector3d(p_ss, 0, p_ss);
      des.yaw = takeoff_land.start_pose(3);
      des.yaw_rate = 0.0;
    }
  }
  ROS_WARN("des.p, [%f, %f, %f]", des.p(0), des.p(1), des.p(2));

  return des;
}

void PX4CtrlFSM::set_hov_with_odom()
{
  hover_pose.head<3>() = odom_data.p;
  hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

  last_set_hover_pose_time = ros::Time::now();
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
  takeoff_land.start_pose.head<3>() = odom_data.p;
  takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

  takeoff_land.toggle_takeoff_land_time = ros::Time::now();
  ROS_INFO("\033[32m[px4ctrl] SET_SUCUSESS, POS %.4f %.4f %.4f YAW %.4f.\033[32m",takeoff_land.start_pose(0),takeoff_land.start_pose(1),takeoff_land.start_pose(2),takeoff_land.start_pose(3));
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time)
{
  return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time)
{
  return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time)
{
  return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::recv_new_odom()
{
  if (odom_data.recv_new_msg)
  {
    odom_data.recv_new_msg = false;
    return true;
  }

  return false;
}

void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
  // airsim_ros::AngleRateThrottle msg;
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = std::string("FCU");

  msg.body_rate.x = u.bodyrates.x();
  msg.body_rate.y = u.bodyrates.y();
  msg.body_rate.z = u.bodyrates.z();

  msg.thrust = u.thrust;

  ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
  // airsim_ros::PoseCmd msg;

  // Eigen::Vector3d euler_angle = quaternion_to_ypr(u.q);
  // msg.yaw = euler_angle(0);
  // msg.pitch = euler_angle(1);
  // msg.roll = euler_angle(2);
  // msg.throttle = u.thrust;

  // ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.pose = odom_msg.pose.pose;

  traj_start_trigger_pub.publish(msg);
}